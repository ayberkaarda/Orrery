#include "https_download.h"

#include "orrery/geom/net/ca_certificates.h"
#include "orrery/geom/net/sha256.h"
#include "orrery/geom/net/url_policy.h"

#include <curl/curl.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <fstream>
#include <memory>
#include <span>
#include <utility>

namespace orrery::geom::net::detail {

namespace {

using core::failure;
using Clock = std::chrono::steady_clock;

// Redirect responses are discarded; their bodies are capped separately.
constexpr std::uint64_t kMaxRedirectBody = std::uint64_t{64} * 1024U;

FetchError error(FetchErrorKind kind, std::string message) {
    return FetchError{kind, std::move(message)};
}

bool curl_ready() {
    static const bool ready = curl_global_init(CURL_GLOBAL_DEFAULT) == CURLE_OK;
    return ready;
}

struct CurlDeleter {
    void operator()(CURL* handle) const noexcept { curl_easy_cleanup(handle); }
};
using CurlHandle = std::unique_ptr<CURL, CurlDeleter>;

bool is_redirect_status(long status) noexcept {
    return status == 301 || status == 302 || status == 303 || status == 307 || status == 308;
}

enum class Abort : std::uint8_t { none, too_large, status, mime, io };

// State of one request (one redirect hop).
struct Transfer {
    CURL* handle = nullptr;
    const std::filesystem::path* destination = nullptr;
    std::uint64_t limit = 0;
    std::ofstream out;
    Sha256 hash;
    std::uint64_t bytes = 0;
    long status = 0;
    std::string content_type;
    bool started = false;
    Abort abort = Abort::none;

    // Called once the response headers are known (first body bytes or end of
    // an empty body). Returns false if the response is rejected.
    bool begin() {
        started = true;
        curl_easy_getinfo(handle, CURLINFO_RESPONSE_CODE, &status);
        const char* type = nullptr;
        curl_easy_getinfo(handle, CURLINFO_CONTENT_TYPE, &type);
        content_type = type == nullptr ? std::string{} : std::string{type};
        if (is_redirect_status(status)) {
            return true;
        }
        if (status != 200) {
            abort = Abort::status;
            return false;
        }
        if (!content_type_allowed(content_type)) {
            abort = Abort::mime;
            return false;
        }
        out.open(*destination, std::ios::binary | std::ios::trunc);
        if (!out) {
            abort = Abort::io;
            return false;
        }
        return true;
    }

    std::size_t accept(std::span<const std::byte> data) {
        if (!started && !begin()) {
            return 0;
        }
        const std::uint64_t cap = is_redirect_status(status) ? kMaxRedirectBody : limit;
        if (data.size() > cap - bytes) {
            abort = Abort::too_large;
            return 0;
        }
        bytes += data.size();
        if (is_redirect_status(status)) {
            return data.size();
        }
        hash.update(data);
        out.write(reinterpret_cast<const char*>(data.data()),
                  static_cast<std::streamsize>(data.size()));
        if (!out) {
            abort = Abort::io;
            return 0;
        }
        return data.size();
    }
};

std::size_t on_body(char* data, std::size_t size, std::size_t count, void* user) {
    auto* transfer = static_cast<Transfer*>(user);
    try {
        return transfer->accept(std::span{reinterpret_cast<const std::byte*>(data), size * count});
    } catch (...) {
        transfer->abort = Abort::io;
        return 0;
    }
}

FetchError abort_error(const Transfer& transfer, const FetchOptions& options) {
    switch (transfer.abort) {
        case Abort::too_large:
            return error(FetchErrorKind::too_large,
                         "the download exceeds the size limit of " +
                             std::to_string(options.limits.max_download_bytes) + " bytes");
        case Abort::status:
            return error(FetchErrorKind::http_status,
                         "the server answered HTTP " + std::to_string(transfer.status));
        case Abort::mime:
            return error(FetchErrorKind::mime_rejected,
                         "Content-Type '" + transfer.content_type + "' is not a model type");
        case Abort::io:
        case Abort::none:
            break;
    }
    return error(FetchErrorKind::io_failure, "cannot write the downloaded file");
}

FetchError curl_error(CURLcode code, const Transfer& transfer, const FetchOptions& options,
                      const char* detail) {
    if (code == CURLE_WRITE_ERROR && transfer.abort != Abort::none) {
        return abort_error(transfer, options);
    }
    std::string message = curl_easy_strerror(code);
    if (detail != nullptr && *detail != '\0') {
        message += ": ";
        message += detail;
    }
    switch (code) {
        case CURLE_OPERATION_TIMEDOUT:
            return error(FetchErrorKind::timeout, std::move(message));
        case CURLE_FILESIZE_EXCEEDED:
            return error(FetchErrorKind::too_large,
                         "the declared size exceeds the size limit of " +
                             std::to_string(options.limits.max_download_bytes) + " bytes");
        case CURLE_UNSUPPORTED_PROTOCOL:
            return error(FetchErrorKind::scheme_not_allowed, std::move(message));
        case CURLE_URL_MALFORMAT:
            return error(FetchErrorKind::invalid_url, std::move(message));
        case CURLE_PEER_FAILED_VERIFICATION:
        case CURLE_SSL_CONNECT_ERROR:
        case CURLE_SSL_CERTPROBLEM:
        case CURLE_SSL_CIPHER:
        case CURLE_SSL_CACERT_BADFILE:
        case CURLE_SSL_ISSUER_ERROR:
        case CURLE_SSL_PINNEDPUBKEYNOTMATCH:
        case CURLE_SSL_INVALIDCERTSTATUS:
        case CURLE_SSL_CRL_BADFILE:
            return error(FetchErrorKind::tls_failure, std::move(message));
        default:
            break;
    }
    return error(FetchErrorKind::network, std::move(message));
}

long clamp_ms(std::chrono::milliseconds value) noexcept {
    const auto count = std::clamp<std::chrono::milliseconds::rep>(value.count(), 1, 0x7FFFFFFF);
    return static_cast<long>(count);
}

// Options shared by every hop. Returns false if libcurl rejects one.
// `system_ca` is used outside Windows when no override is set.
bool configure(CURL* handle, const FetchOptions& options,
               [[maybe_unused]] const CaCertificates& system_ca, char* error_buffer) {
    const FetchLimits& limits = options.limits;
    bool ok = true;
    const auto set = [&ok](CURLcode code) { ok = ok && code == CURLE_OK; };
    set(curl_easy_setopt(handle, CURLOPT_ERRORBUFFER, error_buffer));
    set(curl_easy_setopt(handle, CURLOPT_NOSIGNAL, 1L));
    // https only, on the request and (defensively) on redirects, although
    // redirects are followed by download_https itself.
    set(curl_easy_setopt(handle, CURLOPT_PROTOCOLS_STR, "https"));
    set(curl_easy_setopt(handle, CURLOPT_REDIR_PROTOCOLS_STR, "https"));
    set(curl_easy_setopt(handle, CURLOPT_FOLLOWLOCATION, 0L));
    set(curl_easy_setopt(handle, CURLOPT_SSL_VERIFYPEER, 1L));
    set(curl_easy_setopt(handle, CURLOPT_SSL_VERIFYHOST, 2L));
    set(curl_easy_setopt(handle, CURLOPT_SSLVERSION, CURL_SSLVERSION_TLSv1_2));
    // Bodies are stored exactly as sent: no transparent decompression.
    set(curl_easy_setopt(handle, CURLOPT_HTTP_CONTENT_DECODING, 0L));
    set(curl_easy_setopt(handle, CURLOPT_MAXFILESIZE_LARGE,
                         static_cast<curl_off_t>(limits.max_download_bytes)));
    set(curl_easy_setopt(handle, CURLOPT_LOW_SPEED_LIMIT,
                         static_cast<long>(limits.low_speed_bytes_per_second)));
    set(curl_easy_setopt(handle, CURLOPT_LOW_SPEED_TIME,
                         static_cast<long>(std::clamp<std::chrono::seconds::rep>(
                             limits.low_speed_window.count(), 1, 0x7FFFFFFF))));
    set(curl_easy_setopt(handle, CURLOPT_USERAGENT, "Orrery model loader"));
    set(curl_easy_setopt(handle, CURLOPT_WRITEFUNCTION, on_body));
    if (!options.use_environment_proxy) {
        set(curl_easy_setopt(handle, CURLOPT_PROXY, ""));
    }
    if (options.ca_bundle_override.empty()) {
#ifdef _WIN32
        // OpenSSL has no default bundle on Windows: use the system store.
        set(curl_easy_setopt(handle, CURLOPT_SSL_OPTIONS, CURLSSLOPT_NATIVE_CA));
#else
        // The certificates of this machine (find_ca_certificates), never the
        // locations compiled into the static libcurl and OpenSSL.
        const std::string bundle = system_ca.bundle.string();
        set(curl_easy_setopt(handle, CURLOPT_CAINFO, bundle.empty() ? nullptr : bundle.c_str()));
        set(curl_easy_setopt(handle, CURLOPT_CAPATH,
                             system_ca.directories.empty() ? nullptr
                                                           : system_ca.directories.c_str()));
#endif
    } else {
        const std::string bundle = options.ca_bundle_override.string();
        set(curl_easy_setopt(handle, CURLOPT_CAINFO, bundle.c_str()));
        set(curl_easy_setopt(handle, CURLOPT_CAPATH, nullptr));
    }
    return ok;
}

} // namespace

core::Result<Download, FetchError> download_https(std::string_view url,
                                                  const std::filesystem::path& destination,
                                                  const FetchOptions& options) {
    auto checked = check_https_url(url);
    if (!checked) {
        return failure(copy_error(checked.error()));
    }
    CaCertificates system_ca;
#ifndef _WIN32
    if (options.ca_bundle_override.empty()) {
        auto found = find_ca_certificates(ca_environment_from_process(), kDefaultCaBundles,
                                          kDefaultCaDirectories);
        if (!found) {
            return failure(copy_error(found.error()));
        }
        system_ca = std::move(found).value();
    }
#endif
    if (!curl_ready()) {
        return failure(error(FetchErrorKind::network, "libcurl initialisation failed"));
    }
    const CurlHandle handle{curl_easy_init()};
    if (!handle) {
        return failure(error(FetchErrorKind::network, "libcurl handle allocation failed"));
    }
    std::array<char, CURL_ERROR_SIZE> error_buffer{};
    if (!configure(handle.get(), options, system_ca, error_buffer.data())) {
        return failure(error(FetchErrorKind::network, "libcurl rejected the transfer options"));
    }

    const auto deadline = Clock::now() + options.limits.total_timeout;
    std::string current = std::move(checked).value();
    for (std::uint32_t hop = 0;; ++hop) {
        const auto remaining =
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
        if (remaining.count() <= 0) {
            return failure(error(FetchErrorKind::timeout, "the download took too long"));
        }
        Transfer transfer;
        transfer.handle = handle.get();
        transfer.destination = &destination;
        transfer.limit = options.limits.max_download_bytes;
        error_buffer[0] = '\0';
        bool ok = curl_easy_setopt(handle.get(), CURLOPT_URL, current.c_str()) == CURLE_OK;
        ok = ok && curl_easy_setopt(handle.get(), CURLOPT_WRITEDATA, &transfer) == CURLE_OK;
        ok = ok &&
             curl_easy_setopt(handle.get(), CURLOPT_TIMEOUT_MS, clamp_ms(remaining)) == CURLE_OK;
        ok = ok && curl_easy_setopt(
                       handle.get(), CURLOPT_CONNECTTIMEOUT_MS,
                       clamp_ms(std::min(remaining, options.limits.connect_timeout))) == CURLE_OK;
        if (!ok) {
            return failure(error(FetchErrorKind::invalid_url, "libcurl rejected the URL"));
        }

        const CURLcode code = curl_easy_perform(handle.get());
        if (code != CURLE_OK) {
            return failure(curl_error(code, transfer, options, error_buffer.data()));
        }
        if (!transfer.started && !transfer.begin()) {
            return failure(abort_error(transfer, options));
        }
        if (!is_redirect_status(transfer.status)) {
            transfer.out.close();
            if (!transfer.out) {
                return failure(
                    error(FetchErrorKind::io_failure, "cannot write the downloaded file"));
            }
            return Download{transfer.bytes, to_hex(transfer.hash.finish()), transfer.content_type,
                            current, hop};
        }

        // A redirect: validate the target before it is contacted.
        const char* location = nullptr;
        curl_easy_getinfo(handle.get(), CURLINFO_REDIRECT_URL, &location);
        if (location == nullptr) {
            return failure(error(FetchErrorKind::http_status, "redirect " +
                                                                  std::to_string(transfer.status) +
                                                                  " without a usable Location"));
        }
        if (hop + 1 > options.limits.max_redirects) {
            return failure(
                error(FetchErrorKind::too_many_redirects,
                      "more than " + std::to_string(options.limits.max_redirects) + " redirects"));
        }
        auto next = check_https_url(location);
        if (!next) {
            return failure(
                error(FetchErrorKind::redirect_not_allowed,
                      "redirect to a URL that is not allowed (" + next.error().message + ")"));
        }
        current = std::move(next).value();
    }
}

} // namespace orrery::geom::net::detail
