#include "orrery/geom/net/ca_certificates.h"

#include <cstdlib>
#include <system_error>
#include <utility>

namespace orrery::geom::net {

namespace {

using core::failure;

// OpenSSL's separator for a list of certificate directories.
#ifdef _WIN32
constexpr char kListSeparator = ';';
#else
constexpr char kListSeparator = ':';
#endif

FetchError tls_error(std::string message) {
    return FetchError{FetchErrorKind::tls_failure, std::move(message)};
}

bool is_bundle(const std::filesystem::path& path) {
    std::error_code error;
    if (!std::filesystem::is_regular_file(path, error)) {
        return false;
    }
    const auto size = std::filesystem::file_size(path, error);
    return !error && size > 0;
}

bool is_existing_directory(const std::filesystem::path& path) {
    std::error_code error;
    return std::filesystem::is_directory(path, error);
}

bool is_hex_digit(char c) noexcept {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
}

// OpenSSL's name for a hashed certificate: 8 hex digits, '.', a number.
bool is_hashed_name(std::string_view name) noexcept {
    constexpr std::size_t kHashLength = 8;
    if (name.size() < kHashLength + 2 || name[kHashLength] != '.') {
        return false;
    }
    for (const char c : name.substr(0, kHashLength)) {
        if (!is_hex_digit(c)) {
            return false;
        }
    }
    for (const char c : name.substr(kHashLength + 1)) {
        if (c < '0' || c > '9') {
            return false;
        }
    }
    return true;
}

bool holds_hashed_certificate(const std::filesystem::path& directory) {
    std::error_code error;
    std::filesystem::directory_iterator it{directory, error};
    for (; !error && it != std::filesystem::directory_iterator{}; it.increment(error)) {
        if (is_hashed_name(it->path().filename().string())) {
            return true;
        }
    }
    return false;
}

// True if at least one entry of a list (kListSeparator) is a directory.
bool lists_a_directory(std::string_view list) {
    while (!list.empty()) {
        const auto end = list.find(kListSeparator);
        const std::string_view entry = list.substr(0, end);
        if (!entry.empty() && is_existing_directory(std::filesystem::path{entry})) {
            return true;
        }
        if (end == std::string_view::npos) {
            break;
        }
        list.remove_prefix(end + 1);
    }
    return false;
}

core::Result<CaCertificates, FetchError> from_environment(const CaEnvironment& environment) {
    CaCertificates found;
    const bool curl_bundle = !environment.curl_ca_bundle.empty();
    const std::string& bundle =
        curl_bundle ? environment.curl_ca_bundle : environment.ssl_cert_file;
    const char* const bundle_variable = curl_bundle ? "CURL_CA_BUNDLE" : "SSL_CERT_FILE";
    if (!bundle.empty()) {
        if (!is_bundle(std::filesystem::path{bundle})) {
            return failure(tls_error(std::string{bundle_variable} + " names '" + bundle +
                                     "', which is not a non-empty file"));
        }
        found.bundle = std::filesystem::path{bundle};
        found.origin = std::string{bundle_variable} + "=" + bundle;
    }
    if (!environment.ssl_cert_dir.empty()) {
        if (!lists_a_directory(environment.ssl_cert_dir)) {
            return failure(tls_error("SSL_CERT_DIR names '" + environment.ssl_cert_dir +
                                     "', which holds no directory"));
        }
        found.directories = environment.ssl_cert_dir;
        if (!found.origin.empty()) {
            found.origin += ", ";
        }
        found.origin += "SSL_CERT_DIR=" + environment.ssl_cert_dir;
    }
    return found;
}

} // namespace

#ifdef _WIN32
CaEnvironment ca_environment_from_process() {
    return {};
}
#else
namespace {
std::string environment_value(const char* name) {
    const char* value = std::getenv(name); // NOLINT(concurrency-mt-unsafe): read-only use
    return value == nullptr ? std::string{} : std::string{value};
}
} // namespace

CaEnvironment ca_environment_from_process() {
    return CaEnvironment{environment_value("CURL_CA_BUNDLE"), environment_value("SSL_CERT_FILE"),
                         environment_value("SSL_CERT_DIR")};
}
#endif

core::Result<CaCertificates, FetchError>
find_ca_certificates(const CaEnvironment& environment, std::span<const std::string_view> bundles,
                     std::span<const std::string_view> directories) {
    if (!environment.curl_ca_bundle.empty() || !environment.ssl_cert_file.empty() ||
        !environment.ssl_cert_dir.empty()) {
        return from_environment(environment);
    }
    std::string tried;
    const auto note = [&tried](std::string_view candidate) {
        if (!tried.empty()) {
            tried += ", ";
        }
        tried += candidate;
    };
    for (const std::string_view bundle : bundles) {
        if (is_bundle(std::filesystem::path{bundle})) {
            return CaCertificates{std::filesystem::path{bundle}, {}, std::string{bundle}};
        }
        note(bundle);
    }
    for (const std::string_view directory : directories) {
        if (holds_hashed_certificate(std::filesystem::path{directory})) {
            return CaCertificates{{}, std::string{directory}, std::string{directory}};
        }
        note(std::string{directory} + "/ (hashed certificates)");
    }
    return failure(tls_error("no CA certificates to verify the server: CURL_CA_BUNDLE, "
                             "SSL_CERT_FILE and SSL_CERT_DIR are unset, and none of these "
                             "was found: " +
                             (tried.empty() ? std::string{"(no candidates)"} : tried) +
                             ". Install the distribution's CA certificates package or set "
                             "SSL_CERT_FILE to a PEM bundle"));
}

} // namespace orrery::geom::net
