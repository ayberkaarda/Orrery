#include "orrery/geom/net/model_fetch.h"

#include "https_download.h"
#include "model_cache.h"
#include "orrery/geom/net/url_policy.h"

#include <cstdlib>
#include <optional>
#include <system_error>
#include <utility>

namespace orrery::geom::net {

namespace {

using core::failure;

constexpr std::string_view kDownloadName = "download.part";

// Removes the staging directory unless it was committed to the cache.
class StagingGuard {
public:
    explicit StagingGuard(std::filesystem::path dir) : dir_(std::move(dir)) {}
    StagingGuard(const StagingGuard&) = delete;
    StagingGuard& operator=(const StagingGuard&) = delete;
    StagingGuard(StagingGuard&&) = delete;
    StagingGuard& operator=(StagingGuard&&) = delete;
    ~StagingGuard() {
        std::error_code ec;
        std::filesystem::remove_all(dir_, ec);
    }

private:
    std::filesystem::path dir_;
};

FetchedModel to_result(detail::CacheEntry entry, std::string_view url, const FetchOptions& options,
                       bool from_cache) {
    return FetchedModel{std::move(entry.model_file),
                        std::move(entry.sha256),
                        entry.format,
                        std::string{url},
                        options.license_note,
                        std::move(entry.archive_member),
                        from_cache};
}

std::filesystem::path source_name_for(ModelFormat format) {
    return "source" + std::string{model_format_extension(format)};
}

// A cache hit is served only if the cached download would pass this request's
// policy as a fresh download: the size limit, the content sniffing, and for a
// zip the archive limits. The model is derived from the source again rather
// than trusted: a plain download is its own model, and the model of a zip
// must be byte-identical to a fresh extraction under the current limits.
// On false the caller downloads again, so the outcome (success or the
// policy error) is the one an uncached fetch would give.
bool hit_conforms(const detail::CacheEntry& entry, const detail::ModelCache& cache,
                  const FetchOptions& options) {
    std::error_code ec;
    const std::uintmax_t size = std::filesystem::file_size(entry.source_file, ec);
    if (ec || size > options.limits.max_download_bytes) {
        return false;
    }
    const auto detected = detect_file_format(entry.source_file);
    if (!detected || entry.source_file.filename() != source_name_for(detected.value())) {
        return false;
    }
    const ModelFormat format = detected.value();
    if (format != ModelFormat::zip) {
        return is_model_format(format) && entry.format == format &&
               entry.model_file == entry.source_file && entry.archive_member.empty();
    }
    auto staging = cache.make_staging();
    if (!staging) {
        return false;
    }
    const StagingGuard guard{staging.value()};
    const auto models =
        extract_archive_models(entry.source_file, staging.value(), options.limits.archive);
    if (!models) {
        return false;
    }
    const ExtractedModel& first = models.value().front();
    const std::string fresh = detail::sha256_of_file(first.file);
    return !fresh.empty() && entry.format == first.format &&
           entry.archive_member == first.member_name &&
           entry.model_file.filename() == first.file.filename() &&
           detail::sha256_of_file(entry.model_file) == fresh;
}

#ifdef _WIN32
std::optional<std::filesystem::path> environment_path(const wchar_t* name) {
    wchar_t* value = nullptr;
    std::size_t length = 0;
    if (_wdupenv_s(&value, &length, name) != 0 || value == nullptr) {
        return std::nullopt;
    }
    std::filesystem::path path{value};
    std::free(value); // NOLINT(cppcoreguidelines-no-malloc): allocated by _wdupenv_s
    if (path.empty() || !path.is_absolute()) {
        return std::nullopt;
    }
    return path;
}
#else
std::optional<std::filesystem::path> environment_path(const char* name) {
    const char* value = std::getenv(name); // NOLINT(concurrency-mt-unsafe): read-only use
    if (value == nullptr || *value == '\0') {
        return std::nullopt;
    }
    std::filesystem::path path{value};
    if (!path.is_absolute()) {
        return std::nullopt;
    }
    return path;
}
#endif

} // namespace

std::filesystem::path default_model_cache_dir() {
#ifdef _WIN32
    if (auto local = environment_path(L"LOCALAPPDATA")) {
        return *local / "orrery" / "cache" / "models";
    }
#elif defined(__APPLE__)
    if (auto home = environment_path("HOME")) {
        return *home / "Library" / "Caches" / "orrery" / "models";
    }
#else
    if (auto xdg = environment_path("XDG_CACHE_HOME")) {
        return *xdg / "orrery" / "models";
    }
    if (auto home = environment_path("HOME")) {
        return *home / ".cache" / "orrery" / "models";
    }
#endif
    std::error_code ec;
    return std::filesystem::temp_directory_path(ec) / "orrery" / "models";
}

core::Result<FetchedModel, FetchError> fetch_model(std::string_view url,
                                                   const FetchOptions& options) {
    // Rejected URLs never touch the cache or the network.
    auto checked = check_https_url(url);
    if (!checked) {
        return failure(copy_error(checked.error()));
    }
    const detail::ModelCache cache{options.cache_dir.empty() ? default_model_cache_dir()
                                                             : options.cache_dir};
    if (auto problem = cache.prepare()) {
        return failure(*std::move(problem));
    }
    if (options.use_cache) {
        auto hit = cache.lookup_url(url);
        if (hit && hit_conforms(*hit, cache, options)) {
            return to_result(*std::move(hit), url, options, true);
        }
    }

    auto staging = cache.make_staging();
    if (!staging) {
        return failure(copy_error(staging.error()));
    }
    const std::filesystem::path dir = staging.value();
    const StagingGuard guard{dir};

    const std::filesystem::path download_file = dir / kDownloadName;
    auto download = detail::download_https(url, download_file, options);
    if (!download) {
        return failure(copy_error(download.error()));
    }
    const detail::Download& got = download.value();

    auto detected = detect_file_format(download_file);
    if (!detected) {
        return failure(copy_error(detected.error()));
    }
    const ModelFormat format = detected.value();
    if (format == ModelFormat::unknown) {
        return failure(FetchError{FetchErrorKind::unrecognised_content,
                                  "the downloaded content is not a supported model format "
                                  "(the file name and extension are not trusted)"});
    }
    if (!content_type_matches(got.content_type, format)) {
        return failure(FetchError{FetchErrorKind::mime_rejected,
                                  "Content-Type '" + got.content_type +
                                      "' does not match the downloaded content (" +
                                      std::string{model_format_name(format)} + ")"});
    }

    const std::filesystem::path source_name = source_name_for(format);
    std::error_code ec;
    std::filesystem::rename(download_file, dir / source_name, ec);
    if (ec) {
        return failure(FetchError{FetchErrorKind::io_failure,
                                  "cannot name the downloaded file: " + ec.message()});
    }

    std::filesystem::path model_name = source_name;
    ModelFormat model_format = format;
    std::string member;
    if (format == ModelFormat::zip) {
        auto models = extract_archive_models(dir / source_name, dir, options.limits.archive);
        if (!models) {
            return failure(copy_error(models.error()));
        }
        const ExtractedModel& first = models.value().front();
        model_name = first.file.filename();
        model_format = first.format;
        member = first.member_name;
    }

    auto entry = cache.commit(dir, got.sha256, source_name, model_format, model_name, member);
    if (!entry) {
        return failure(copy_error(entry.error()));
    }
    if (auto problem = cache.remember_url(url, got.sha256)) {
        return failure(*std::move(problem));
    }
    return to_result(std::move(entry).value(), url, options, false);
}

} // namespace orrery::geom::net
