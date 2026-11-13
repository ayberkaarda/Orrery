#include "orrery/sim/aero_import.h"

#include <string>
#include <string_view>
#include <system_error>
#include <utility>

#include "orrery/geom/net/model_fetch.h"

namespace orrery::sim {

namespace {

void report(TaskProgress* progress, const char* stage) {
    if (progress != nullptr) {
        progress->set_stage(stage);
    }
}

std::string import_title(geom::ImportErrorCode code) {
    switch (code) {
        case geom::ImportErrorCode::unsupported_format:
            return "Unsupported model format";
        case geom::ImportErrorCode::file_not_found:
            return "Model file not found";
        case geom::ImportErrorCode::file_too_large:
            return "Model file too large";
        case geom::ImportErrorCode::parse_failed:
            return "The model could not be read";
        case geom::ImportErrorCode::too_many_nodes:
            return "The model has too many nodes";
        case geom::ImportErrorCode::invalid_mesh:
            return "The model failed validation";
    }
    return "Import failed";
}

std::string fetch_title(geom::net::FetchErrorKind kind) {
    using Kind = geom::net::FetchErrorKind;
    switch (kind) {
        case Kind::invalid_url:
        case Kind::scheme_not_allowed:
        case Kind::redirect_not_allowed:
            return "URL refused";
        case Kind::too_many_redirects:
            return "Too many redirects";
        case Kind::tls_failure:
            return "Secure connection failed";
        case Kind::network:
        case Kind::timeout:
        case Kind::http_status:
            return "Download failed";
        case Kind::too_large:
            return "Download too large";
        case Kind::mime_rejected:
        case Kind::unrecognised_content:
            return "Not a supported model file";
        case Kind::archive_bomb:
        case Kind::archive_too_many_entries:
        case Kind::archive_unsafe_entry:
        case Kind::archive_without_model:
        case Kind::archive_invalid:
            return "Archive refused";
        case Kind::cache_failure:
        case Kind::io_failure:
            return "Cannot store the download";
        case Kind::cancelled:
            return "Cancelled";
        case Kind::digest_mismatch:
            return std::string(kModelChangedTitle);
    }
    return "Download failed";
}

ImportResult import_path(const std::filesystem::path& path, scene::GeometryRef ref,
                         TaskProgress* progress) {
    if (cancelled(progress)) {
        return core::failure(cancelled_error());
    }
    report(progress, "importing");
    auto imported = geom::import_mesh_file(path);
    if (!imported) {
        const geom::ImportError& error = imported.error();
        return core::failure(GeometryError{import_title(error.code),
                                           path.filename().string() + ": " + error.message});
    }
    ImportedModel model;
    model.imported = std::move(imported).value();
    model.file_name = path.filename().string();
    ref.unit_scale = model.imported.unit.scale_to_metres;
    ref.up_axis = model.imported.up.axis;
    model.ref = std::move(ref);
    return model;
}

std::filesystem::path resolve_model_path(const scene::GeometryRef& ref,
                                         const std::filesystem::path& base_dir) {
    std::filesystem::path path(ref.path);
    if (path.is_relative() && !base_dir.empty()) {
        path = (base_dir / path).lexically_normal();
    }
    return path;
}

bool model_file_present(const scene::GeometryRef& ref, const std::filesystem::path& base_dir) {
    std::error_code error;
    return !ref.path.empty() &&
           std::filesystem::is_regular_file(resolve_model_path(ref, base_dir), error);
}

// Why a URL model was not downloaded, with what the user needs to decide.
GeometryError download_refusal(const scene::GeometryRef& ref) {
    std::string detail = "the model of this experiment is not in the cache, and opening an "
                         "experiment never downloads on its own. It would be downloaded from ";
    detail += url_host(ref.url);
    detail += " (";
    detail += ref.url;
    detail += ", recorded SHA-256 ";
    detail += ref.sha256.empty() ? std::string_view("none") : std::string_view(ref.sha256);
    detail += "). Confirm the download in the window, or pass --allow-download to a headless "
              "run.";
    return GeometryError{std::string(kDownloadNotAllowedTitle), std::move(detail)};
}

} // namespace

bool cancelled(const TaskProgress* progress) noexcept {
    return progress != nullptr && progress->stop_requested();
}

GeometryError cancelled_error() {
    return {"Cancelled", "the work was cancelled before it finished"};
}

std::filesystem::path default_model_cache() {
    return geom::net::default_model_cache_dir();
}

ImportResult load_model_file(const std::filesystem::path& path, std::string license_note,
                             TaskProgress* progress) {
    scene::GeometryRef ref;
    ref.source = scene::GeometrySource::file;
    ref.path = path.string();
    ref.license_note = std::move(license_note);
    return import_path(path, std::move(ref), progress);
}

ImportResult fetch_model_url(std::string_view url, const UrlImportOptions& options,
                             TaskProgress* progress) {
    if (cancelled(progress)) {
        return core::failure(cancelled_error());
    }
    report(progress, "downloading");
    geom::net::FetchOptions fetch;
    fetch.cache_dir = options.cache_dir;
    fetch.license_note = options.license_note;
    fetch.expected_sha256 = options.expected_sha256;
    fetch.ca_bundle_override = options.ca_bundle_override;
    if (progress != nullptr) {
        // A stop request (Cancel, closing the window) ends even a stalled
        // download within about a second.
        fetch.cancel_requested = [progress] { return progress->stop_requested(); };
    }
    auto fetched = geom::net::fetch_model(url, fetch);
    if (!fetched) {
        const geom::net::FetchError& error = fetched.error();
        if (error.kind == geom::net::FetchErrorKind::cancelled) {
            return core::failure(cancelled_error());
        }
        return core::failure(GeometryError{
            fetch_title(error.kind),
            std::string(geom::net::fetch_error_kind_name(error.kind)) + ": " + error.message});
    }
    const geom::net::FetchedModel& model = fetched.value();
    scene::GeometryRef ref;
    ref.source = scene::GeometrySource::url;
    ref.url = model.source_url;
    ref.path = model.local_path.string();
    ref.sha256 = model.sha256;
    ref.license_note = model.license_note;
    auto imported = import_path(model.local_path, std::move(ref), progress);
    if (imported && !model.archive_member.empty()) {
        imported.value().file_name = model.archive_member;
    }
    return imported;
}

bool needs_download(const scene::GeometryRef& ref, const std::filesystem::path& base_dir) {
    return ref.source == scene::GeometrySource::url && !model_file_present(ref, base_dir);
}

std::string url_host(std::string_view url) {
    const std::size_t scheme = url.find("://");
    if (scheme == std::string_view::npos) {
        return {};
    }
    std::string_view authority = url.substr(scheme + 3);
    authority = authority.substr(0, authority.find_first_of("/?#"));
    if (const std::size_t at = authority.rfind('@'); at != std::string_view::npos) {
        authority = authority.substr(at + 1);
    }
    if (!authority.starts_with("[")) {
        authority = authority.substr(0, authority.find(':'));
    }
    return std::string(authority);
}

PrepareResult load_experiment_geometry(const scene::GeometryRef& ref,
                                       const std::filesystem::path& base_dir,
                                       TaskProgress* progress, const ExperimentDownload& download) {
    if (ref.source == scene::GeometrySource::primitive) {
        report(progress, "building");
        return build_primitive(ref);
    }
    const std::filesystem::path path = resolve_model_path(ref, base_dir);
    const bool present = model_file_present(ref, base_dir);
    ImportResult imported = [&]() -> ImportResult {
        if (ref.source == scene::GeometrySource::url && !present) {
            if (!download.allowed) {
                return core::failure(download_refusal(ref));
            }
            // The loader compares the recorded digest before anything reads
            // the bytes, so a changed model never reaches the importer.
            UrlImportOptions options = download.fetch;
            options.license_note = ref.license_note;
            options.expected_sha256 = ref.sha256;
            return fetch_model_url(ref.url, options, progress);
        }
        scene::GeometryRef file_ref = ref;
        file_ref.path = path.string();
        return import_path(path, std::move(file_ref), progress);
    }();
    if (!imported) {
        const GeometryError& cause = imported.error();
        return core::failure(GeometryError{cause.title, cause.detail});
    }
    if (cancelled(progress)) {
        return core::failure(cancelled_error());
    }
    report(progress, "repairing");
    ImportedModel model = std::move(imported).value();
    model.ref.license_note = ref.license_note;
    model.ref.url = ref.url.empty() ? model.ref.url : ref.url;
    auto prepared = prepare_user_model(model, ref.unit_scale, ref.up_axis);
    if (prepared && present) {
        // Keep the reference as recorded (the path as written in the file);
        // a fresh download keeps the new cache path instead.
        prepared.value().ref.path = ref.path;
        prepared.value().ref.sha256 = ref.sha256;
    }
    return prepared;
}

} // namespace orrery::sim
