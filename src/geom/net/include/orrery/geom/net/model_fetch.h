#pragma once

#include "orrery/core/result.h"
#include "orrery/geom/net/fetch_error.h"
#include "orrery/geom/net/model_format.h"
#include "orrery/geom/net/zip_extract.h"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>

// Secure model download from a URL (ADR-010, docs/url-import-security.md).
//
// fetch_model() accepts only https, also on every redirect hop; enforces the
// size cap while streaming; checks the Content-Type and the magic bytes (the
// file extension is never trusted); hashes the bytes with SHA-256 and stores
// them in a content-addressed cache; opens a single level of zip with
// decompression-bomb limits. It does not parse the model: the returned file
// is still untrusted input for the importer.
namespace orrery::geom::net {

struct FetchLimits {
    std::uint64_t max_download_bytes = std::uint64_t{200} * 1024 * 1024;
    std::uint32_t max_redirects = 5;
    std::chrono::milliseconds connect_timeout{15'000};
    // Whole download, all redirect hops included.
    std::chrono::milliseconds total_timeout{300'000};
    // The transfer is aborted when fewer than low_speed_bytes_per_second
    // arrive during low_speed_window.
    std::uint32_t low_speed_bytes_per_second = 1024;
    std::chrono::seconds low_speed_window{30};
    ArchiveLimits archive;
};

struct FetchOptions {
    FetchLimits limits;
    // Cache root; empty selects default_model_cache_dir().
    std::filesystem::path cache_dir;
    // false skips the URL lookup and always downloads (the result is still
    // stored in the cache).
    bool use_cache = true;
    // Licence or source note entered by the user; carried into the result.
    // The program does not check it.
    std::string license_note;
    // Honour http_proxy/https_proxy/no_proxy from the environment.
    bool use_environment_proxy = true;
    // Test seam: a PEM file whose certificates replace the system trust
    // store. Empty (the default, and the only value production code uses)
    // verifies servers against the system store. Peer and host name
    // verification are never disabled.
    std::filesystem::path ca_bundle_override;
};

struct FetchedModel {
    // The model file: the download itself, or the first model extracted from
    // a zip download. Lives in the cache; its extension follows the detected
    // content.
    std::filesystem::path local_path;
    // SHA-256 of the downloaded bytes (lower-case hex), the cache key.
    std::string sha256;
    ModelFormat format = ModelFormat::unknown;
    std::string source_url;
    std::string license_note;
    // Name inside the archive for a zip download, empty otherwise.
    std::string archive_member;
    bool from_cache = false;
};

[[nodiscard]] core::Result<FetchedModel, FetchError> fetch_model(std::string_view url,
                                                                 const FetchOptions& options);

// Platform cache root for downloaded models:
//   Linux and other Unix: $XDG_CACHE_HOME/orrery/models, else ~/.cache/orrery/models
//   macOS: ~/Library/Caches/orrery/models
//   Windows: %LOCALAPPDATA%\orrery\cache\models
// Falls back to <temp>/orrery/models when no home directory is known.
[[nodiscard]] std::filesystem::path default_model_cache_dir();

} // namespace orrery::geom::net
