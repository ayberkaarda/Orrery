#pragma once

#include "orrery/core/result.h"
#include "orrery/geom/net/fetch_error.h"
#include "orrery/geom/net/model_format.h"

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

// Content-addressed model cache (private to orrery_geom_net).
//
// Layout under the root:
//   <sha256 of the download>/          one entry per distinct content
//       source<ext>                    the downloaded bytes
//       NNNN-<name><ext>               models extracted from a zip download
//       entry.txt                      format, model file and its SHA-256
//   urls/<sha256 of the URL>           the content SHA-256 last fetched from that URL
//   .staging/<random>/                 entries being built (the download temp dir)
//
// An entry is built in .staging and renamed into place only when complete, so
// a crash never leaves a half-written entry under its final name. Every
// lookup re-hashes the model file before returning it.
namespace orrery::geom::net::detail {

struct CacheEntry {
    std::filesystem::path model_file;
    std::string sha256; // content address (the download)
    ModelFormat format = ModelFormat::unknown;
    std::string archive_member;
};

class ModelCache {
public:
    explicit ModelCache(std::filesystem::path root) : root_(std::move(root)) {}

    // Creates the root, urls/ and .staging/ directories.
    [[nodiscard]] std::optional<FetchError> prepare() const;

    // The verified entry last fetched from `url`, if any.
    [[nodiscard]] std::optional<CacheEntry> lookup_url(std::string_view url) const;

    // A new empty staging directory.
    [[nodiscard]] core::Result<std::filesystem::path, FetchError> make_staging() const;

    // Writes entry.txt into `staging` and moves it to <root>/<sha256>. If an
    // intact entry with that address exists already, it is kept and the
    // staging directory is removed.
    [[nodiscard]] core::Result<CacheEntry, FetchError>
    commit(const std::filesystem::path& staging, const std::string& sha256, ModelFormat format,
           const std::filesystem::path& model_name, const std::string& archive_member) const;

    // Records that `url` served the content `sha256`.
    [[nodiscard]] std::optional<FetchError> remember_url(std::string_view url,
                                                         const std::string& sha256) const;

    [[nodiscard]] const std::filesystem::path& root() const noexcept { return root_; }

private:
    [[nodiscard]] std::optional<CacheEntry> read_entry(const std::string& sha256) const;

    std::filesystem::path root_;
};

// SHA-256 of a file, lower-case hex; empty if the file cannot be read.
[[nodiscard]] std::string sha256_of_file(const std::filesystem::path& file);

} // namespace orrery::geom::net::detail
