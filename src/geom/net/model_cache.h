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
//       entry.txt                      source and model file names, format, member
//   urls/<sha256 of the URL>           the content SHA-256 last fetched from that URL
//   .staging/<random>/                 entries being built (the download temp dir)
//
// An entry is built in .staging and renamed into place only when complete, so
// a crash never leaves a half-written entry under its final name.
//
// Nothing in the cache is trusted. entry.txt and the URL index are hints: the
// only binding is the content address itself. Every lookup re-hashes the
// source file against the name of its directory; the caller then derives the
// model from that verified source again (fetch_model: the model of a plain
// download is the source itself, the model of a zip is compared with a fresh
// extraction under the current limits). Links are never followed: the
// urls/ and .staging/ directories and every entry directory must be real
// directories (a symbolic link or a junction is refused).
namespace orrery::geom::net::detail {

struct CacheEntry {
    std::filesystem::path source_file; // the download, verified against sha256
    std::filesystem::path model_file;  // not verified by the cache (see above)
    std::string sha256;                // content address (the download)
    ModelFormat format = ModelFormat::unknown;
    std::string archive_member;
};

class ModelCache {
public:
    explicit ModelCache(std::filesystem::path root) : root_(std::move(root)) {}

    // Creates the root, urls/ and .staging/ directories; fails if urls/ or
    // .staging/ is a link (symbolic link or junction) instead of a directory.
    [[nodiscard]] std::optional<FetchError> prepare() const;

    // The entry last fetched from `url`, if any, with its source file
    // verified against its content address. The model is the caller's to
    // verify.
    [[nodiscard]] std::optional<CacheEntry> lookup_url(std::string_view url) const;

    // A new empty staging directory.
    [[nodiscard]] core::Result<std::filesystem::path, FetchError> make_staging() const;

    // Writes entry.txt into `staging` (which holds the verified download
    // `source_name` and the model `model_name`) and moves it to
    // <root>/<sha256>. An existing entry with that address is kept only if it
    // holds byte-identical source and model files under the same names;
    // anything else there (a damaged or altered entry, a link) is replaced.
    [[nodiscard]] core::Result<CacheEntry, FetchError>
    commit(const std::filesystem::path& staging, const std::string& sha256,
           const std::filesystem::path& source_name, ModelFormat format,
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
