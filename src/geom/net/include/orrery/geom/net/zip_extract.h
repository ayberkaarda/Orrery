#pragma once

#include "orrery/core/result.h"
#include "orrery/geom/net/fetch_error.h"
#include "orrery/geom/net/model_format.h"

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

// Single-level zip support for downloaded models. The archive is untrusted:
// every entry is validated before anything is extracted, and only entries
// whose content is a supported model format are written to disk. Nested
// archives are not opened.
namespace orrery::geom::net {

struct ArchiveLimits {
    std::uint32_t max_entries = 1000;
    std::uint64_t max_total_uncompressed_bytes = std::uint64_t{200} * 1024 * 1024;
    // An entry whose uncompressed size exceeds this multiple of its
    // compressed size is a decompression bomb (ADR-010).
    std::uint64_t max_compression_ratio = 100;
};

struct ExtractedModel {
    std::filesystem::path file; // inside the output directory
    ModelFormat format = ModelFormat::unknown;
    std::string member_name; // the name inside the archive, for display only
};

// True if an archive member name is a plain relative path: not empty, no
// control characters, no leading '/' or '\', no drive letter or other ':',
// and no ".." component (either separator).
[[nodiscard]] bool is_safe_member_name(std::string_view name) noexcept;

// Validates the whole archive (entry count, names, symlinks, encryption,
// declared sizes and compression ratios), then extracts the model entries
// into `out_dir` (which must exist) under generated names; archive paths are
// never used as file system paths. While extracting, the real size of each
// entry is counted and an entry larger than declared is rejected as a bomb.
// Models are returned in archive order; an archive without a model fails.
[[nodiscard]] core::Result<std::vector<ExtractedModel>, FetchError>
extract_archive_models(const std::filesystem::path& archive, const std::filesystem::path& out_dir,
                       const ArchiveLimits& limits);

} // namespace orrery::geom::net
