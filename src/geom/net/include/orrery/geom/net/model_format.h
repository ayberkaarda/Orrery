#pragma once

#include "orrery/core/result.h"
#include "orrery/geom/net/fetch_error.h"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string_view>

// Content sniffing for downloaded files. The file name and its extension are
// never consulted: the format is decided by magic bytes and structure only,
// and the server's Content-Type is checked against an allowlist and against
// the detected format.
namespace orrery::geom::net {

enum class ModelFormat : std::uint8_t {
    unknown,
    stl_ascii,
    stl_binary,
    obj,
    gltf, // glTF 2.0 JSON
    glb,  // glTF 2.0 binary container
    ply,
    fbx, // binary or ASCII FBX
    zip, // an archive, never a model by itself
};

[[nodiscard]] std::string_view model_format_name(ModelFormat format) noexcept;

// File extension written for a detected format (".stl", ".glb", ...), so a
// cached file always carries the extension of its real content.
[[nodiscard]] std::string_view model_format_extension(ModelFormat format) noexcept;

// True for formats an importer can open (everything except unknown and zip).
[[nodiscard]] bool is_model_format(ModelFormat format) noexcept;

// Number of leading bytes the detector inspects.
inline constexpr std::size_t kSniffBytes = 64 * 1024;

// Detects the format from the first bytes of a file (`head`, at most
// kSniffBytes) and its total size.
[[nodiscard]] ModelFormat detect_model_format(std::span<const std::byte> head,
                                              std::uint64_t total_size) noexcept;

[[nodiscard]] core::Result<ModelFormat, FetchError>
detect_file_format(const std::filesystem::path& file);

// True if a response with this Content-Type may be downloaded at all
// (generic binary/text types, model types and zip types; an absent header is
// allowed). Parameters such as "; charset=utf-8" are ignored.
[[nodiscard]] bool content_type_allowed(std::string_view content_type) noexcept;

// True if a specific Content-Type agrees with the detected format; generic
// types (application/octet-stream, text/plain, absent) agree with every
// format.
[[nodiscard]] bool content_type_matches(std::string_view content_type, ModelFormat format) noexcept;

} // namespace orrery::geom::net
