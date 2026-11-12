#include "orrery/geom/net/model_format.h"

#include <algorithm>
#include <array>
#include <fstream>
#include <string>
#include <system_error>
#include <vector>

namespace orrery::geom::net {

namespace {

using core::failure;

std::uint32_t read_le32(std::span<const std::byte> bytes, std::size_t offset) noexcept {
    return std::to_integer<std::uint32_t>(bytes[offset]) |
           (std::to_integer<std::uint32_t>(bytes[offset + 1]) << 8U) |
           (std::to_integer<std::uint32_t>(bytes[offset + 2]) << 16U) |
           (std::to_integer<std::uint32_t>(bytes[offset + 3]) << 24U);
}

bool starts_with(std::span<const std::byte> bytes, std::string_view prefix) noexcept {
    if (bytes.size() < prefix.size()) {
        return false;
    }
    for (std::size_t i = 0; i < prefix.size(); ++i) {
        if (std::to_integer<unsigned char>(bytes[i]) != static_cast<unsigned char>(prefix[i])) {
            return false;
        }
    }
    return true;
}

// Text means: no NUL and no control characters other than tab, line feed,
// carriage return, vertical tab and form feed. Bytes >= 0x80 are allowed
// (UTF-8 comments and names).
bool is_text(std::string_view text) noexcept {
    return std::all_of(text.begin(), text.end(), [](char c) {
        const auto u = static_cast<unsigned char>(c);
        return u >= 0x20 || u == '\t' || u == '\n' || u == '\r' || u == '\v' || u == '\f';
    });
}

std::string_view trim_left(std::string_view text) noexcept {
    const auto first = text.find_first_not_of(" \t\r\n\v\f");
    return first == std::string_view::npos ? std::string_view{} : text.substr(first);
}

std::string_view trim(std::string_view text) noexcept {
    text = trim_left(text);
    const auto last = text.find_last_not_of(" \t\r\n\v\f");
    return last == std::string_view::npos ? std::string_view{} : text.substr(0, last + 1);
}

std::string_view first_word(std::string_view line) noexcept {
    const auto end = line.find_first_of(" \t");
    return end == std::string_view::npos ? line : line.substr(0, end);
}

bool is_binary_stl(std::span<const std::byte> head, std::uint64_t total_size) noexcept {
    if (total_size < 84 || head.size() < 84) {
        return false;
    }
    const std::uint64_t triangles = read_le32(head, 80);
    return triangles > 0 && 84 + 50 * triangles == total_size;
}

bool is_glb(std::span<const std::byte> head, std::uint64_t total_size) noexcept {
    return head.size() >= 12 && starts_with(head, "glTF") && read_le32(head, 4) == 2 &&
           read_le32(head, 8) == total_size;
}

bool is_ply(std::string_view text) noexcept {
    if (!(text.starts_with("ply\n") || text.starts_with("ply\r\n"))) {
        return false;
    }
    const auto header_end = text.find("end_header");
    const auto format = text.find("\nformat ");
    return format != std::string_view::npos && header_end != std::string_view::npos &&
           format < header_end;
}

// Every complete line of an OBJ file is empty, a comment or a statement of
// the OBJ grammar, and at least one vertex statement exists.
bool is_obj(std::string_view text, bool truncated) noexcept {
    static constexpr std::array<std::string_view, 37> kKeywords{
        "v",     "vt",    "vn",       "vp",       "f",      "l",          "p",         "o",
        "g",     "s",     "mtllib",   "usemtl",   "cstype", "deg",        "curv",      "curv2",
        "surf",  "parm",  "trim",     "hole",     "scrv",   "sp",         "end",       "con",
        "mg",    "bevel", "c_interp", "d_interp", "lod",    "shadow_obj", "trace_obj", "ctech",
        "stech", "bmat",  "step",     "call",     "csh"};
    if (truncated) {
        const auto last_newline = text.rfind('\n');
        text = last_newline == std::string_view::npos ? std::string_view{}
                                                      : text.substr(0, last_newline);
    }
    bool has_vertex = false;
    while (!text.empty()) {
        const auto newline = text.find('\n');
        const std::string_view line =
            trim(newline == std::string_view::npos ? text : text.substr(0, newline));
        text = newline == std::string_view::npos ? std::string_view{} : text.substr(newline + 1);
        if (line.empty() || line.front() == '#') {
            continue;
        }
        const std::string_view word = first_word(line);
        if (std::find(kKeywords.begin(), kKeywords.end(), word) == kKeywords.end()) {
            return false;
        }
        has_vertex = has_vertex || word == "v";
    }
    return has_vertex;
}

ModelFormat detect_text_format(std::span<const std::byte> head, std::uint64_t total_size) {
    const std::string_view raw{reinterpret_cast<const char*>(head.data()), head.size()};
    if (!is_text(raw)) {
        return ModelFormat::unknown;
    }
    std::string_view text = raw;
    if (text.starts_with("\xEF\xBB\xBF")) {
        text.remove_prefix(3);
    }
    if (is_ply(text)) {
        return ModelFormat::ply;
    }
    text = trim_left(text);
    if (text.starts_with("solid") && text.size() > 5 &&
        std::string_view{" \t\r\n"}.find(text[5]) != std::string_view::npos &&
        (text.find("facet") != std::string_view::npos ||
         text.find("endsolid") != std::string_view::npos)) {
        return ModelFormat::stl_ascii;
    }
    if (text.starts_with("{")) {
        return text.find("\"asset\"") != std::string_view::npos ? ModelFormat::gltf
                                                                : ModelFormat::unknown;
    }
    if (text.starts_with("; FBX")) {
        return ModelFormat::fbx;
    }
    return is_obj(text, head.size() < total_size) ? ModelFormat::obj : ModelFormat::unknown;
}

// Media type without parameters, lower case.
std::string media_type(std::string_view content_type) {
    const auto semicolon = content_type.find(';');
    std::string type{trim(content_type.substr(0, semicolon))};
    std::transform(type.begin(), type.end(), type.begin(), [](char c) {
        return (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c;
    });
    return type;
}

struct MimeRule {
    std::string_view type;
    std::array<ModelFormat, 2> formats; // unknown = any format (generic type)
};

constexpr std::array<MimeRule, 20> kMimeRules{{
    {"", {ModelFormat::unknown, ModelFormat::unknown}},
    {"application/octet-stream", {ModelFormat::unknown, ModelFormat::unknown}},
    {"binary/octet-stream", {ModelFormat::unknown, ModelFormat::unknown}},
    {"text/plain", {ModelFormat::unknown, ModelFormat::unknown}},
    {"model/stl", {ModelFormat::stl_ascii, ModelFormat::stl_binary}},
    {"model/x.stl-ascii", {ModelFormat::stl_ascii, ModelFormat::stl_ascii}},
    {"model/x.stl-binary", {ModelFormat::stl_binary, ModelFormat::stl_binary}},
    {"application/sla", {ModelFormat::stl_ascii, ModelFormat::stl_binary}},
    {"application/vnd.ms-pki.stl", {ModelFormat::stl_ascii, ModelFormat::stl_binary}},
    {"model/obj", {ModelFormat::obj, ModelFormat::obj}},
    {"text/x-obj", {ModelFormat::obj, ModelFormat::obj}},
    {"model/gltf+json", {ModelFormat::gltf, ModelFormat::gltf}},
    {"application/json", {ModelFormat::gltf, ModelFormat::gltf}},
    {"model/gltf-binary", {ModelFormat::glb, ModelFormat::glb}},
    {"application/ply", {ModelFormat::ply, ModelFormat::ply}},
    {"model/ply", {ModelFormat::ply, ModelFormat::ply}},
    {"application/vnd.autodesk.fbx", {ModelFormat::fbx, ModelFormat::fbx}},
    {"application/zip", {ModelFormat::zip, ModelFormat::zip}},
    {"application/x-zip-compressed", {ModelFormat::zip, ModelFormat::zip}},
    {"application/x-zip", {ModelFormat::zip, ModelFormat::zip}},
}};

const MimeRule* find_rule(std::string_view content_type) {
    const std::string type = media_type(content_type);
    const auto it = std::find_if(kMimeRules.begin(), kMimeRules.end(),
                                 [&](const MimeRule& rule) { return rule.type == type; });
    return it == kMimeRules.end() ? nullptr : &*it;
}

} // namespace

std::string_view model_format_name(ModelFormat format) noexcept {
    switch (format) {
        case ModelFormat::stl_ascii:
            return "stl_ascii";
        case ModelFormat::stl_binary:
            return "stl_binary";
        case ModelFormat::obj:
            return "obj";
        case ModelFormat::gltf:
            return "gltf";
        case ModelFormat::glb:
            return "glb";
        case ModelFormat::ply:
            return "ply";
        case ModelFormat::fbx:
            return "fbx";
        case ModelFormat::zip:
            return "zip";
        case ModelFormat::unknown:
            break;
    }
    return "unknown";
}

std::string_view model_format_extension(ModelFormat format) noexcept {
    switch (format) {
        case ModelFormat::stl_ascii:
        case ModelFormat::stl_binary:
            return ".stl";
        case ModelFormat::obj:
            return ".obj";
        case ModelFormat::gltf:
            return ".gltf";
        case ModelFormat::glb:
            return ".glb";
        case ModelFormat::ply:
            return ".ply";
        case ModelFormat::fbx:
            return ".fbx";
        case ModelFormat::zip:
            return ".zip";
        case ModelFormat::unknown:
            break;
    }
    return ".bin";
}

bool is_model_format(ModelFormat format) noexcept {
    return format != ModelFormat::unknown && format != ModelFormat::zip;
}

ModelFormat detect_model_format(std::span<const std::byte> head,
                                std::uint64_t total_size) noexcept {
    if (head.size() > kSniffBytes) {
        head = head.first(kSniffBytes);
    }
    if (head.size() < 4 || head.size() > total_size) {
        return ModelFormat::unknown;
    }
    if (starts_with(head, std::string_view{"PK\x03\x04", 4})) {
        return ModelFormat::zip;
    }
    if (is_glb(head, total_size)) {
        return ModelFormat::glb;
    }
    if (starts_with(head, std::string_view{"Kaydara FBX Binary  \0", 21})) {
        return ModelFormat::fbx;
    }
    if (is_binary_stl(head, total_size)) {
        return ModelFormat::stl_binary;
    }
    return detect_text_format(head, total_size);
}

core::Result<ModelFormat, FetchError> detect_file_format(const std::filesystem::path& file) {
    std::error_code ec;
    const std::uintmax_t size = std::filesystem::file_size(file, ec);
    if (ec) {
        return failure(FetchError{FetchErrorKind::io_failure,
                                  "cannot read the size of a downloaded file: " + ec.message()});
    }
    std::ifstream in(file, std::ios::binary);
    if (!in) {
        return failure(
            FetchError{FetchErrorKind::io_failure, "cannot open a downloaded file for sniffing"});
    }
    std::vector<std::byte> head(
        static_cast<std::size_t>(std::min<std::uintmax_t>(size, kSniffBytes)));
    in.read(reinterpret_cast<char*>(head.data()), static_cast<std::streamsize>(head.size()));
    if (static_cast<std::size_t>(in.gcount()) != head.size()) {
        return failure(
            FetchError{FetchErrorKind::io_failure, "cannot read the head of a downloaded file"});
    }
    return detect_model_format(head, size);
}

bool content_type_allowed(std::string_view content_type) noexcept {
    try {
        return find_rule(content_type) != nullptr;
    } catch (...) {
        return false;
    }
}

bool content_type_matches(std::string_view content_type, ModelFormat format) noexcept {
    try {
        const MimeRule* rule = find_rule(content_type);
        if (rule == nullptr) {
            return false;
        }
        if (rule->formats[0] == ModelFormat::unknown) {
            return true;
        }
        return format == rule->formats[0] || format == rule->formats[1];
    } catch (...) {
        return false;
    }
}

} // namespace orrery::geom::net
