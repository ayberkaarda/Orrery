#include "model_prescan.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <string_view>
#include <vector>

namespace orrery::geom::detail {

namespace {

constexpr std::size_t max_ply_header = 1U << 20U;
// Deflate cannot expand data by more than about 1032:1.
constexpr std::uint64_t deflate_max_ratio = 1032;
constexpr int max_fbx_depth = 64;

// Copy of the first `count` bytes as text.
std::string text_prefix(std::span<const std::byte> data, std::size_t count) {
    std::string text(std::min(count, data.size()), ' ');
    for (std::size_t i = 0; i < text.size(); ++i) {
        text[i] = static_cast<char>(data[i]);
    }
    return text;
}

char char_at(std::span<const std::byte> data, std::size_t i) {
    return static_cast<char>(data[i]);
}

bool parse_count(std::string_view token, std::uint64_t& value) {
    if (token.empty() || token.size() > 19) {
        return false;
    }
    value = 0;
    for (char c : token) {
        if (c < '0' || c > '9') {
            return false;
        }
        value = value * 10 + static_cast<std::uint64_t>(c - '0');
    }
    return true;
}

std::vector<std::string_view> split(std::string_view line) {
    std::vector<std::string_view> words;
    std::size_t i = 0;
    while (i < line.size()) {
        while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) {
            ++i;
        }
        const std::size_t start = i;
        while (i < line.size() && line[i] != ' ' && line[i] != '\t') {
            ++i;
        }
        if (i > start) {
            words.push_back(line.substr(start, i - start));
        }
    }
    return words;
}

std::uint64_t ply_type_size(std::string_view type) {
    if (type == "char" || type == "uchar" || type == "int8" || type == "uint8") {
        return 1;
    }
    if (type == "short" || type == "ushort" || type == "int16" || type == "uint16") {
        return 2;
    }
    if (type == "double" || type == "float64") {
        return 8;
    }
    return 4;
}

std::optional<std::string> prescan_ply(std::span<const std::byte> data,
                                       const ImportLimits& limits) {
    const std::string head = text_prefix(data, max_ply_header + 64);
    const std::string_view text = head;

    struct Element {
        std::string_view name;
        std::uint64_t count = 0;
        std::uint64_t min_bytes = 0;
    };
    std::vector<Element> elements;
    bool ascii = true;
    // The header ends at the first line that is exactly "end_header" (as the
    // parser reads it), not at the first occurrence of the word: a comment or
    // obj_info line may contain it before the element declarations.
    std::size_t body = std::string_view::npos;
    std::size_t pos = 0;
    while (pos < text.size() && pos <= max_ply_header) {
        std::size_t eol = text.find('\n', pos);
        const bool last = eol == std::string_view::npos;
        eol = last ? text.size() : eol;
        std::string_view line = text.substr(pos, eol - pos);
        if (!line.empty() && line.back() == '\r') {
            line.remove_suffix(1);
        }
        pos = last ? text.size() : eol + 1;
        const std::vector<std::string_view> w = split(line);
        if (w.empty()) {
            continue;
        }
        if (w[0] == "end_header" && w.size() == 1) {
            body = pos;
            break;
        }
        if (w[0] == "format" && w.size() >= 2) {
            ascii = w[1] == "ascii";
        } else if (w[0] == "element") {
            Element e;
            if (w.size() < 3 || !parse_count(w[2], e.count)) {
                return "PLY element count is not a number";
            }
            e.name = w[1];
            elements.push_back(e);
        } else if (w[0] == "property" && !elements.empty()) {
            const bool list = w.size() >= 2 && w[1] == "list";
            const std::uint64_t size = list ? (w.size() >= 3 ? ply_type_size(w[2]) : 1)
                                            : (w.size() >= 2 ? ply_type_size(w[1]) : 1);
            elements.back().min_bytes += ascii ? 2 : size;
        }
    }
    if (body == std::string_view::npos) {
        return "PLY header not found";
    }
    const std::uint64_t available = data.size() - body;
    std::uint64_t needed = 0;
    for (const Element& e : elements) {
        const std::uint64_t cap =
            e.name == "vertex" ? limits.mesh.max_vertices
            : e.name == "face" ? limits.mesh.max_triangles
                               : std::max(limits.mesh.max_vertices, limits.mesh.max_triangles);
        if (e.count > cap) {
            return "PLY element '" + std::string(e.name) + "' declares " + std::to_string(e.count) +
                   " entries, above the limit";
        }
        const std::uint64_t per = std::max<std::uint64_t>(e.min_bytes, 1);
        if (e.count > (available - std::min(needed, available)) / per) {
            return "PLY element '" + std::string(e.name) +
                   "' declares more entries than the "
                   "file holds";
        }
        needed += e.count * per;
    }
    return std::nullopt;
}

struct ArrayBudget {
    std::uint64_t per_array = 0;
    std::uint64_t total = 0;
    std::uint64_t used = 0;
};

std::optional<std::string> charge(ArrayBudget& budget, std::uint64_t bytes) {
    if (bytes > budget.per_array) {
        return "FBX array declares " + std::to_string(bytes) + " bytes, above the limit";
    }
    budget.used += bytes;
    if (budget.used > budget.total) {
        return "FBX arrays together exceed the limit";
    }
    return std::nullopt;
}

std::optional<std::string> prescan_fbx_ascii(std::span<const std::byte> data, ArrayBudget& budget) {
    // Arrays are written "*N {" followed by N comma-separated values.
    const std::size_t size = data.size();
    for (std::size_t star = 0; star < size; ++star) {
        if (char_at(data, star) != '*') {
            continue;
        }
        std::size_t i = star + 1;
        while (i < size && char_at(data, i) >= '0' && char_at(data, i) <= '9') {
            ++i;
        }
        std::size_t j = i;
        while (j < size && (char_at(data, j) == ' ' || char_at(data, j) == '\t')) {
            ++j;
        }
        if (i == star + 1 || j >= size || char_at(data, j) != '{') {
            continue;
        }
        std::uint64_t count = 0;
        if (!parse_count(text_prefix(data.subspan(star + 1), i - star - 1), count)) {
            return "FBX array count is too large";
        }
        if (count > (size - j) / 2) {
            return "FBX array declares more values than the file holds";
        }
        if (auto e = charge(budget, 8 * count)) {
            return e;
        }
    }
    return std::nullopt;
}

class BinaryReader {
public:
    explicit BinaryReader(std::span<const std::byte> data) : data_(data) {}

    [[nodiscard]] std::size_t size() const noexcept { return data_.size(); }

    bool read(std::size_t at, std::size_t bytes, std::uint64_t& out) const {
        if (at > data_.size() || bytes > data_.size() - at) {
            return false;
        }
        out = 0;
        for (std::size_t k = 0; k < bytes; ++k) {
            out |= std::to_integer<std::uint64_t>(data_[at + k]) << (8U * k);
        }
        return true;
    }

private:
    std::span<const std::byte> data_;
};

std::optional<std::string> fbx_properties(const BinaryReader& r, std::size_t pos, std::size_t end,
                                          std::uint64_t count, ArrayBudget& budget) {
    for (std::uint64_t n = 0; n < count; ++n) {
        std::uint64_t type = 0;
        if (pos >= end || !r.read(pos, 1, type)) {
            return "FBX property list is truncated";
        }
        ++pos;
        std::uint64_t skip = 0;
        switch (static_cast<char>(type)) {
            case 'C':
                skip = 1;
                break;
            case 'Y':
                skip = 2;
                break;
            case 'I':
            case 'F':
                skip = 4;
                break;
            case 'D':
            case 'L':
                skip = 8;
                break;
            case 'S':
            case 'R': {
                std::uint64_t length = 0;
                if (!r.read(pos, 4, length)) {
                    return "FBX string is truncated";
                }
                skip = 4 + length;
                break;
            }
            case 'b':
            case 'c':
            case 'f':
            case 'i':
            case 'd':
            case 'l': {
                std::uint64_t length = 0;
                std::uint64_t encoding = 0;
                std::uint64_t stored = 0;
                if (!r.read(pos, 4, length) || !r.read(pos + 4, 4, encoding) ||
                    !r.read(pos + 8, 4, stored)) {
                    return "FBX array header is truncated";
                }
                const char t = static_cast<char>(type);
                const std::uint64_t stride = (t == 'd' || t == 'l')   ? 8
                                             : (t == 'b' || t == 'c') ? 1
                                                                      : 4;
                const std::uint64_t decoded = length * stride;
                if (encoding == 0 && decoded != stored) {
                    return "FBX raw array length does not match its data";
                }
                if (encoding == 1 && decoded > deflate_max_ratio * stored + 1024) {
                    return "FBX compressed array claims an impossible expansion";
                }
                if (encoding > 1) {
                    return "FBX array has an unknown encoding";
                }
                if (auto e = charge(budget, decoded)) {
                    return e;
                }
                skip = 12 + stored;
                break;
            }
            default:
                return "FBX property has an unknown type";
        }
        if (skip > end - pos) {
            return "FBX property overruns its record";
        }
        pos += static_cast<std::size_t>(skip);
    }
    return std::nullopt;
}

// Walks a list of node records in [pos, end) until its null record.
std::optional<std::string> fbx_records(const BinaryReader& r, std::size_t pos, std::size_t end,
                                       bool wide, int depth, ArrayBudget& budget) {
    if (depth > max_fbx_depth) {
        return "FBX node hierarchy is too deep";
    }
    const std::size_t field = wide ? 8 : 4;
    const std::size_t header = 3 * field + 1;
    while (pos < end) {
        std::uint64_t record_end = 0;
        std::uint64_t properties = 0;
        std::uint64_t property_bytes = 0;
        std::uint64_t name_length = 0;
        if (header > end - pos || !r.read(pos, field, record_end) ||
            !r.read(pos + field, field, properties) ||
            !r.read(pos + 2 * field, field, property_bytes) ||
            !r.read(pos + 3 * field, 1, name_length)) {
            return "FBX record header is truncated";
        }
        if (record_end == 0) {
            return std::nullopt; // null record: end of this list
        }
        if (record_end <= pos || record_end > end) {
            return "FBX record has an invalid end offset";
        }
        const std::size_t props = pos + header + static_cast<std::size_t>(name_length);
        if (props > record_end || property_bytes > record_end - props ||
            properties > property_bytes) {
            return "FBX record overruns its end offset";
        }
        const auto props_end = props + static_cast<std::size_t>(property_bytes);
        if (auto e = fbx_properties(r, props, props_end, properties, budget)) {
            return e;
        }
        const auto next = static_cast<std::size_t>(record_end);
        if (props_end < next) {
            if (auto e = fbx_records(r, props_end, next, wide, depth + 1, budget)) {
                return e;
            }
        }
        pos = next;
    }
    return std::nullopt;
}

std::optional<std::string> prescan_fbx(std::span<const std::byte> data,
                                       const ImportLimits& limits) {
    ArrayBudget budget;
    budget.per_array =
        24ULL * std::max<std::uint64_t>(limits.mesh.max_vertices, limits.mesh.max_triangles);
    budget.total = 4 * budget.per_array;
    constexpr std::string_view magic("Kaydara FBX Binary  \0", 21);
    if (text_prefix(data, magic.size()) != magic) {
        return prescan_fbx_ascii(data, budget);
    }
    const BinaryReader r(data);
    std::uint64_t version = 0;
    if (!r.read(23, 4, version)) {
        return "FBX binary header is truncated";
    }
    return fbx_records(r, 27, r.size(), version >= 7500, 0, budget);
}

} // namespace

std::optional<std::string> prescan_model(std::span<const std::byte> data, ModelFormat format,
                                         const ImportLimits& limits) {
    switch (format) {
        case ModelFormat::ply:
            return prescan_ply(data, limits);
        case ModelFormat::fbx:
            return prescan_fbx(data, limits);
        case ModelFormat::obj:
        case ModelFormat::stl:
        case ModelFormat::gltf:
        case ModelFormat::glb:
            break;
    }
    return std::nullopt;
}

} // namespace orrery::geom::detail
