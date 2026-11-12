#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <random>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace orrery::geom::net::test {

// A fresh directory under the system temp directory, removed on destruction.
class TempDir {
public:
    TempDir() {
        std::random_device device;
        const auto tag = (std::uint64_t{device()} << 32U) ^ std::uint64_t{device()};
        path_ = std::filesystem::temp_directory_path() / ("orrery-net-test-" + std::to_string(tag));
        std::filesystem::create_directories(path_);
    }
    TempDir(const TempDir&) = delete;
    TempDir& operator=(const TempDir&) = delete;
    TempDir(TempDir&&) = delete;
    TempDir& operator=(TempDir&&) = delete;
    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path_, ec);
    }

    [[nodiscard]] const std::filesystem::path& path() const noexcept { return path_; }

private:
    std::filesystem::path path_;
};

inline std::span<const std::byte> bytes_of(std::string_view text) noexcept {
    return std::as_bytes(std::span{text.data(), text.size()});
}

inline void write_file(const std::filesystem::path& file, std::string_view content) {
    std::ofstream out(file, std::ios::binary | std::ios::trunc);
    out.write(content.data(), static_cast<std::streamsize>(content.size()));
}

inline std::string read_file(const std::filesystem::path& file) {
    std::ifstream in(file, std::ios::binary);
    return std::string{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

inline std::size_t count_files(const std::filesystem::path& dir) {
    std::size_t count = 0;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
        if (entry.is_regular_file()) {
            ++count;
        }
    }
    return count;
}

// A binary STL with `triangles` zero triangles (84 + 50 n bytes).
inline std::string binary_stl(std::uint32_t triangles) {
    std::string stl(80, ' ');
    stl.replace(0, 12, "binary model");
    for (unsigned i = 0; i < 4; ++i) {
        stl.push_back(static_cast<char>((triangles >> (8U * i)) & 0xFFU));
    }
    stl.append(std::size_t{50} * triangles, '\0');
    return stl;
}

inline std::string ascii_stl() {
    return "solid part\n facet normal 0 0 1\n  outer loop\n   vertex 0 0 0\n   vertex 1 0 0\n"
           "   vertex 0 1 0\n  endloop\n endfacet\nendsolid part\n";
}

inline std::string obj_text() {
    return "# wing\nmtllib wing.mtl\no wing\nv 0 0 0\nv 1 0 0\nv 0 1 0\nvn 0 0 1\n"
           "usemtl skin\ns off\nf 1//1 2//1 3//1\n";
}

// The start of a Windows executable: an "MZ" header and a PE signature.
inline std::string executable_bytes() {
    std::string exe = "MZ";
    exe.append(58, '\x90');
    exe += std::string{"\x80\0\0\0", 4};
    exe.append(64, '\0');
    exe += std::string{"PE\0\0", 4};
    exe.append(200, '\x01');
    return exe;
}

// One member of a test archive.
struct ZipMember {
    ZipMember(std::string member_name, std::string content, bool deflated = false)
        : name(std::move(member_name)), data(std::move(content)), deflate(deflated) {}

    std::string name;
    std::string data;
    bool deflate = false;
    std::optional<std::uint32_t> unix_mode;     // sets "made by Unix" and the mode bits
    std::optional<std::uint32_t> declared_size; // lie about the uncompressed size
    std::uint16_t flags = 0;                    // general purpose bits (bit 0: encrypted)
};

// Builds a zip archive byte by byte, so tests can produce archives that a
// well-behaved writer refuses to create (traversal names, lying sizes, ...).
std::string build_zip(const std::vector<ZipMember>& members);

} // namespace orrery::geom::net::test
