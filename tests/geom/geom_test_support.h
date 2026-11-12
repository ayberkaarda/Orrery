#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ios>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#include "orrery/geom/triangle_mesh.h"

namespace orrery::geom::test {

inline double relative_error(double value, double reference) {
    return std::abs(value - reference) / std::max(std::abs(reference), 1e-300);
}

inline std::filesystem::path fixture_dir() {
    return std::filesystem::path(ORRERY_GEOM_FIXTURE_DIR);
}

// Fresh scratch directory for one test.
inline std::filesystem::path output_dir(const std::string& name) {
    const std::filesystem::path dir = std::filesystem::path(ORRERY_GEOM_OUTPUT_DIR) / name;
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}

inline void write_text(const std::filesystem::path& path, const std::string& text) {
    std::ofstream out(path, std::ios::binary);
    out << text;
}

inline void write_bytes(const std::filesystem::path& path, const std::vector<std::byte>& bytes) {
    std::string text(bytes.size(), ' ');
    std::transform(bytes.begin(), bytes.end(), text.begin(),
                   [](std::byte b) { return static_cast<char>(b); });
    write_text(path, text);
}

inline std::vector<std::byte> read_bytes(const std::filesystem::path& path) {
    // Sized read rather than stream iterators: GCC 13 reports a false
    // null dereference in istreambuf_iterator at -O3.
    std::error_code ec;
    const std::uintmax_t size = std::filesystem::file_size(path, ec);
    if (ec) {
        return {};
    }
    std::ifstream in(path, std::ios::binary);
    std::string text(static_cast<std::size_t>(size), '\0');
    in.read(text.data(), static_cast<std::streamsize>(text.size()));
    text.resize(static_cast<std::size_t>(in.gcount()));
    std::vector<std::byte> bytes(text.size());
    std::transform(text.begin(), text.end(), bytes.begin(),
                   [](char c) { return static_cast<std::byte>(c); });
    return bytes;
}

inline void put_u32(std::vector<std::byte>& out, std::uint32_t v) {
    for (int i = 0; i < 4; ++i) {
        out.push_back(static_cast<std::byte>((v >> (8 * i)) & 0xffU));
    }
}

inline void put_f32(std::vector<std::byte>& out, double v) {
    put_u32(out, std::bit_cast<std::uint32_t>(static_cast<float>(v)));
}

inline std::string ascii_stl(const TriangleMesh& m) {
    std::ostringstream s;
    s.precision(17);
    s << "solid test\n";
    for (const Triangle& t : m.triangles) {
        s << "facet normal 0 0 0\n outer loop\n";
        for (std::uint32_t i : t) {
            const Vector3& v = m.vertices[i];
            s << "  vertex " << v.x << ' ' << v.y << ' ' << v.z << '\n';
        }
        s << " endloop\nendfacet\n";
    }
    s << "endsolid test\n";
    return s.str();
}

inline std::vector<std::byte> binary_stl(const TriangleMesh& m) {
    std::vector<std::byte> out(80, std::byte{0});
    put_u32(out, static_cast<std::uint32_t>(m.triangles.size()));
    for (const Triangle& t : m.triangles) {
        for (int k = 0; k < 3; ++k) {
            put_f32(out, 0.0);
        }
        for (std::uint32_t i : t) {
            put_f32(out, m.vertices[i].x);
            put_f32(out, m.vertices[i].y);
            put_f32(out, m.vertices[i].z);
        }
        out.push_back(std::byte{0});
        out.push_back(std::byte{0});
    }
    return out;
}

inline std::string obj_text(const TriangleMesh& m) {
    std::ostringstream s;
    s.precision(17);
    for (const Vector3& v : m.vertices) {
        s << "v " << v.x << ' ' << v.y << ' ' << v.z << '\n';
    }
    for (const Triangle& t : m.triangles) {
        s << "f " << t[0] + 1 << ' ' << t[1] + 1 << ' ' << t[2] + 1 << '\n';
    }
    return s.str();
}

inline std::string ply_text(const TriangleMesh& m) {
    std::ostringstream s;
    s.precision(17);
    s << "ply\nformat ascii 1.0\nelement vertex " << m.vertices.size()
      << "\nproperty float x\nproperty float y\nproperty float z\nelement face "
      << m.triangles.size() << "\nproperty list uchar int vertex_indices\nend_header\n";
    for (const Vector3& v : m.vertices) {
        s << v.x << ' ' << v.y << ' ' << v.z << '\n';
    }
    for (const Triangle& t : m.triangles) {
        s << "3 " << t[0] << ' ' << t[1] << ' ' << t[2] << '\n';
    }
    return s.str();
}

inline std::string base64(const std::vector<std::byte>& data) {
    static constexpr char table[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    for (std::size_t i = 0; i < data.size(); i += 3) {
        std::uint32_t n = std::to_integer<std::uint32_t>(data[i]) << 16U;
        if (i + 1 < data.size()) {
            n |= std::to_integer<std::uint32_t>(data[i + 1]) << 8U;
        }
        if (i + 2 < data.size()) {
            n |= std::to_integer<std::uint32_t>(data[i + 2]);
        }
        out.push_back(table[(n >> 18U) & 63U]);
        out.push_back(table[(n >> 12U) & 63U]);
        out.push_back(i + 1 < data.size() ? table[(n >> 6U) & 63U] : '=');
        out.push_back(i + 2 < data.size() ? table[n & 63U] : '=');
    }
    return out;
}

// glTF 2.0 buffer: float32 positions followed by uint32 indices.
inline std::vector<std::byte> gltf_buffer(const TriangleMesh& m) {
    std::vector<std::byte> out;
    for (const Vector3& v : m.vertices) {
        put_f32(out, v.x);
        put_f32(out, v.y);
        put_f32(out, v.z);
    }
    for (const Triangle& t : m.triangles) {
        for (std::uint32_t i : t) {
            put_u32(out, i);
        }
    }
    return out;
}

// glTF 2.0 JSON with one mesh; `uri` is the buffer location (a data URI or a
// relative file name).
inline std::string gltf_json(const TriangleMesh& m, const std::string& uri) {
    const Bounds b = bounds(m);
    const std::size_t positions = 12 * m.vertices.size();
    const std::size_t indices = 12 * m.triangles.size();
    std::ostringstream s;
    s.precision(17);
    s << R"({"asset":{"version":"2.0"},"scene":0,"scenes":[{"nodes":[0]}],)"
      << R"("nodes":[{"mesh":0}],"meshes":[{"primitives":[{"attributes":{"POSITION":0},)"
      << R"("indices":1}]}],"buffers":[{"byteLength":)" << positions + indices;
    if (!uri.empty()) {
        s << R"(,"uri":")" << uri << '"';
    }
    s << R"(}],"bufferViews":[{"buffer":0,"byteOffset":0,"byteLength":)" << positions
      << R"(},{"buffer":0,"byteOffset":)" << positions << R"(,"byteLength":)" << indices
      << R"(}],"accessors":[{"bufferView":0,"componentType":5126,"count":)" << m.vertices.size()
      << R"(,"type":"VEC3","min":[)" << b.min.x << ',' << b.min.y << ',' << b.min.z
      << R"(],"max":[)" << b.max.x << ',' << b.max.y << ',' << b.max.z
      << R"(]},{"bufferView":1,"componentType":5125,"count":)" << 3 * m.triangles.size()
      << R"(,"type":"SCALAR"}]})";
    return s.str();
}

inline std::string gltf_embedded(const TriangleMesh& m) {
    return gltf_json(m, "data:application/octet-stream;base64," + base64(gltf_buffer(m)));
}

// Binary glTF container (header, JSON chunk, BIN chunk).
inline std::vector<std::byte> glb(const TriangleMesh& m) {
    std::string json = gltf_json(m, "");
    while (json.size() % 4 != 0) {
        json.push_back(' ');
    }
    std::vector<std::byte> bin = gltf_buffer(m);
    while (bin.size() % 4 != 0) {
        bin.push_back(std::byte{0});
    }
    std::vector<std::byte> out;
    put_u32(out, 0x46546C67U); // "glTF"
    put_u32(out, 2);
    put_u32(out, static_cast<std::uint32_t>(12 + 8 + json.size() + 8 + bin.size()));
    put_u32(out, static_cast<std::uint32_t>(json.size()));
    put_u32(out, 0x4E4F534AU); // "JSON"
    for (char c : json) {
        out.push_back(static_cast<std::byte>(c));
    }
    put_u32(out, static_cast<std::uint32_t>(bin.size()));
    put_u32(out, 0x004E4942U); // "BIN\0"
    out.insert(out.end(), bin.begin(), bin.end());
    return out;
}

inline std::vector<std::byte> bytes_of(const std::string& text) {
    std::vector<std::byte> out(text.size());
    std::transform(text.begin(), text.end(), out.begin(),
                   [](char c) { return static_cast<std::byte>(c); });
    return out;
}

} // namespace orrery::geom::test
