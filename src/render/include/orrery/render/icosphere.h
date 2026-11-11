#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <glm/ext/vector_float3.hpp>

// Icosphere meshes for body rendering (docs/rendering.md, "Bodies").
//
// Level 0 is the regular icosahedron (12 vertices, 20 triangles); every level
// splits each triangle into four, so level n has 10 * 4^n + 2 vertices and
// 20 * 4^n triangles. All vertex positions are computed in double precision
// (edge midpoints from the double parents, normalised in double) and cast to
// float once, so every stored vertex lies on the unit sphere to float rounding
// and the position doubles as the outward unit normal.
namespace orrery::render {

inline constexpr int kMaxIcosphereSubdivisions = 7;

struct IcosphereMesh {
    // Unit-sphere positions; position == outward normal.
    std::vector<glm::vec3> positions;
    // Triangle list, counter-clockwise seen from outside.
    std::vector<std::uint32_t> indices;

    [[nodiscard]] std::size_t triangle_count() const noexcept { return indices.size() / 3U; }
};

// `subdivisions` is clamped to [0, kMaxIcosphereSubdivisions].
[[nodiscard]] IcosphereMesh make_icosphere(int subdivisions);

[[nodiscard]] constexpr std::size_t icosphere_vertex_count(int subdivisions) noexcept {
    std::size_t faces = 20;
    for (int i = 0; i < subdivisions; ++i) {
        faces *= 4U;
    }
    return faces / 2U + 2U; // Euler: V = E - F + 2 with E = 3F/2
}

[[nodiscard]] constexpr std::size_t icosphere_triangle_count(int subdivisions) noexcept {
    std::size_t faces = 20;
    for (int i = 0; i < subdivisions; ++i) {
        faces *= 4U;
    }
    return faces;
}

} // namespace orrery::render
