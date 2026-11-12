#pragma once

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>

#include "orrery/geom/mesh_validation.h"

namespace orrery::geom::detail {

// Size arithmetic that saturates instead of wrapping.
[[nodiscard]] inline std::size_t saturating_mul(std::size_t a, std::size_t b) noexcept {
    constexpr std::size_t top = std::numeric_limits<std::size_t>::max();
    return a != 0 && b > top / a ? top : a * b;
}

[[nodiscard]] inline std::size_t saturating_add(std::size_t a, std::size_t b) noexcept {
    constexpr std::size_t top = std::numeric_limits<std::size_t>::max();
    return b > top - a ? top : a + b;
}

// The generators (revolution meshes, NACA wings) build at most what
// validate_mesh accepts by default, which also keeps every index below
// 2^32 - 1. Called with the counts a resolution implies, before anything is
// allocated. Throws std::invalid_argument when they exceed the limits.
inline void check_generated_size(std::size_t vertices, std::size_t triangles, const char* who) {
    const MeshLimits limits;
    if (vertices > limits.max_vertices || triangles > limits.max_triangles) {
        throw std::invalid_argument(std::string(who) +
                                    ": resolution gives a mesh above the size limit (" +
                                    std::to_string(limits.max_vertices) + " vertices, " +
                                    std::to_string(limits.max_triangles) + " triangles)");
    }
}

} // namespace orrery::geom::detail
