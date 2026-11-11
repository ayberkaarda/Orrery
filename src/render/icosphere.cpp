#include "orrery/render/icosphere.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <utility>

#include <glm/ext/vector_double3.hpp>
#include <glm/geometric.hpp>

namespace orrery::render {

namespace {

// Ordered map keyed by the sorted vertex pair: deterministic and independent
// of hashing.
using MidpointCache = std::map<std::pair<std::uint32_t, std::uint32_t>, std::uint32_t>;

std::uint32_t midpoint(std::vector<glm::dvec3>& vertices, MidpointCache& cache, std::uint32_t a,
                       std::uint32_t b) {
    const auto key = std::minmax(a, b);
    if (const auto found = cache.find(key); found != cache.end()) {
        return found->second;
    }
    const glm::dvec3 middle = glm::normalize(vertices[a] + vertices[b]);
    const auto index = static_cast<std::uint32_t>(vertices.size());
    vertices.push_back(middle);
    cache.emplace(key, index);
    return index;
}

} // namespace

IcosphereMesh make_icosphere(int subdivisions) {
    const int levels = std::clamp(subdivisions, 0, kMaxIcosphereSubdivisions);

    // Icosahedron: cyclic permutations of (0, +-1, +-phi), normalised.
    const double phi = (1.0 + std::sqrt(5.0)) / 2.0;
    std::vector<glm::dvec3> vertices{
        {-1.0, phi, 0.0}, {1.0, phi, 0.0}, {-1.0, -phi, 0.0}, {1.0, -phi, 0.0},
        {0.0, -1.0, phi}, {0.0, 1.0, phi}, {0.0, -1.0, -phi}, {0.0, 1.0, -phi},
        {phi, 0.0, -1.0}, {phi, 0.0, 1.0}, {-phi, 0.0, -1.0}, {-phi, 0.0, 1.0},
    };
    for (glm::dvec3& vertex : vertices) {
        vertex = glm::normalize(vertex);
    }
    std::vector<std::uint32_t> indices{
        0, 11, 5,  0, 5,  1, 0, 1, 7, 0, 7,  10, 0, 10, 11, 1, 5, 9, 5, 11,
        4, 11, 10, 2, 10, 7, 6, 7, 1, 8, 3,  9,  4, 3,  4,  2, 3, 2, 6, 3,
        6, 8,  3,  8, 9,  4, 9, 5, 2, 4, 11, 6,  2, 10, 8,  6, 7, 9, 8, 1,
    };

    for (int level = 0; level < levels; ++level) {
        MidpointCache cache;
        std::vector<std::uint32_t> refined;
        refined.reserve(indices.size() * 4U);
        for (std::size_t t = 0; t < indices.size(); t += 3) {
            const std::uint32_t a = indices[t];
            const std::uint32_t b = indices[t + 1];
            const std::uint32_t c = indices[t + 2];
            const std::uint32_t ab = midpoint(vertices, cache, a, b);
            const std::uint32_t bc = midpoint(vertices, cache, b, c);
            const std::uint32_t ca = midpoint(vertices, cache, c, a);
            refined.insert(refined.end(), {a, ab, ca, b, bc, ab, c, ca, bc, ab, bc, ca});
        }
        indices = std::move(refined);
    }

    IcosphereMesh mesh;
    mesh.positions.reserve(vertices.size());
    for (const glm::dvec3& vertex : vertices) {
        mesh.positions.emplace_back(vertex);
    }
    mesh.indices = std::move(indices);
    return mesh;
}

} // namespace orrery::render
