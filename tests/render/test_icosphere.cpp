#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <utility>

#include <glm/ext/vector_double3.hpp>
#include <glm/geometric.hpp>

#include "orrery/render/icosphere.h"

using namespace orrery::render;

TEST_CASE("Icosphere levels have the expected vertex and triangle counts", "[render][icosphere]") {
    for (int level = 0; level <= kMaxIcosphereSubdivisions; ++level) {
        const IcosphereMesh mesh = make_icosphere(level);
        INFO("level " << level);
        CHECK(mesh.positions.size() == icosphere_vertex_count(level));
        CHECK(mesh.triangle_count() == icosphere_triangle_count(level));
    }
    CHECK(icosphere_vertex_count(0) == 12);
    CHECK(icosphere_vertex_count(5) == 10242);
    // Out-of-range levels are clamped.
    CHECK(make_icosphere(-3).positions.size() == 12);
    CHECK(make_icosphere(99).positions.size() == icosphere_vertex_count(kMaxIcosphereSubdivisions));
}

TEST_CASE("Icosphere vertices lie on the unit sphere to float rounding", "[render][icosphere]") {
    const IcosphereMesh mesh = make_icosphere(6);
    double worst = 0.0;
    for (const glm::vec3& p : mesh.positions) {
        const double r = glm::length(glm::dvec3(p));
        worst = std::max(worst, std::abs(r - 1.0));
    }
    // One float rounding per component: about 1e-7 relative.
    CHECK(worst < 2.0e-7);
}

TEST_CASE("Icosphere triangles wind counter-clockwise seen from outside and close the surface",
          "[render][icosphere]") {
    for (int level : {0, 1, 3}) {
        const IcosphereMesh mesh = make_icosphere(level);
        std::map<std::pair<std::uint32_t, std::uint32_t>, int> directed_edges;
        double area = 0.0;
        bool outward = true;
        for (std::size_t t = 0; t < mesh.indices.size(); t += 3) {
            const std::uint32_t ia = mesh.indices[t];
            const std::uint32_t ib = mesh.indices[t + 1];
            const std::uint32_t ic = mesh.indices[t + 2];
            const glm::dvec3 a(mesh.positions[ia]);
            const glm::dvec3 b(mesh.positions[ib]);
            const glm::dvec3 c(mesh.positions[ic]);
            const glm::dvec3 n = glm::cross(b - a, c - a);
            outward = outward && glm::dot(n, a + b + c) > 0.0;
            area += 0.5 * glm::length(n);
            ++directed_edges[{ia, ib}];
            ++directed_edges[{ib, ic}];
            ++directed_edges[{ic, ia}];
        }
        INFO("level " << level);
        CHECK(outward);
        // Closed, consistently oriented 2-manifold: every directed edge once,
        // and its reverse present.
        bool manifold = true;
        for (const auto& [edge, count] : directed_edges) {
            manifold =
                manifold && count == 1 && directed_edges.count({edge.second, edge.first}) == 1;
        }
        CHECK(manifold);
        // Euler characteristic of a sphere: V - E + F = 2.
        const auto v = static_cast<long long>(mesh.positions.size());
        const auto e = static_cast<long long>(directed_edges.size() / 2);
        const auto f = static_cast<long long>(mesh.triangle_count());
        CHECK(v - e + f == 2);
        // The polyhedron area approaches 4 pi from below.
        CHECK(area < 4.0 * 3.141592653589793);
        if (level == 3) {
            CHECK(area > 0.99 * 4.0 * 3.141592653589793);
        }
    }
}
