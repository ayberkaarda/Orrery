#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <numbers>
#include <random>
#include <vector>

#include <glm/geometric.hpp>

#include "aero_test_support.h"
#include "orrery/render/aero_isosurface.h"

// Marching cubes (docs/aero-visualisation.md, "Q-criterion iso-surface"):
// the generated case table, the area of a sphere, closed and consistently
// oriented surfaces on random closed fields, and the GPU kernel against the
// CPU reference.

using namespace orrery::render;
using orrery::render::test::aero_compute_or_skip;
using orrery::render::test::scalar_grid;

namespace {

// Signed distance to a sphere, positive inside.
ScalarGrid sphere_field(std::size_t n, double radius, const glm::dvec3& centre) {
    return scalar_grid(n, n, n, {0.0, 0.0, 0.0}, 1.0,
                       [&](const glm::dvec3& p) { return radius - glm::length(p - centre); });
}

// Random blobs that vanish towards the grid boundary (values there are
// negative), so every iso-surface above 0 is closed; integer-valued noise
// makes many corners equal and many faces ambiguous.
ScalarGrid blob_field(std::size_t n, std::uint32_t seed) {
    std::mt19937 random(seed);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    std::vector<double> noise(n * n * n);
    for (double& v : noise) {
        v = std::round(unit(random) * 3.0);
    }
    ScalarGrid g =
        scalar_grid(n, n, n, {0.0, 0.0, 0.0}, 1.0, [](const glm::dvec3&) { return 0.0; });
    const double half = 0.5 * static_cast<double>(n - 1);
    for (std::size_t k = 0; k < n; ++k) {
        for (std::size_t j = 0; j < n; ++j) {
            for (std::size_t i = 0; i < n; ++i) {
                const glm::dvec3 p(static_cast<double>(i), static_cast<double>(j),
                                   static_cast<double>(k));
                const double r = glm::length(p - glm::dvec3(half)) / half;
                const std::size_t node = g.grid.index(i, j, k);
                g.values[node] = noise[node] + 3.0 * (0.8 - r);
                if (i == 0 || j == 0 || k == 0 || i + 1 == n || j + 1 == n || k + 1 == n) {
                    g.values[node] = -10.0;
                }
            }
        }
    }
    return g;
}

double enclosed_volume(const IsoSurface& s) {
    double volume = 0.0;
    for (std::size_t t = 0; t + 2 < s.indices.size(); t += 3) {
        volume += glm::dot(s.positions[s.indices[t]], glm::cross(s.positions[s.indices[t + 1]],
                                                                 s.positions[s.indices[t + 2]])) /
                  6.0;
    }
    return volume;
}

} // namespace

TEST_CASE("The generated marching-cubes table", "[render][aero][marching_cubes]") {
    const auto& table = marching_cubes_table();
    REQUIRE(table.size() == 256);
    CHECK(table[0].empty());
    CHECK(table[255].empty());
    std::size_t largest = 0;
    for (std::size_t c = 0; c < 256; ++c) {
        CHECK(table[c].size() % 3 == 0);
        largest = std::max(largest, table[c].size() / 3);
        // Complementary cases cut the same edges.
        std::vector<std::uint8_t> a = table[c];
        std::vector<std::uint8_t> b = table[255 - c];
        std::ranges::sort(a);
        std::ranges::sort(b);
        a.erase(std::unique(a.begin(), a.end()), a.end());
        b.erase(std::unique(b.begin(), b.end()), b.end());
        CHECK(a == b);
    }
    CHECK(largest <= 12);
    // A single inside corner: one triangle on its three edges.
    REQUIRE(table[1].size() == 3);
    std::vector<std::uint8_t> corner = table[1];
    std::ranges::sort(corner);
    CHECK(corner == std::vector<std::uint8_t>{0, 4, 8});
}

TEST_CASE("Marching cubes: sphere area within 2 percent, closed and outward",
          "[render][aero][marching_cubes]") {
    for (const double radius : {6.0, 10.5, 15.25}) {
        const auto n = static_cast<std::size_t>(2.0 * radius + 6.0);
        const glm::dvec3 centre(0.5 * static_cast<double>(n - 1) + 0.13,
                                0.5 * static_cast<double>(n - 1) - 0.21,
                                0.5 * static_cast<double>(n - 1) + 0.07);
        const IsoSurface s = marching_cubes(sphere_field(n, radius, centre), 0.0);
        const double area = surface_area(s);
        const double exact = 4.0 * std::numbers::pi * radius * radius;
        INFO("radius " << radius << " area " << area << " exact " << exact);
        std::cout << "measured: marching cubes sphere r = " << radius << " cells: area error "
                  << area / exact - 1.0 << ", " << s.triangle_count() << " triangles\n";
        CHECK(std::abs(area / exact - 1.0) < 0.02);
        const MeshTopology t = topology(s);
        CHECK(t.closed());
        CHECK(t.degenerate_triangles == 0);
        // Euler characteristic of a sphere: V - E + F = 2.
        CHECK(static_cast<long long>(s.positions.size()) - static_cast<long long>(t.edges) +
                  static_cast<long long>(s.triangle_count()) ==
              2);
        // Outward winding: positive enclosed volume close to the ball's.
        const double volume = enclosed_volume(s);
        CHECK(volume / (4.0 / 3.0 * std::numbers::pi * radius * radius * radius) ==
              Catch::Approx(1.0).margin(0.03));
        // Normals point away from the centre.
        for (std::size_t v = 0; v < s.positions.size(); ++v) {
            CHECK(glm::dot(s.normals[v], glm::normalize(s.positions[v] - centre)) > 0.99);
        }
    }
}

TEST_CASE("Marching cubes: random closed fields give closed, consistently oriented surfaces",
          "[render][aero][marching_cubes]") {
    for (std::uint32_t seed = 1; seed <= 6; ++seed) {
        const ScalarGrid field = blob_field(20, seed);
        for (const double level : {0.0, 0.5, 1.0}) {
            const IsoSurface s = marching_cubes(field, level);
            const MeshTopology t = topology(s);
            INFO("seed " << seed << " level " << level << " triangles " << s.triangle_count()
                         << " boundary " << t.boundary_edges << " non-manifold "
                         << t.non_manifold_edges << " misoriented " << t.misoriented_edges);
            CHECK(s.triangle_count() > 100);
            CHECK(t.closed());
            CHECK(enclosed_volume(s) > 0.0);
        }
    }
}

TEST_CASE("Marching cubes: inputs without a surface", "[render][aero][marching_cubes]") {
    const ScalarGrid flat =
        scalar_grid(5, 5, 1, {0.0, 0.0, 0.0}, 1.0, [](const glm::dvec3& p) { return p.x; });
    CHECK(marching_cubes(flat, 1.0).indices.empty());
    const ScalarGrid uniform =
        scalar_grid(4, 4, 4, {0.0, 0.0, 0.0}, 1.0, [](const glm::dvec3&) { return 1.0; });
    CHECK(marching_cubes(uniform, 0.0).indices.empty());
    // A plane z = 1.5 through an open field: a flat sheet with a boundary.
    const ScalarGrid ramp =
        scalar_grid(4, 4, 4, {0.0, 0.0, 0.0}, 1.0, [](const glm::dvec3& p) { return 1.5 - p.z; });
    const IsoSurface sheet = marching_cubes(ramp, 0.0);
    CHECK(surface_area(sheet) == Catch::Approx(9.0));
    CHECK(topology(sheet).boundary_edges == 12);
    for (const glm::dvec3& p : sheet.positions) {
        CHECK(p.z == Catch::Approx(1.5));
    }
}

TEST_CASE("Marching cubes rejects non-finite samples and interpolates extreme ones",
          "[render][aero][marching_cubes]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const double huge = std::numeric_limits<double>::max();
    // A NaN or infinite corner next to inside corners: rejected outright
    // (no NaN vertex can come out).
    for (const double bad : {nan, inf, -inf}) {
        ScalarGrid field = sphere_field(6, 2.0, glm::dvec3(2.5));
        field.values[field.grid.index(3, 2, 2)] = bad;
        const IsoSurface s = marching_cubes(field, 0.0);
        CHECK(s.indices.empty());
        CHECK(s.positions.empty());
    }
    CHECK(marching_cubes(sphere_field(6, 2.0, glm::dvec3(2.5)), nan).indices.empty());
    // Opposite extreme finite values: the level halfway sits at the edge
    // midpoints (b - a overflows when computed directly).
    ScalarGrid cube =
        scalar_grid(2, 2, 2, {0.0, 0.0, 0.0}, 1.0, [](const glm::dvec3&) { return 0.0; });
    for (std::size_t c = 0; c < cube.values.size(); ++c) {
        cube.values[c] = c == 0 ? huge : -huge;
    }
    const IsoSurface corner = marching_cubes(cube, 0.0);
    REQUIRE(corner.triangle_count() == 1);
    for (const glm::dvec3& p : corner.positions) {
        CHECK(std::isfinite(p.x));
        CHECK(std::isfinite(p.y));
        CHECK(std::isfinite(p.z));
        CHECK(p.x + p.y + p.z == Catch::Approx(0.5)); // one coordinate 0.5, the others 0
    }
}

TEST_CASE("Marching cubes counts zero-area triangles at level-valued corners",
          "[render][aero][marching_cubes]") {
    // One cube, corner 0 exactly at the level (outside), the seven others
    // inside: the triangle around corner 0 has three distinct vertices that
    // all sit on that corner.
    ScalarGrid cube =
        scalar_grid(2, 2, 2, {0.0, 0.0, 0.0}, 1.0, [](const glm::dvec3&) { return 1.0; });
    cube.values[0] = 0.0;
    const IsoSurface s = marching_cubes(cube, 0.0);
    REQUIRE(s.triangle_count() == 1);
    CHECK(s.indices[0] != s.indices[1]);
    CHECK(s.indices[1] != s.indices[2]);
    CHECK(surface_area(s) == 0.0);
    CHECK(topology(s).degenerate_triangles == 1);
    // The output keeps them (each vertex stays the one of its grid edge, so
    // closed surfaces stay closed): a field with many level-valued nodes is
    // still closed, and only its zero-area triangles are counted.
    // Integer values in [-1, 2] inside, -1 on the boundary: many nodes sit
    // exactly at level 0 and the surface is closed.
    std::mt19937 random(7);
    std::uniform_int_distribution<int> pick(-1, 2);
    ScalarGrid steps =
        scalar_grid(12, 12, 12, {0.0, 0.0, 0.0}, 1.0, [](const glm::dvec3&) { return -1.0; });
    for (std::size_t k = 1; k + 1 < 12; ++k) {
        for (std::size_t j = 1; j + 1 < 12; ++j) {
            for (std::size_t i = 1; i + 1 < 12; ++i) {
                steps.values[steps.grid.index(i, j, k)] = static_cast<double>(pick(random));
            }
        }
    }
    const IsoSurface b = marching_cubes(steps, 0.0);
    const MeshTopology t = topology(b);
    CHECK(t.closed());
    std::size_t flat = 0;
    for (std::size_t i = 0; i + 2 < b.indices.size(); i += 3) {
        const glm::dvec3& p0 = b.positions[b.indices[i]];
        const glm::dvec3 n =
            glm::cross(b.positions[b.indices[i + 1]] - p0, b.positions[b.indices[i + 2]] - p0);
        flat += glm::length(n) == 0.0 ? std::size_t{1} : std::size_t{0};
    }
    CHECK(flat > 0);
    CHECK(t.degenerate_triangles == flat);
}

TEST_CASE("Marching cubes on the GPU stays finite for extreme samples",
          "[render][aero][marching_cubes][gl]") {
    auto gl = aero_compute_or_skip();
    const double huge = std::numeric_limits<double>::max();
    ScalarGrid cube =
        scalar_grid(2, 2, 2, {0.0, 0.0, 0.0}, 1.0, [](const glm::dvec3&) { return 0.0; });
    for (std::size_t c = 0; c < cube.values.size(); ++c) {
        cube.values[c] = c == 0 ? huge : -huge;
    }
    auto gpu = marching_cubes_gpu(*gl.backend, cube, 0.0);
    REQUIRE(gpu.has_value());
    const IsoSurface& g = gpu.value();
    const IsoSurface cpu = marching_cubes(cube, 0.0);
    CHECK(g.edges == cpu.edges);
    REQUIRE(g.positions.size() == cpu.positions.size());
    for (std::size_t v = 0; v < g.positions.size(); ++v) {
        CHECK(glm::length(g.positions[v] - cpu.positions[v]) < 1.0e-6);
    }
    ScalarGrid broken = sphere_field(6, 2.0, glm::dvec3(2.5));
    broken.values[broken.grid.index(3, 2, 2)] = std::numeric_limits<double>::quiet_NaN();
    auto rejected = marching_cubes_gpu(*gl.backend, broken, 0.0);
    REQUIRE(rejected.has_value());
    CHECK(rejected.value().indices.empty());
}

TEST_CASE("Marching cubes on the GPU matches the CPU reference",
          "[render][aero][marching_cubes][gl]") {
    auto gl = aero_compute_or_skip();
    for (const std::uint32_t seed : {3U, 4U}) {
        ScalarGrid field = blob_field(24, seed);
        field.grid.origin = {-1.0, 2.0, 0.5};
        field.grid.spacing = 0.125;
        const IsoSurface cpu = marching_cubes(field, 0.5);
        auto gpu = marching_cubes_gpu(*gl.backend, field, 0.5);
        INFO((gpu ? std::string() : gpu.error()));
        REQUIRE(gpu.has_value());
        const IsoSurface& g = gpu.value();
        CHECK(g.edges == cpu.edges);
        CHECK(g.indices == cpu.indices);
        double worst = 0.0;
        for (std::size_t v = 0; v < cpu.positions.size() && v < g.positions.size(); ++v) {
            worst = std::max(worst, glm::length(g.positions[v] - cpu.positions[v]));
        }
        INFO("worst vertex difference " << worst);
        std::cout << "measured: marching cubes GPU vs CPU, seed " << seed << ": "
                  << g.triangle_count() << " triangles, worst vertex " << worst << " m\n";
        CHECK(worst < 1.0e-5);
        CHECK(topology(g).closed());
    }
    // The sphere, and the triangle limit.
    const ScalarGrid sphere = sphere_field(32, 12.0, glm::dvec3(15.5, 15.2, 15.7));
    auto gpu = marching_cubes_gpu(*gl.backend, sphere, 0.0);
    REQUIRE(gpu.has_value());
    CHECK(surface_area(gpu.value()) ==
          Catch::Approx(surface_area(marching_cubes(sphere, 0.0))).epsilon(1.0e-6));
    CHECK_FALSE(marching_cubes_gpu(*gl.backend, sphere, 0.0, 10).has_value());
}
