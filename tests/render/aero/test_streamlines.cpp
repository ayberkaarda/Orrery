#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <numbers>
#include <optional>
#include <vector>

#include <glm/geometric.hpp>

#include "aero_test_support.h"
#include "orrery/render/aero_streamlines.h"

// Streamlines and particles (docs/aero-visualisation.md, "Streamlines and
// particles"): closed orbits in a circular flow, stopping rules, seeding,
// and the GPU kernels against the CPU reference.

using namespace orrery::render;
using orrery::render::test::aero_compute_or_skip;
using orrery::render::test::vector_grid;

namespace {

// Solid-body rotation about the z axis, linear in x and y, so trilinear
// interpolation is exact and only the integrator errs.
VectorGrid rotation(std::size_t nz) {
    return vector_grid(41, 41, nz, {-2.0, -2.0, nz > 1 ? -0.5 : 0.0}, 0.1,
                       [](const glm::dvec3& p) { return glm::dvec3(-p.y, p.x, 0.0); });
}

} // namespace

TEST_CASE("Streamlines of a circular flow close on themselves", "[render][aero][streamlines]") {
    for (const std::size_t nz : {std::size_t{1}, std::size_t{11}}) {
        const VectorGrid u = rotation(nz);
        StreamlineSettings settings;
        settings.step = 0.5; // 0.05 m of arc
        settings.max_points = 400;
        const std::vector<glm::dvec3> seeds{{1.0, 0.0, 0.0}, {0.0, -1.5, 0.0}};
        const std::vector<Streamline> lines = trace_streamlines(u, {}, seeds, settings);
        REQUIRE(lines.size() == 2);
        for (std::size_t s = 0; s < 2; ++s) {
            const Streamline& line = lines[s];
            const double radius = glm::length(seeds[s]);
            REQUIRE(line.points.size() == settings.max_points);
            double drift = 0.0;
            for (const glm::dvec3& p : line.points) {
                drift = std::max(drift, std::abs(glm::length(p) - radius));
            }
            INFO("radius " << radius << " drift " << drift);
            std::cout << "measured: circular-flow streamline r = " << radius << " nz = " << nz
                      << ": radius drift " << drift << " m over " << line.points.size()
                      << " steps\n";
            CHECK(drift < 1.0e-7); // RK4 on the exact field
            // After one revolution (2 pi r of arc) the line is back at its seed.
            const auto revolution =
                static_cast<std::size_t>(std::round(2.0 * std::numbers::pi * radius / 0.05));
            const double gap = glm::length(line.points[revolution] - seeds[s]);
            CHECK(gap < 0.05);
            double closest = 1.0e9;
            for (std::size_t i = revolution / 2; i < line.points.size(); ++i) {
                closest = std::min(closest, glm::length(line.points[i] - seeds[s]));
            }
            CHECK(closest < 0.026);
            CHECK(line.speed.front() == Catch::Approx(radius));
        }
    }
}

TEST_CASE("Streamlines stop at the grid edge, in bodies and at stagnation",
          "[render][aero][streamlines]") {
    // Uniform flow along +x through a 2D channel with a solid block.
    VectorGrid u = vector_grid(21, 11, 1, {0.0, 0.0, 0.0}, 0.1,
                               [](const glm::dvec3&) { return glm::dvec3(1.0, 0.0, 0.0); });
    std::vector<std::uint8_t> solid(u.values.size(), 0);
    for (std::size_t j = 0; j < 4; ++j) {
        solid[u.grid.index(12, j, 0)] = 1;
    }
    StreamlineSettings settings;
    settings.step = 0.5;
    settings.max_points = 1000;
    const std::vector<glm::dvec3> seeds{{0.05, 0.15, 0.0}, {0.05, 0.75, 0.0}, {5.0, 5.0, 0.0}};
    const std::vector<Streamline> lines = trace_streamlines(u, solid, seeds, settings);
    REQUIRE(lines.size() == 3);
    CHECK(lines[0].points.back().x < 1.15 + 1.0e-9); // stops before the block at x = 1.2
    CHECK(lines[0].points.back().x > 1.05);
    CHECK(lines[1].points.back().x > 1.9); // runs to the outlet
    CHECK(lines[1].points.back().x <= 2.0 + 1.0e-12);
    CHECK(lines[2].points.empty()); // seed outside the grid

    // Both directions: the seed in the middle, the line from upstream to downstream.
    settings.direction = TraceDirection::both;
    settings.max_points = 9;
    const std::vector<glm::dvec3> middle{{1.0, 0.75, 0.0}};
    const Streamline both = trace_streamlines(u, {}, middle, settings).front();
    REQUIRE(both.points.size() == 9);
    CHECK(both.points.front().x == Catch::Approx(0.8));
    CHECK(both.points.back().x == Catch::Approx(1.2));
    settings.direction = TraceDirection::backward;
    const Streamline back = trace_streamlines(u, {}, middle, settings).front();
    CHECK(back.points.back().x == Catch::Approx(1.0));
    CHECK(back.points.front().x < 1.0);

    // A stagnant field gives no line.
    VectorGrid still = vector_grid(5, 5, 1, {0.0, 0.0, 0.0}, 1.0,
                                   [](const glm::dvec3&) { return glm::dvec3(0.0); });
    CHECK(trace_streamlines(still, {}, middle, settings).front().points.empty());
}

TEST_CASE("Seed rakes", "[render][aero][streamlines]") {
    const std::vector<glm::dvec3> line = seed_line({0.0, 0.0, 0.0}, {1.0, 2.0, 0.0}, 5);
    REQUIRE(line.size() == 5);
    CHECK(line[4].y == 2.0);
    CHECK(line[2].x == 0.5);
    CHECK(seed_line({0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, 1).front().x == 1.0);
    GridGeometry g;
    g.dims = {11, 5, 3};
    g.spacing = 0.5;
    const std::vector<glm::dvec3> plane = seed_plane(g, Axis::x, -1.0, 4, 2);
    REQUIRE(plane.size() == 8);
    CHECK(plane[0].x == 0.0); // clamped into the grid
    CHECK(plane[0].y == Catch::Approx(0.25));
    CHECK(plane[0].z == Catch::Approx(0.25));
    CHECK(plane[7].y == Catch::Approx(1.75));
    CHECK(plane[7].z == Catch::Approx(0.75));
}

TEST_CASE("Particles follow the circular flow with second-order accuracy",
          "[render][aero][streamlines]") {
    const VectorGrid u = rotation(1);
    glm::dvec3 p(1.0, 0.0, 0.0);
    const double dt = 2.0 * std::numbers::pi / 400.0;
    for (int i = 0; i < 400; ++i) {
        const std::optional<glm::dvec3> next = advect_particle(u, {}, p, dt);
        REQUIRE(next.has_value());
        p = *next;
    }
    // One revolution: midpoint rule error of order dt^2 per revolution.
    CHECK(glm::length(p - glm::dvec3(1.0, 0.0, 0.0)) < 2.0e-3);
    CHECK_FALSE(advect_particle(u, {}, {1.9, 1.9, 0.0}, 0.5).has_value()); // leaves the grid
}

TEST_CASE("Particle substeps stay in range for extreme and non-finite inputs",
          "[render][aero][streamlines]") {
    // The view used to convert ceil(cells / 0.5) to int before clamping it,
    // which is undefined for these inputs; the count is now checked first.
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const double huge = std::numeric_limits<double>::max();
    const double tiny = std::numeric_limits<double>::denorm_min();
    CHECK(particle_substeps(1.0, 1.0, 1.0) == 2);    // one spacing: two half steps
    CHECK(particle_substeps(0.25, 1.0, 1.0) == 1);   // under half a spacing
    CHECK(particle_substeps(0.0, 1.0, 1.0) == 1);    // a frame without time still writes a trace
    CHECK(particle_substeps(-1.0, 1.0, 1.0) == 2);   // backwards in time
    CHECK(particle_substeps(1.0, 1.0e9, 1.0) == 64); // capped
    CHECK(particle_substeps(1.0, huge, tiny) == 64); // the ratio overflows to infinity
    CHECK(particle_substeps(huge, huge, 1.0) == 64);
    CHECK(particle_substeps(1.0, nan, 1.0) == 64);
    CHECK(particle_substeps(1.0, inf, 1.0) == 64);
    CHECK(particle_substeps(nan, 1.0, 1.0) == 0); // NaN or infinite time scale: no advance
    CHECK(particle_substeps(inf, 1.0, 1.0) == 0);
    CHECK(particle_substeps(-inf, 1.0, 1.0) == 0);
    CHECK(particle_substeps(1.0, 1.0, 0.0) == 0);
    CHECK(particle_substeps(1.0, 1.0, nan) == 0);
}

TEST_CASE("Non-finite seeds, steps and velocities stop lines before the solid lookup",
          "[render][aero][streamlines]") {
    // A populated mask makes every position reach the nearest-node lookup;
    // a non-finite coordinate must stop there instead of being converted.
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    VectorGrid u = rotation(1);
    const std::vector<std::uint8_t> solid(u.values.size(), 0);
    const std::vector<glm::dvec3> seeds{{nan, 0.0, 0.0}, {0.5, nan, 0.0}, {inf, 0.0, 0.0}};
    for (const Streamline& line : trace_streamlines(u, solid, seeds, {})) {
        CHECK(line.points.empty());
    }
    StreamlineSettings nan_step;
    nan_step.step = nan;
    const std::vector<glm::dvec3> good{{1.0, 0.0, 0.0}};
    for (const Streamline& line : trace_streamlines(u, solid, good, nan_step)) {
        CHECK(line.points.size() <= 1);
    }
    CHECK_FALSE(advect_particle(u, solid, {nan, 0.0, 0.0}, 0.01).has_value());
    CHECK_FALSE(advect_particle(u, solid, {1.0, 0.0, 0.0}, nan).has_value());
    CHECK_FALSE(advect_particle(u, solid, {1.0, 0.0, 0.0}, inf).has_value());
    // A non-finite velocity sample sends the midpoint to NaN.
    for (glm::dvec3& v : u.values) {
        v = glm::dvec3(nan, 0.0, 0.0);
    }
    CHECK_FALSE(advect_particle(u, solid, {1.0, 0.0, 0.0}, 0.01).has_value());
    for (const Streamline& line : trace_streamlines(u, solid, good, {})) {
        CHECK(line.points.empty());
    }
}

TEST_CASE("Streamlines on the GPU match the CPU reference", "[render][aero][streamlines][gl]") {
    auto gl = aero_compute_or_skip();
    // A smooth swirl with a body: u = rotation + uniform drift, solid disc.
    VectorGrid u = vector_grid(48, 40, 6, {-1.2, -1.0, -0.1}, 0.05, [](const glm::dvec3& p) {
        return glm::dvec3(1.0 - 0.6 * p.y, 0.6 * p.x + 0.2 * std::sin(3.0 * p.x), 0.05 * p.z);
    });
    std::vector<std::uint8_t> solid(u.values.size(), 0);
    for (std::size_t k = 0; k < 6; ++k) {
        for (std::size_t j = 0; j < 40; ++j) {
            for (std::size_t i = 0; i < 48; ++i) {
                const glm::dvec3 p = u.grid.node_position(i, j, k);
                if (glm::length(glm::dvec3(p.x - 0.3, p.y, 0.0)) < 0.15) {
                    solid[u.grid.index(i, j, k)] = 1;
                }
            }
        }
    }
    StreamlineSettings settings;
    settings.step = 0.4;
    settings.max_points = 300;
    settings.direction = TraceDirection::both;
    const std::vector<glm::dvec3> seeds = seed_line({-1.1, -0.8, 0.05}, {-1.1, 0.8, 0.1}, 16);
    const std::vector<Streamline> cpu = trace_streamlines(u, solid, seeds, settings);
    auto gpu = trace_streamlines_gpu(*gl.backend, u, solid, seeds, settings);
    INFO((gpu ? std::string() : gpu.error()));
    REQUIRE(gpu.has_value());
    REQUIRE(gpu.value().size() == cpu.size());
    std::size_t compared = 0;
    double worst = 0.0;
    for (std::size_t s = 0; s < cpu.size(); ++s) {
        const Streamline& a = cpu[s];
        const Streamline& b = gpu.value()[s];
        INFO("seed " << s << " points " << a.points.size() << " / " << b.points.size());
        // Lines that end at a stopping rule may differ by a step or two.
        CHECK(std::abs(static_cast<long long>(a.points.size()) -
                       static_cast<long long>(b.points.size())) <= 2);
        const std::size_t n = std::min(a.points.size(), b.points.size());
        for (std::size_t i = 0; i < n; ++i) {
            worst = std::max(worst, glm::length(a.points[i] - b.points[i]));
            ++compared;
        }
    }
    INFO("compared " << compared << " points, worst " << worst << " m");
    std::cout << "measured: streamlines GPU vs CPU: " << compared << " points, worst " << worst
              << " m\n";
    CHECK(compared > 1000);
    CHECK(worst < 1.0e-4); // binary32 against binary64 over up to 300 steps
}

TEST_CASE("Particles on the GPU match the CPU reference", "[render][aero][streamlines][gl]") {
    auto gl = aero_compute_or_skip();
    const VectorGrid u = rotation(11);
    std::vector<glm::dvec3> positions;
    for (int i = 0; i < 200; ++i) {
        const double a = 0.031 * i;
        const double r = 0.2 + 0.0085 * i;
        positions.emplace_back(r * std::cos(a), r * std::sin(a), -0.4 + 0.004 * i);
    }
    positions.emplace_back(1.95, 1.97, 0.0); // leaves the grid
    const double dt = 0.05;
    auto gpu = advect_particles_gpu(*gl.backend, u, {}, positions, dt);
    INFO((gpu ? std::string() : gpu.error()));
    REQUIRE(gpu.has_value());
    REQUIRE(gpu.value().size() == positions.size());
    double worst = 0.0;
    for (std::size_t i = 0; i < positions.size(); ++i) {
        const std::optional<glm::dvec3> cpu = advect_particle(u, {}, positions[i], dt);
        REQUIRE(cpu.has_value() == gpu.value()[i].has_value());
        if (cpu) {
            worst = std::max(worst, glm::length(*cpu - *gpu.value()[i]));
        }
    }
    INFO("worst " << worst);
    std::cout << "measured: particles GPU vs CPU, one step: worst " << worst << " m\n";
    CHECK(worst < 1.0e-5);
    CHECK_FALSE(gpu.value().back().has_value());
}
