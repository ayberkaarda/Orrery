#pragma once

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <glm/ext/vector_double3.hpp>

#include "../gl_test_support.h"
#include "orrery/render/aero_compute.h"
#include "orrery/render/aero_field.h"
#include "orrery/render/gl_context.h"
#include "orrery/render/snapshot.h"

// Helpers of the aero visualisation tests: analytic fields on grids and the
// compute backend with the aero kernels in a hidden-window context (tests
// tagged [gl] skip without OpenGL 4.6 unless ORRERY_REQUIRE_GL=1).
namespace orrery::render::test {

inline ScalarGrid scalar_grid(std::size_t nx, std::size_t ny, std::size_t nz,
                              const glm::dvec3& origin, double spacing,
                              const std::function<double(const glm::dvec3&)>& f) {
    ScalarGrid grid;
    grid.grid.dims = {nx, ny, nz};
    grid.grid.origin = origin;
    grid.grid.spacing = spacing;
    grid.values.reserve(nx * ny * nz);
    for (std::size_t k = 0; k < nz; ++k) {
        for (std::size_t j = 0; j < ny; ++j) {
            for (std::size_t i = 0; i < nx; ++i) {
                grid.values.push_back(f(grid.grid.node_position(i, j, k)));
            }
        }
    }
    return grid;
}

inline VectorGrid vector_grid(std::size_t nx, std::size_t ny, std::size_t nz,
                              const glm::dvec3& origin, double spacing,
                              const std::function<glm::dvec3(const glm::dvec3&)>& f) {
    VectorGrid grid;
    grid.grid.dims = {nx, ny, nz};
    grid.grid.origin = origin;
    grid.grid.spacing = spacing;
    grid.values.reserve(nx * ny * nz);
    for (std::size_t k = 0; k < nz; ++k) {
        for (std::size_t j = 0; j < ny; ++j) {
            for (std::size_t i = 0; i < nx; ++i) {
                grid.values.push_back(f(grid.grid.node_position(i, j, k)));
            }
        }
    }
    return grid;
}

struct AeroCompute {
    GlContext context;
    // Declared after the context so it is destroyed first, while the
    // context is still current.
    std::unique_ptr<GlComputeBackend> backend;
};

inline AeroCompute aero_compute_or_skip() {
    auto context = GlContext::create(OffscreenRenderer::context_options());
    std::optional<AeroCompute> made;
    if (!context) {
        if (environment_flag("ORRERY_REQUIRE_GL")) {
            FAIL("OpenGL 4.6 required (ORRERY_REQUIRE_GL=1) but unavailable: " << context.error());
        }
        SKIP("no OpenGL 4.6 core context: " << context.error());
    } else {
        context.value().make_current();
        auto backend = make_aero_compute_backend();
        if (!backend) {
            FAIL("OpenGL 4.6 context created, but the compute backend failed: " << backend.error());
        }
        made.emplace(AeroCompute{std::move(context).value(), std::move(backend).value()});
    }
    return std::move(*made);
}

} // namespace orrery::render::test
