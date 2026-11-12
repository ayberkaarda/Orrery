#pragma once

#include <catch2/catch_test_macros.hpp>

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "../../render/gl_test_support.h"
#include "orrery/render/gl_compute_backend.h"
#include "orrery/render/gl_context.h"
#include "orrery/render/snapshot.h"

// OpenGL compute for the LBM tests tagged [gl]: a hidden-window OpenGL 4.6
// context (never shown) and the compute backend in it. A context that cannot
// be created skips the test with the reason (ORRERY_REQUIRE_GL=1 turns that
// into a failure); a backend that fails once the context exists always fails.
namespace orrery::aero::lbm::test {

struct GlCompute {
    render::GlContext context;
    // Declared after the context so it is destroyed first, while the
    // context is still current.
    std::unique_ptr<render::GlComputeBackend> backend;
};

inline GlCompute gl_compute_or_skip() {
    auto context = render::GlContext::create(render::OffscreenRenderer::context_options());
    std::optional<GlCompute> made;
    if (!context) {
        if (render::test::environment_flag("ORRERY_REQUIRE_GL")) {
            FAIL("OpenGL 4.6 required (ORRERY_REQUIRE_GL=1) but unavailable: " << context.error());
        }
        SKIP("no OpenGL 4.6 core context: " << context.error());
    } else {
        context.value().make_current();
        auto backend = render::GlComputeBackend::create();
        if (!backend) {
            FAIL("OpenGL 4.6 context created, but the compute backend failed: " << backend.error());
        }
        made.emplace(GlCompute{std::move(context).value(), std::move(backend).value()});
    }
    return std::move(*made);
}

} // namespace orrery::aero::lbm::test
