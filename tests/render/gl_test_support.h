#pragma once

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "orrery/render/snapshot.h"

// Helpers for tests tagged [gl]. Creating a renderer has two stages: the
// OpenGL 4.6 context (capability probe) and the pipeline built in it. Only a
// context that cannot be created (no window system, software GDI driver, Mesa
// older than 4.6) skips the test with the reason; setting the environment
// variable ORRERY_REQUIRE_GL=1 turns that skip into a failure, for machines
// that must run the GL tests. Any failure after the context exists (shader
// compilation, texture allocation, framebuffer completeness) is a renderer
// fault and always fails the test.
namespace orrery::render::test {

inline std::optional<std::string> environment(const char* name) {
#ifdef _MSC_VER
    char* raw = nullptr;
    std::size_t length = 0;
    if (_dupenv_s(&raw, &length, name) != 0 || raw == nullptr) {
        return std::nullopt;
    }
    const std::unique_ptr<char, decltype(&std::free)> owner(raw, &std::free);
    return std::string(owner.get());
#else
    const char* value = std::getenv(name); // NOLINT(concurrency-mt-unsafe)
    if (value == nullptr) {
        return std::nullopt;
    }
    return std::string(value);
#endif
}

inline bool environment_flag(const char* name) {
    return environment(name) == std::optional<std::string>("1");
}

enum class RendererOutcome : std::uint8_t { ready, context_unavailable, setup_failed };

struct RendererAttempt {
    RendererOutcome outcome = RendererOutcome::context_unavailable;
    std::optional<OffscreenRenderer> renderer;
    std::string message;
};

inline RendererAttempt
try_create_renderer(int width, int height, const HdrSettings& settings = {},
                    const HdrPipelineOverrides& overrides = {},
                    const ContextOptions& options = OffscreenRenderer::context_options()) {
    auto context = GlContext::create(options);
    if (!context) {
        return {RendererOutcome::context_unavailable, std::nullopt, context.error()};
    }
    auto renderer =
        OffscreenRenderer::create(std::move(context).value(), width, height, settings, overrides);
    if (!renderer) {
        return {RendererOutcome::setup_failed, std::nullopt, renderer.error()};
    }
    return {RendererOutcome::ready, std::move(renderer).value(), {}};
}

inline OffscreenRenderer renderer_or_skip(int width, int height, const HdrSettings& settings = {}) {
    RendererAttempt attempt = try_create_renderer(width, height, settings);
    switch (attempt.outcome) {
        case RendererOutcome::ready:
            break;
        case RendererOutcome::context_unavailable:
            if (environment_flag("ORRERY_REQUIRE_GL")) {
                FAIL("OpenGL 4.6 required (ORRERY_REQUIRE_GL=1) but unavailable: "
                     << attempt.message);
            }
            SKIP("no OpenGL 4.6 core context: " << attempt.message);
            break;
        case RendererOutcome::setup_failed:
            FAIL("OpenGL 4.6 context created, but the renderer could not be set up: "
                 << attempt.message);
            break;
    }
    return std::move(*attempt.renderer);
}

} // namespace orrery::render::test
