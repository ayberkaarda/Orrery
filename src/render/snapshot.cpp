#include "orrery/render/snapshot.h"

#include <utility>

#include "gl_context_internal.h"
#include "gl_loader.h"

namespace orrery::render {

OffscreenRenderer::OffscreenRenderer(GlContext context, HdrPipeline pipeline) noexcept
    : context_(std::move(context)), pipeline_(std::move(pipeline)) {}

OffscreenRenderer::OffscreenRenderer(OffscreenRenderer&& other) noexcept
    : context_(std::move(other.context_)), pipeline_(std::exchange(other.pipeline_, std::nullopt)) {
}

OffscreenRenderer& OffscreenRenderer::operator=(OffscreenRenderer&& other) noexcept {
    if (this != &other) {
        release();
        context_ = std::move(other.context_);
        pipeline_ = std::exchange(other.pipeline_, std::nullopt);
    }
    return *this;
}

OffscreenRenderer::~OffscreenRenderer() {
    release();
}

void OffscreenRenderer::release() noexcept {
    if (!context_.valid()) {
        pipeline_.reset(); // moved-from: owns no GL objects
        return;
    }
    // Restore whatever else was current; if it was this context, nothing
    // is current afterwards.
    void* const previous = context_.is_current() ? nullptr : detail::current_native_context();
    context_.make_current();
    pipeline_.reset();
    {
        const GlContext released = std::move(context_);
    }
    if (previous != nullptr) {
        detail::make_native_context_current(previous);
    }
}

ContextOptions OffscreenRenderer::context_options() {
    ContextOptions options;
    options.width = 16;
    options.height = 16;
    options.title = "Orrery offscreen";
    options.visible = false;
    return options;
}

core::Result<OffscreenRenderer, std::string>
OffscreenRenderer::create(int width, int height, const HdrSettings& settings) {
    auto context = GlContext::create(context_options());
    if (!context) {
        return core::failure(context.error());
    }
    return create(std::move(context).value(), width, height, settings);
}

core::Result<OffscreenRenderer, std::string>
OffscreenRenderer::create(GlContext context, int width, int height, const HdrSettings& settings,
                          const HdrPipelineOverrides& overrides) {
    context.make_current();
    auto pipeline = HdrPipeline::create(width, height, settings, overrides);
    if (!pipeline) {
        return core::failure(pipeline.error());
    }
    return OffscreenRenderer(std::move(context), std::move(pipeline).value());
}

core::Result<ImageRgba8, std::string> OffscreenRenderer::render(const SceneCallback& scene) {
    context_.make_current();
    context_.reset_debug_error_count();
    pipeline_->begin_scene();
    if (scene) {
        scene(pipeline_->frame());
        context_.make_current();
    }
    pipeline_->resolve();
    ImageRgba8 image = pipeline_->read_srgb();
    glFinish();
    if (const std::size_t errors = context_.debug_error_count(); errors > 0) {
        return core::failure("OpenGL debug output reported " + std::to_string(errors) +
                             " error(s) while rendering the frame (see the render log)");
    }
    return image;
}

core::Result<ImageRgba8, std::string>
render_snapshot(int width, int height, const SceneCallback& scene, const HdrSettings& settings) {
    void* const previous = detail::current_native_context();
    core::Result<ImageRgba8, std::string> result = [&]() -> core::Result<ImageRgba8, std::string> {
        auto renderer = OffscreenRenderer::create(width, height, settings);
        if (!renderer) {
            return core::failure(renderer.error());
        }
        return renderer.value().render(scene);
    }();
    // The renderer and its context are gone; the caller's context (if any)
    // is still alive.
    if (previous != nullptr) {
        detail::make_native_context_current(previous);
    }
    return result;
}

} // namespace orrery::render
