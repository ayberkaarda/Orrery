#pragma once

#include <functional>
#include <optional>
#include <string>

#include "orrery/core/result.h"
#include "orrery/render/gl_context.h"
#include "orrery/render/hdr_pipeline.h"
#include "orrery/render/image.h"

// Headless snapshots: render a scene offscreen through the HDR pipeline and
// read back 8-bit sRGB RGBA. Uses a hidden window for the context; where no
// window system or OpenGL 4.6 driver exists, creation fails with a message
// (tests skip on that message instead of failing).
namespace orrery::render {

// Called with the HDR scene framebuffer bound and cleared; draws the scene in
// scene-linear radiance with log depth. GL resources created inside the
// callback must also be destroyed inside it, or live in an object owned
// alongside the OffscreenRenderer.
using SceneCallback = std::function<void(const FrameInfo&)>;

// Owns one hidden-window context and the pipeline built in it. Contexts are
// not shared, so the renderer releases its GL objects with its own context
// current: destruction and move assignment make that context current, delete
// the pipeline, destroy the context and then make the previously current
// context current again (nothing is current afterwards if that was the
// renderer's own context). A moved-from renderer may only be destroyed or
// assigned to.
class OffscreenRenderer {
public:
    OffscreenRenderer(OffscreenRenderer&& other) noexcept;
    OffscreenRenderer& operator=(OffscreenRenderer&& other) noexcept;
    OffscreenRenderer(const OffscreenRenderer&) = delete;
    OffscreenRenderer& operator=(const OffscreenRenderer&) = delete;
    ~OffscreenRenderer();

    [[nodiscard]] static core::Result<OffscreenRenderer, std::string>
    create(int width, int height, const HdrSettings& settings = {});

    // The two stages of create(), separately: the hidden-window context
    // (fails when no window system or OpenGL 4.6 core driver is available)
    // and the pipeline built in that context (a failure here is a renderer
    // fault, not a missing capability).
    [[nodiscard]] static ContextOptions context_options();
    [[nodiscard]] static core::Result<OffscreenRenderer, std::string>
    create(GlContext context, int width, int height, const HdrSettings& settings = {},
           const HdrPipelineOverrides& overrides = {});

    // Renders one frame and leaves this renderer's context current. The scene
    // callback may use other contexts (for example a nested render_snapshot);
    // this context is made current again before the frame is resolved. Fails
    // if the driver reported a KHR_debug error (type ERROR or severity HIGH)
    // during the frame.
    [[nodiscard]] core::Result<ImageRgba8, std::string> render(const SceneCallback& scene);

    [[nodiscard]] GlContext& context() noexcept { return context_; }
    [[nodiscard]] HdrPipeline& pipeline() noexcept { return *pipeline_; }

private:
    OffscreenRenderer(GlContext context, HdrPipeline pipeline) noexcept;
    // Deletes the pipeline in its own context, then destroys the context.
    void release() noexcept;

    GlContext context_;
    std::optional<HdrPipeline> pipeline_;
};

// One-shot convenience: creates a renderer, renders one frame, destroys it,
// and makes the context that was current before the call current again.
[[nodiscard]] core::Result<ImageRgba8, std::string>
render_snapshot(int width, int height, const SceneCallback& scene,
                const HdrSettings& settings = {});

} // namespace orrery::render
