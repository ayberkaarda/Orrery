#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <variant>

#include "orrery/core/result.h"
#include "orrery/render/black_hole.h"
#include "orrery/render/camera.h"
#include "orrery/render/gl_resources.h"
#include "orrery/render/hdr_pipeline.h"
#include "orrery/render/render_settings.h"

// GPU part of the black-hole lensing pass (private to orrery_render; the CPU
// part is orrery/render/black_hole.h). Owned by SpaceRenderer.
namespace orrery::render::detail {

// The unlensed sky as SpaceRenderer draws it, for the environment cube map.
struct SkySource {
    const Program* milky_way = nullptr; // milky_way.frag.glsl with fullscreen.vert.glsl
    const VertexArray* stars = nullptr; // starfield vertices (StarVertex layout)
    std::size_t star_count = 0;
    const VertexArray* empty = nullptr; // vertex array without buffers
    double milky_way_intensity = 0.0;   // scene-linear radiance scale of the band
    double star_brightness = 0.0;       // user factor
};

class BlackHolePass {
public:
    // Compiles the programs and uploads the black-body table. Requires a
    // current OpenGL 4.6 context.
    [[nodiscard]] static core::Result<BlackHolePass, std::string> create();

    BlackHolePass(BlackHolePass&& other) noexcept;
    BlackHolePass& operator=(BlackHolePass&& other) noexcept;
    BlackHolePass(const BlackHolePass&) = delete;
    BlackHolePass& operator=(const BlackHolePass&) = delete;
    ~BlackHolePass();

    // Traces and composites the lensed sky, shadow and disk into the bound
    // HDR scene framebuffer (additive, depth test ALWAYS, writes depth where
    // the shadow or the disk covers the pixel). The environment cube map is
    // (re)built first when the sky brightness changed. Fails without drawing
    // when a resource cannot be created.
    [[nodiscard]] core::Result<std::monostate, std::string>
    draw(const LensingTarget& target, const Camera& camera, const FrameInfo& frame,
         const RenderSettings& settings, const SkySource& sky);

    // Traces in geometry mode and reads the result back (tests).
    [[nodiscard]] core::Result<GeodesicTraceImage, std::string>
    trace_geometry(const LensingTarget& target, const Camera& camera, const FrameInfo& frame,
                   const RenderSettings& settings);

private:
    struct Impl;
    explicit BlackHolePass(std::unique_ptr<Impl> impl) noexcept;
    std::unique_ptr<Impl> impl_;
};

// Face size of the environment cube map: at 1080 rows over 40 degrees a
// screen pixel subtends 6.7e-4 rad, a texel 9.8e-4 rad at a face centre and
// 4.3e-4 rad at a corner.
inline constexpr int kEnvironmentFaceSize = 2048;

} // namespace orrery::render::detail
