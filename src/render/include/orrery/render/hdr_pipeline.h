#pragma once

#include <optional>
#include <string>
#include <vector>

#include <glm/ext/vector_double3.hpp>

#include "orrery/core/result.h"
#include "orrery/render/gl_resources.h"
#include "orrery/render/image.h"

// HDR frame pipeline (docs/rendering.md, section "HDR chain"):
//
//   scene pass -> RGBA16F colour + DEPTH32F (log depth), optionally MSAA
//              -> resolve (multisample blit)
//              -> tone map pass: exposure, ACES fitted
//              -> SRGB8_ALPHA8 target (hardware sRGB encode on store)
//              -> readback as 8-bit sRGB RGBA
//
// Bloom (spec 7.6) is not part of this phase; the tone map pass reads the
// resolved HDR texture directly.
namespace orrery::render {

struct HdrSettings {
    double exposure = 1.0;
    int samples = 4;             // MSAA samples of the scene pass (1 = off)
    glm::dvec3 clear_color{0.0}; // scene-linear background
};

// Replacement inputs for create(), used to exercise its failure handling
// (tests inject a broken tone map shader or an unusable output format).
// Default-constructed overrides build the production pipeline. The shader
// source is only read during create().
struct HdrPipelineOverrides {
    std::optional<ShaderSource> tone_map_fragment; // instead of tonemap.frag.glsl
    std::optional<TextureFormat> output_format;    // instead of SRGB8_ALPHA8
};

struct FrameInfo {
    int width = 0;
    int height = 0;
    [[nodiscard]] double aspect() const noexcept {
        return height > 0 ? static_cast<double>(width) / height : 1.0;
    }
};

class HdrPipeline {
public:
    // Requires a current OpenGL 4.6 context. width and height in [1, 16384].
    [[nodiscard]] static core::Result<HdrPipeline, std::string>
    create(int width, int height, const HdrSettings& settings,
           const HdrPipelineOverrides& overrides = {});

    // Binds the scene framebuffer, sets the viewport and depth state and
    // clears colour (to settings.clear_color) and depth (to 1).
    void begin_scene() const;
    // Resolves MSAA and tone maps into the sRGB target.
    void resolve() const;
    // Binds the sRGB target (valid after resolve()) for display-referred
    // overlays such as the user interface: colours are written as stored
    // sRGB-encoded values (GL_FRAMEBUFFER_SRGB off), without depth.
    void begin_overlay() const;
    // Reads the sRGB target, rows top to bottom.
    [[nodiscard]] ImageRgba8 read_srgb() const;
    // Reads the resolved scene-linear colour (valid after resolve()) as
    // RGBA floats, rows bottom to top (tests: finiteness, radiometry).
    [[nodiscard]] std::vector<float> read_hdr() const;
    // Copies the sRGB target (after resolve() and any overlay) to the default
    // framebuffer of the current context, scaled to width x height (nearest
    // when the sizes match, linear otherwise). The stored sRGB-encoded bytes
    // are copied unchanged (GL_FRAMEBUFFER_SRGB off), which is what a window
    // without an sRGB-capable default framebuffer displays correctly.
    void present(int width, int height) const;

    [[nodiscard]] FrameInfo frame() const noexcept { return {width_, height_}; }
    [[nodiscard]] const HdrSettings& settings() const noexcept { return settings_; }
    void set_exposure(double exposure) noexcept { settings_.exposure = exposure; }
    void set_clear_color(const glm::dvec3& color) noexcept { settings_.clear_color = color; }
    // Resolved scene-linear colour (valid after resolve()).
    [[nodiscard]] const Texture& hdr_texture() const noexcept {
        return resolved_color_ ? *resolved_color_ : scene_color_;
    }

private:
    struct Resources;
    HdrPipeline(int width, int height, const HdrSettings& settings, Resources resources);

    int width_;
    int height_;
    HdrSettings settings_;
    Texture scene_color_; // multisample when settings_.samples > 1
    Texture depth_;
    std::optional<Texture> resolved_color_; // single-sample copy when MSAA is on
    Texture srgb_;
    Framebuffer scene_fbo_;
    std::optional<Framebuffer> resolve_fbo_;
    Framebuffer srgb_fbo_;
    Program tone_map_;
    VertexArray empty_vao_;
};

} // namespace orrery::render