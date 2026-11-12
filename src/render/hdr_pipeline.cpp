#include "orrery/render/hdr_pipeline.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <initializer_list>
#include <utility>

#include "gl_loader.h"
#include "orrery/render/embedded_shaders.h"

namespace orrery::render {

namespace {

constexpr int kMaxDimension = 16384;

} // namespace

struct HdrPipeline::Resources {
    Texture scene_color;
    Texture depth;
    std::optional<Texture> resolved_color;
    Texture srgb;
    Framebuffer scene_fbo;
    std::optional<Framebuffer> resolve_fbo;
    Framebuffer srgb_fbo;
    Program tone_map;
    VertexArray empty_vao;
};

HdrPipeline::HdrPipeline(int width, int height, const HdrSettings& settings, Resources resources)
    : width_(width), height_(height), settings_(settings),
      scene_color_(std::move(resources.scene_color)), depth_(std::move(resources.depth)),
      resolved_color_(std::move(resources.resolved_color)), srgb_(std::move(resources.srgb)),
      scene_fbo_(std::move(resources.scene_fbo)), resolve_fbo_(std::move(resources.resolve_fbo)),
      srgb_fbo_(std::move(resources.srgb_fbo)), tone_map_(std::move(resources.tone_map)),
      empty_vao_(std::move(resources.empty_vao)) {}

core::Result<HdrPipeline, std::string> HdrPipeline::create(int width, int height,
                                                           const HdrSettings& settings,
                                                           const HdrPipelineOverrides& overrides) {
    if (width < 1 || height < 1 || width > kMaxDimension || height > kMaxDimension) {
        return core::failure("HDR pipeline: unsupported size " + std::to_string(width) + "x" +
                             std::to_string(height));
    }
    // The scene pass needs multisample colour and depth textures.
    GLint max_samples = 1;
    constexpr std::array<GLenum, 3> kLimits{GL_MAX_SAMPLES, GL_MAX_COLOR_TEXTURE_SAMPLES,
                                            GL_MAX_DEPTH_TEXTURE_SAMPLES};
    for (const GLenum limit : kLimits) {
        GLint value = 1;
        glGetIntegerv(limit, &value);
        max_samples = limit == GL_MAX_SAMPLES ? value : std::min(max_samples, value);
    }
    HdrSettings applied = settings;
    applied.samples = std::clamp(settings.samples, 1, std::max(max_samples, 1));

    const std::array<StageSource, 2> stages{
        StageSource{ShaderStage::vertex,
                    {shaders::fullscreen_vert.name, shaders::fullscreen_vert.source},
                    {}},
        StageSource{ShaderStage::fragment,
                    overrides.tone_map_fragment.value_or(
                        ShaderSource{shaders::tonemap_frag.name, shaders::tonemap_frag.source}),
                    {}},
    };
    auto tone_map = Program::create(stages);
    if (!tone_map) {
        return core::failure("HDR pipeline: " + tone_map.error());
    }

    auto scene_color = Texture::create_2d(width, height, TextureFormat::rgba16f, applied.samples);
    if (!scene_color) {
        return core::failure("HDR pipeline scene colour: " + scene_color.error());
    }
    auto depth = Texture::create_2d(width, height, TextureFormat::depth32f, applied.samples);
    if (!depth) {
        return core::failure("HDR pipeline scene depth: " + depth.error());
    }
    Framebuffer scene_fbo = Framebuffer::create();
    scene_fbo.attach_color(0, scene_color.value());
    scene_fbo.attach_depth(depth.value());
    if (auto status = scene_fbo.check(); !status) {
        return core::failure("HDR pipeline scene target: " + status.error());
    }

    std::optional<Texture> resolved_color;
    std::optional<Framebuffer> resolve_fbo;
    if (applied.samples > 1) {
        auto resolved = Texture::create_2d(width, height, TextureFormat::rgba16f);
        if (!resolved) {
            return core::failure("HDR pipeline resolve colour: " + resolved.error());
        }
        resolved_color = std::move(resolved).value();
        resolve_fbo = Framebuffer::create();
        resolve_fbo->attach_color(0, *resolved_color);
        if (auto status = resolve_fbo->check(); !status) {
            return core::failure("HDR pipeline resolve target: " + status.error());
        }
    }

    auto srgb = Texture::create_2d(width, height,
                                   overrides.output_format.value_or(TextureFormat::srgb8_alpha8));
    if (!srgb) {
        return core::failure("HDR pipeline sRGB target: " + srgb.error());
    }
    Framebuffer srgb_fbo = Framebuffer::create();
    srgb_fbo.attach_color(0, srgb.value());
    if (auto status = srgb_fbo.check(); !status) {
        return core::failure("HDR pipeline sRGB target: " + status.error());
    }

    return HdrPipeline(width, height, applied,
                       Resources{std::move(scene_color).value(), std::move(depth).value(),
                                 std::move(resolved_color), std::move(srgb).value(),
                                 std::move(scene_fbo), std::move(resolve_fbo), std::move(srgb_fbo),
                                 std::move(tone_map).value(), VertexArray::create()});
}

void HdrPipeline::begin_scene() const {
    scene_fbo_.bind();
    glViewport(0, 0, width_, height_);
    glDisable(GL_FRAMEBUFFER_SRGB);
    glDisable(GL_BLEND);
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);
    const std::array<GLfloat, 4> clear{static_cast<GLfloat>(settings_.clear_color.x),
                                       static_cast<GLfloat>(settings_.clear_color.y),
                                       static_cast<GLfloat>(settings_.clear_color.z), 1.0F};
    glClearNamedFramebufferfv(scene_fbo_.id(), GL_COLOR, 0, clear.data());
    const GLfloat far_depth = 1.0F;
    glClearNamedFramebufferfv(scene_fbo_.id(), GL_DEPTH, 0, &far_depth);
}

void HdrPipeline::resolve() const {
    if (resolve_fbo_) {
        glBlitNamedFramebuffer(scene_fbo_.id(), resolve_fbo_->id(), 0, 0, width_, height_, 0, 0,
                               width_, height_, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    }

    srgb_fbo_.bind();
    glViewport(0, 0, width_, height_);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glEnable(GL_FRAMEBUFFER_SRGB);
    hdr_texture().bind(0);
    tone_map_.set("u_exposure", static_cast<float>(settings_.exposure));
    tone_map_.use();
    empty_vao_.bind();
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glDisable(GL_FRAMEBUFFER_SRGB);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void HdrPipeline::begin_overlay() const {
    srgb_fbo_.bind();
    glViewport(0, 0, width_, height_);
    glDisable(GL_FRAMEBUFFER_SRGB);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_MULTISAMPLE);
}

ImageRgba8 HdrPipeline::read_srgb() const {
    ImageRgba8 image;
    image.width = static_cast<std::uint32_t>(width_);
    image.height = static_cast<std::uint32_t>(height_);
    std::vector<std::uint8_t> bottom_up(image.byte_size());
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    // Texture reads return the stored (sRGB-encoded) bytes without conversion.
    glGetTextureImage(srgb_.id(), 0, GL_RGBA, GL_UNSIGNED_BYTE,
                      static_cast<GLsizei>(bottom_up.size()), bottom_up.data());
    image.pixels.resize(image.byte_size());
    const std::size_t row = std::size_t{image.width} * 4U;
    for (std::uint32_t y = 0; y < image.height; ++y) {
        const std::size_t source = std::size_t{image.height - 1U - y} * row;
        std::copy_n(bottom_up.begin() + static_cast<std::ptrdiff_t>(source), row,
                    image.pixels.begin() + static_cast<std::ptrdiff_t>(std::size_t{y} * row));
    }
    return image;
}

} // namespace orrery::render
