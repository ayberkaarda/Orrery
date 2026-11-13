#include "black_hole_pass.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_float3x3.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/matrix.hpp>

#include "gl_loader.h"
#include "orrery/core/profile.h"
#include "orrery/render/blackbody_spectrum.h"
#include "orrery/render/embedded_shaders.h"
#include "orrery/render/starfield.h"

namespace orrery::render::detail {

namespace {

constexpr double kPi = 3.141592653589793;
// Angle of one pixel on the reference screen of the sky look: 1080 rows over
// a 40 degree vertical field of view, tan(20 deg) / 540.
constexpr double kReferencePixelAngle = 6.740189523448191e-4;
// Scene-linear luminance of the disk at its peak temperature (g = 1) for
// disk brightness 1: the hottest ring sits on the ACES shoulder, so the
// Doppler-boosted side turns white while the receding side keeps its orange.
constexpr double kDiskRadiance = 2.2;
constexpr int kWorkgroupSize = 8;
// Stars are drawn into a cube face through a viewport that reaches this many
// texels past the face on every side, with the projection widened to match,
// so that a star whose centre lies just beyond the face still adds its share
// to it (points are clipped by their centre). The largest star sprite is
// about 18 texels wide (sky_cube_stars.vert.glsl).
constexpr int kStarGuardTexels = 16;

core::Result<Program, std::string> make_program(std::span<const StageSource> stages,
                                                std::string_view what) {
    auto program = Program::create(stages);
    if (!program) {
        return core::failure("black hole " + std::string(what) + ": " + program.error());
    }
    return program;
}

ShaderSource source_of(const shaders::EmbeddedShader& shader) {
    return {shader.name, shader.source};
}

// The six faces of a cube map in GL order (+X, -X, +Y, -Y, +Z, -Z) seen from
// the centre: forward and up vectors such that the face image, rendered with
// a 90 degree perspective, lands on the texture as the cube map lookup
// expects (OpenGL 4.6, table 8.19).
struct CubeFace {
    glm::vec3 forward;
    glm::vec3 up;
};
constexpr std::array<CubeFace, 6> kCubeFaces{{
    {{1.0F, 0.0F, 0.0F}, {0.0F, -1.0F, 0.0F}},
    {{-1.0F, 0.0F, 0.0F}, {0.0F, -1.0F, 0.0F}},
    {{0.0F, 1.0F, 0.0F}, {0.0F, 0.0F, 1.0F}},
    {{0.0F, -1.0F, 0.0F}, {0.0F, 0.0F, -1.0F}},
    {{0.0F, 0.0F, 1.0F}, {0.0F, -1.0F, 0.0F}},
    {{0.0F, 0.0F, -1.0F}, {0.0F, -1.0F, 0.0F}},
}};

struct SkyKey {
    double milky_way = -1.0;
    double stars = -1.0;
    std::size_t star_count = 0;
    friend bool operator==(const SkyKey&, const SkyKey&) = default;
};

// Restores the draw framebuffer and viewport on scope exit.
class FramebufferScope {
public:
    FramebufferScope() noexcept {
        glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &framebuffer_);
        glGetIntegerv(GL_VIEWPORT, viewport_.data());
    }
    FramebufferScope(const FramebufferScope&) = delete;
    FramebufferScope& operator=(const FramebufferScope&) = delete;
    FramebufferScope(FramebufferScope&&) = delete;
    FramebufferScope& operator=(FramebufferScope&&) = delete;
    ~FramebufferScope() {
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, static_cast<GLuint>(framebuffer_));
        glViewport(viewport_[0], viewport_[1], viewport_[2], viewport_[3]);
    }

private:
    GLint framebuffer_ = 0;
    std::array<GLint, 4> viewport_{};
};

glm::mat3 camera_basis(const Camera& camera) {
    const glm::dquat& q = camera.orientation();
    const glm::dmat3 basis(q * glm::dvec3(1.0, 0.0, 0.0), q * glm::dvec3(0.0, 1.0, 0.0),
                           q * glm::dvec3(0.0, 0.0, 1.0));
    const glm::mat3 single(basis);
    return single;
}

} // namespace

struct BlackHolePass::Impl {
    Program trace;
    Program composite;
    Program cube_stars;
    Texture blackbody;
    BlackbodyTable table;
    VertexArray empty;
    std::optional<Texture> environment;
    std::optional<Framebuffer> environment_fbo;
    SkyKey environment_key;
    std::optional<Texture> trace_disk;
    std::optional<Texture> trace_sky;
    TraceGrid grid;
    double luminance_kelvin = -1.0;
    double luminance = 1.0;

    core::Result<std::monostate, std::string> build_environment(const SkySource& sky);
    core::Result<std::monostate, std::string> ensure_trace_targets(const FrameInfo& frame);
    void dispatch(const LensingTarget& target, const Camera& camera, const FrameInfo& frame,
                  const RenderSettings& settings, bool geometry);
};

BlackHolePass::BlackHolePass(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}
BlackHolePass::BlackHolePass(BlackHolePass&& other) noexcept = default;
BlackHolePass& BlackHolePass::operator=(BlackHolePass&& other) noexcept = default;
BlackHolePass::~BlackHolePass() = default;

core::Result<BlackHolePass, std::string> BlackHolePass::create() {
    const std::array<StageSource, 1> trace_stages{
        StageSource{ShaderStage::compute,
                    source_of(shaders::black_hole_trace_comp),
                    {source_of(shaders::noise)}},
    };
    auto trace = make_program(trace_stages, "trace");
    if (!trace) {
        return core::failure(trace.error());
    }
    const std::array<StageSource, 2> composite_stages{
        StageSource{ShaderStage::vertex, source_of(shaders::fullscreen_vert), {}},
        StageSource{ShaderStage::fragment,
                    source_of(shaders::black_hole_composite_frag),
                    {source_of(shaders::log_depth)}},
    };
    auto composite = make_program(composite_stages, "composite");
    if (!composite) {
        return core::failure(composite.error());
    }
    const std::array<StageSource, 2> star_stages{
        StageSource{ShaderStage::vertex, source_of(shaders::sky_cube_stars_vert), {}},
        StageSource{ShaderStage::fragment, source_of(shaders::sky_cube_stars_frag), {}},
    };
    auto cube_stars = make_program(star_stages, "environment stars");
    if (!cube_stars) {
        return core::failure(cube_stars.error());
    }

    BlackbodyTable table = make_blackbody_table();
    auto blackbody =
        Texture::create_2d(static_cast<int>(table.texels.size()), 1, TextureFormat::rgba32f);
    if (!blackbody) {
        return core::failure("black hole black-body table: " + blackbody.error());
    }
    glTextureSubImage2D(blackbody.value().id(), 0, 0, 0, static_cast<GLsizei>(table.texels.size()),
                        1, GL_RGBA, GL_FLOAT, table.texels.data());

    auto impl = std::make_unique<Impl>(
        Impl{std::move(trace).value(), std::move(composite).value(), std::move(cube_stars).value(),
             std::move(blackbody).value(), std::move(table), VertexArray::create(), std::nullopt,
             std::nullopt, SkyKey{}, std::nullopt, std::nullopt, TraceGrid{}, -1.0, 1.0});
    return BlackHolePass(std::move(impl));
}

core::Result<std::monostate, std::string>
BlackHolePass::Impl::build_environment(const SkySource& sky) {
    const SkyKey key{sky.milky_way_intensity, sky.star_brightness, sky.star_count};
    if (environment && key == environment_key) {
        return std::monostate{};
    }
    if (!environment) {
        int levels = 1;
        while ((kEnvironmentFaceSize >> levels) > 0) {
            ++levels;
        }
        auto cube =
            Texture::create_cube(kEnvironmentFaceSize, TextureFormat::r11f_g11f_b10f, levels);
        if (!cube) {
            return core::failure("black hole environment: " + cube.error());
        }
        environment = std::move(cube).value();
        environment_fbo = Framebuffer::create();
    }
    if (!environment || !environment_fbo) {
        return core::failure(std::string("black hole environment: not allocated"));
    }

    const FramebufferScope restore;
    const GLuint fbo = environment_fbo->id();
    const glm::mat4 projection = glm::perspective(static_cast<float>(0.5 * kPi), 1.0F, 0.1F, 10.0F);
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE);
    for (std::size_t face = 0; face < kCubeFaces.size(); ++face) {
        glNamedFramebufferTextureLayer(fbo, GL_COLOR_ATTACHMENT0, environment->id(), 0,
                                       static_cast<GLint>(face));
        if (face == 0) {
            if (auto status = environment_fbo->check(); !status) {
                environment.reset();
                environment_fbo.reset();
                return core::failure("black hole environment target: " + status.error());
            }
        }
        environment_fbo->bind();
        glViewport(0, 0, kEnvironmentFaceSize, kEnvironmentFaceSize);
        const std::array<GLfloat, 4> black{0.0F, 0.0F, 0.0F, 1.0F};
        glClearNamedFramebufferfv(fbo, GL_COLOR, 0, black.data());
        const glm::mat4 view =
            glm::lookAt(glm::vec3(0.0F), kCubeFaces[face].forward, kCubeFaces[face].up);
        const glm::mat4 view_projection = projection * view;
        if (sky.milky_way != nullptr && sky.empty != nullptr && sky.milky_way_intensity > 0.0) {
            sky.milky_way->set("u_inverse_view_projection", glm::inverse(view_projection));
            sky.milky_way->set("u_ecliptic_to_galactic", glm::mat3(ecliptic_to_galactic()));
            sky.milky_way->set("u_intensity", static_cast<float>(sky.milky_way_intensity));
            sky.milky_way->use();
            sky.empty->bind();
            glDrawArrays(GL_TRIANGLES, 0, 3);
        }
        if (sky.stars != nullptr && sky.star_count > 0 && sky.star_brightness > 0.0) {
            constexpr int guarded = kEnvironmentFaceSize + 2 * kStarGuardTexels;
            glViewport(-kStarGuardTexels, -kStarGuardTexels, guarded, guarded);
            glEnable(GL_PROGRAM_POINT_SIZE);
            cube_stars.set("u_view_projection", view_projection);
            cube_stars.set("u_guard_scale",
                           static_cast<float>(kEnvironmentFaceSize) / static_cast<float>(guarded));
            cube_stars.set("u_face_to_world", glm::transpose(glm::mat3(view)));
            cube_stars.set("u_brightness", static_cast<float>(sky.star_brightness));
            cube_stars.set("u_reference_pixel_angle", static_cast<float>(kReferencePixelAngle));
            cube_stars.set("u_texel_angle", static_cast<float>(2.0 / kEnvironmentFaceSize));
            cube_stars.use();
            sky.stars->bind();
            glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(sky.star_count));
            glDisable(GL_PROGRAM_POINT_SIZE);
            glViewport(0, 0, kEnvironmentFaceSize, kEnvironmentFaceSize);
        }
    }
    glDisable(GL_BLEND);
    glGenerateTextureMipmap(environment->id());
    environment_key = key;
    return std::monostate{};
}

core::Result<std::monostate, std::string>
BlackHolePass::Impl::ensure_trace_targets(const FrameInfo& frame) {
    const TraceGrid wanted = trace_grid(frame);
    if (trace_disk && trace_sky && wanted.width == grid.width && wanted.height == grid.height) {
        return std::monostate{};
    }
    trace_disk.reset();
    trace_sky.reset();
    auto disk = Texture::create_2d(wanted.width, wanted.height, TextureFormat::rgba32f);
    if (!disk) {
        return core::failure("black hole trace target: " + disk.error());
    }
    auto sky = Texture::create_2d(wanted.width, wanted.height, TextureFormat::rgba32f);
    if (!sky) {
        return core::failure("black hole trace target: " + sky.error());
    }
    trace_disk = std::move(disk).value();
    trace_sky = std::move(sky).value();
    grid = wanted;
    return std::monostate{};
}

void BlackHolePass::Impl::dispatch(const LensingTarget& target, const Camera& camera,
                                   const FrameInfo& frame, const RenderSettings& settings,
                                   bool geometry) {
    if (!trace_disk || !trace_sky) {
        return;
    }
    const double aspect = Camera::usable_aspect(frame.aspect());
    const double tan_y = std::tan(0.5 * camera.lens().vertical_fov);
    const glm::dvec3 camera_position = -target.hole_render / target.mass_length;
    const double r0 = target.observer_radius;
    const double shift = r0 > 2.0 ? 1.0 / std::sqrt(1.0 - 2.0 / r0) : 1.0;
    const core::geodesic::DiskModel disk = lensing_disk();
    const int steps =
        std::clamp(settings.geodesic_steps, core::geodesic::kMinSteps, core::geodesic::kMaxSteps);
    if (settings.disk_temperature != luminance_kelvin) {
        luminance_kelvin = settings.disk_temperature;
        luminance = std::max(blackbody_luminance(luminance_kelvin), 1.0e-30);
    }
    const double log_min = std::log(table.min_kelvin);
    const double texels_per_log =
        static_cast<double>(table.texels.size() - 1) / (std::log(table.max_kelvin) - log_min);

    trace.set("u_trace_size", glm::ivec2(grid.width, grid.height));
    trace.set("u_viewport",
              glm::vec2(static_cast<float>(frame.width), static_cast<float>(frame.height)));
    trace.set("u_camera_basis", camera_basis(camera));
    trace.set("u_tan_half_fov",
              glm::vec2(static_cast<float>(tan_y * aspect), static_cast<float>(tan_y)));
    trace.set("u_camera_position", glm::vec3(camera_position));
    trace.set("u_disk_axis",
              glm::vec3(static_cast<float>(disk.axis.x), static_cast<float>(disk.axis.y),
                        static_cast<float>(disk.axis.z)));
    trace.set("u_disk_x", glm::vec3(1.0F, 0.0F, 0.0F));
    trace.set("u_disk_y", glm::vec3(0.0F, 1.0F, 0.0F));
    trace.set("u_disk_radii", glm::vec2(static_cast<float>(disk.inner_radius),
                                        static_cast<float>(disk.outer_radius)));
    trace.set("u_disk_enabled", settings.accretion_disk ? 1 : 0);
    trace.set("u_disk_temperature", static_cast<float>(settings.disk_temperature));
    trace.set("u_disk_radiance",
              static_cast<float>(kDiskRadiance * settings.disk_brightness / luminance));
    trace.set("u_blackbody_range",
              glm::vec2(static_cast<float>(log_min), static_cast<float>(texels_per_log)));
    trace.set("u_blackbody_texels", static_cast<float>(table.texels.size()));
    trace.set("u_max_steps", steps);
    trace.set("u_base_step", static_cast<float>(core::geodesic::base_step(steps)));
    trace.set("u_observer_shift", static_cast<float>(shift));
    trace.set("u_geometry_output", geometry ? 1 : 0);
    trace.set("u_environment_texel", static_cast<float>(2.0 / kEnvironmentFaceSize));
    trace.set("u_environment_max_lod",
              static_cast<float>(std::log2(static_cast<double>(kEnvironmentFaceSize))));
    trace.set("u_environment_scale", settings.show_starfield ? 1.0F : 0.0F);

    blackbody.bind(0);
    if (environment) {
        environment->bind(1);
        glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);
    }
    glBindImageTexture(0, trace_disk->id(), 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
    glBindImageTexture(1, trace_sky->id(), 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
    trace.use();
    glDispatchCompute(static_cast<GLuint>((grid.width + kWorkgroupSize - 1) / kWorkgroupSize),
                      static_cast<GLuint>((grid.height + kWorkgroupSize - 1) / kWorkgroupSize), 1U);
    glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_TEXTURE_UPDATE_BARRIER_BIT);
    glBindImageTexture(0, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
    glBindImageTexture(1, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
}

core::Result<std::monostate, std::string>
BlackHolePass::draw(const LensingTarget& target, const Camera& camera, const FrameInfo& frame,
                    const RenderSettings& settings, const SkySource& sky) {
    ORRERY_ZONE("bh_raymarch");
    Impl& p = *impl_;
    if (auto built = p.build_environment(sky); !built) {
        return built;
    }
    if (auto targets = p.ensure_trace_targets(frame); !targets) {
        return targets;
    }
    p.dispatch(target, camera, frame, settings, false);
    if (!p.trace_disk || !p.trace_sky || !p.environment) {
        return core::failure(std::string("black hole pass: resources missing"));
    }

    const double aspect = Camera::usable_aspect(frame.aspect());
    const double tan_y = std::tan(0.5 * camera.lens().vertical_fov);
    const double r0 = target.observer_radius;
    p.composite.set("u_trace_size", glm::ivec2(p.grid.width, p.grid.height));
    p.composite.set("u_viewport",
                    glm::vec2(static_cast<float>(frame.width), static_cast<float>(frame.height)));
    p.composite.set("u_camera_basis", camera_basis(camera));
    p.composite.set("u_tan_half_fov",
                    glm::vec2(static_cast<float>(tan_y * aspect), static_cast<float>(tan_y)));
    p.composite.set("u_camera_position", glm::vec3(-target.hole_render / target.mass_length));
    p.composite.set("u_observer_shift",
                    static_cast<float>(r0 > 2.0 ? 1.0 / std::sqrt(1.0 - 2.0 / r0) : 1.0));
    p.composite.set("u_analytic_shadow", r0 > core::geodesic::kPhotonSphereRadius ? 1 : 0);
    p.composite.set("u_environment_texel", static_cast<float>(2.0 / kEnvironmentFaceSize));
    p.composite.set("u_environment_max_lod",
                    static_cast<float>(std::log2(static_cast<double>(kEnvironmentFaceSize))));
    p.composite.set("u_environment_scale", settings.show_starfield ? 1.0F : 0.0F);
    p.composite.set("u_mass_length", static_cast<float>(target.mass_length));
    p.composite.set("u_log_depth_scale",
                    static_cast<float>(log_depth_scale(camera.lens().log_depth_far)));

    p.trace_disk->bind(0);
    p.trace_sky->bind(1);
    p.environment->bind(2);
    glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_ALWAYS);
    glDepthMask(GL_TRUE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE);
    p.composite.use();
    p.empty.bind();
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glDisable(GL_BLEND);
    glDepthFunc(GL_LESS);
    return std::monostate{};
}

core::Result<GeodesicTraceImage, std::string>
BlackHolePass::trace_geometry(const LensingTarget& target, const Camera& camera,
                              const FrameInfo& frame, const RenderSettings& settings) {
    Impl& p = *impl_;
    if (auto targets = p.ensure_trace_targets(frame); !targets) {
        return core::failure(targets.error());
    }
    p.dispatch(target, camera, frame, settings, true);
    if (!p.trace_disk || !p.trace_sky) {
        return core::failure(std::string("black hole trace: targets missing"));
    }
    GeodesicTraceImage image;
    image.width = p.grid.width;
    image.height = p.grid.height;
    const std::size_t count =
        static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height);
    image.disk.resize(count);
    image.sky.resize(count);
    const auto bytes = static_cast<GLsizei>(count * sizeof(glm::vec4));
    glGetTextureImage(p.trace_disk->id(), 0, GL_RGBA, GL_FLOAT, bytes, image.disk.data());
    glGetTextureImage(p.trace_sky->id(), 0, GL_RGBA, GL_FLOAT, bytes, image.sky.data());
    return image;
}

} // namespace orrery::render::detail
