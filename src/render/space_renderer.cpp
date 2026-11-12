#include "orrery/render/space_renderer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <span>
#include <utility>

#include <glm/ext/matrix_double3x3.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>

#include "gl_loader.h"
#include "orrery/core/units.h"
#include "orrery/render/embedded_shaders.h"
#include "orrery/render/icosphere.h"
#include "orrery/render/log.h"

namespace orrery::render {

namespace {

// Scene-linear irradiance at 1 AU from a 1 L_sun star. With the default
// exposure an Earth-like albedo of 0.3 then reaches a radiance of about 0.6.
constexpr double kIrradianceAt1Au = 6.0;
// Ambient irradiance as a fraction of the body's light: keeps the night side
// from being an exact silhouette without washing out the terminator.
constexpr float kAmbientFraction = 0.0025F;
// Photosphere radiance of stars relative to their unit-luminance colour
// (BodyAppearance::emission) and the halo colour derived from it.
constexpr double kStarGlowStrength = 0.11;
// Milky Way surface brightness (scene-linear) at the galactic centre.
constexpr double kMilkyWayIntensity = 0.03;
// Star sprites: peak radiance = kStarPeak * flux / sigma^2.
constexpr double kStarPeak = 2.4;
constexpr double kStarMagnitudeGamma = 0.85;

struct Light {
    glm::dvec3 world{0.0};
    double luminosity = 0.0;
    glm::dvec3 color{1.0};
    std::size_t index = 0;
};

glm::vec4 vec4_of(const glm::dvec3& v, double w) {
    return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z),
            static_cast<float>(w)};
}

// Unit vector along v, or nothing if v is zero or not finite. Scaled by its
// largest component first, so that tiny vectors whose squared length
// underflows still have a direction.
std::optional<glm::dvec3> unit_direction(const glm::dvec3& v) noexcept {
    const double scale = std::max({std::abs(v.x), std::abs(v.y), std::abs(v.z)});
    if (!(scale > 0.0) || !std::isfinite(scale)) {
        return std::nullopt;
    }
    const glm::dvec3 scaled = v / scale;
    return scaled / glm::length(scaled);
}

double luminance(const glm::dvec3& c) noexcept {
    return 0.2126 * c.r + 0.7152 * c.g + 0.0722 * c.b;
}

double smoothstep(double edge0, double edge1, double x) noexcept {
    const double t = std::clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

ViewFrustum frustum_of(const Camera& camera, double aspect) {
    const glm::dquat& q = camera.orientation();
    ViewFrustum f;
    f.right = q * glm::dvec3(1.0, 0.0, 0.0);
    f.up = q * glm::dvec3(0.0, 1.0, 0.0);
    f.back = q * glm::dvec3(0.0, 0.0, 1.0);
    f.tan_half_fov_y = std::tan(0.5 * camera.lens().vertical_fov);
    f.aspect = aspect;
    return f;
}

core::Result<Program, std::string> build_program(const shaders::EmbeddedShader& vertex,
                                                 const shaders::EmbeddedShader& fragment,
                                                 std::vector<ShaderSource> fragment_includes) {
    const std::array<StageSource, 2> stages{
        StageSource{ShaderStage::vertex, {vertex.name, vertex.source}, {}},
        StageSource{
            ShaderStage::fragment, {fragment.name, fragment.source}, std::move(fragment_includes)},
    };
    return Program::create(stages);
}

// Grows a dynamic buffer to hold at least `bytes` (geometric growth). Returns
// the buffer, or nullptr if it could not be allocated.
Buffer* ensure_capacity(std::optional<Buffer>& buffer, std::size_t bytes, std::string_view what) {
    if (buffer && buffer->size() >= bytes) {
        return &*buffer;
    }
    std::size_t capacity = buffer ? buffer->size() : std::size_t{4096};
    while (capacity < bytes) {
        capacity *= 2U;
    }
    auto created = Buffer::create_uninitialized(capacity, true);
    if (!created) {
        log_message(LogLevel::error,
                    std::string("space renderer: ") + std::string(what) + ": " + created.error());
        buffer.reset();
        return nullptr;
    }
    buffer = std::move(created).value();
    return &*buffer;
}

struct LodRange {
    std::size_t first_index = 0;
    std::size_t index_count = 0;
    std::size_t base_vertex = 0;
};

struct StarVertex {
    glm::vec4 direction_sigma;
    glm::vec4 color_peak;
};

struct TrailInstance {
    glm::vec4 p0_fade0;
    glm::vec4 p1_fade1;
    glm::vec4 color;
};

} // namespace

glm::dvec3 world_position(const core::SimSnapshot& snapshot, std::size_t index) {
    const double au = core::constants::astronomical_unit.value();
    const auto& p = snapshot.positions()[index];
    return glm::dvec3(p.x.value(), p.y.value(), p.z.value()) * au;
}

glm::vec4 trail_color(const BodyAppearance& appearance, double brightness) noexcept {
    glm::dvec3 base = appearance.albedo;
    if (appearance.style == SurfaceStyle::star) {
        base = blackbody_linear_srgb(appearance.temperature);
    } else if (appearance.style == SurfaceStyle::probe ||
               appearance.style == SurfaceStyle::black_hole) {
        base = appearance.style == SurfaceStyle::probe ? appearance.emission : appearance.secondary;
    } else if (appearance.style == SurfaceStyle::terrestrial) {
        base = glm::dvec3(0.25, 0.45, 0.95); // the ocean albedo is too dark to read as a hue
    }
    const double y = std::max(luminance(base), 1.0e-3);
    glm::dvec3 hue = base / y;
    // Desaturate a little so that trails stay quieter than the bodies.
    hue = glm::mix(glm::dvec3(1.0), hue, 0.75);
    hue /= std::max(luminance(hue), 1.0e-3);
    const glm::dvec3 color = hue * 0.28 * brightness;
    return vec4_of(color, 0.9);
}

PreparedBodies prepare_bodies(const core::SimSnapshot& snapshot, const Camera& camera,
                              const FrameInfo& frame, const RenderSettings& raw_settings) {
    const RenderSettings settings = sanitized(raw_settings);
    const double aspect = Camera::usable_aspect(frame.aspect());
    const double pps = pixel_scale(camera.lens().vertical_fov, frame.height);
    const ViewFrustum frustum = frustum_of(camera, aspect);
    const std::size_t count = snapshot.size();

    std::vector<BodyAppearance> appearances;
    appearances.reserve(count);
    std::vector<Light> lights;
    for (std::size_t i = 0; i < count; ++i) {
        appearances.push_back(
            appearance_for(snapshot.names()[i], snapshot.kinds()[i], snapshot.masses()[i]));
        if (appearances.back().style == SurfaceStyle::star) {
            lights.push_back({world_position(snapshot, i), appearances.back().luminosity,
                              blackbody_linear_srgb(appearances.back().temperature), i});
        }
    }

    PreparedBodies out;
    out.screen.reserve(count);
    std::array<std::vector<BodyInstance>, kLodCount> by_lod;
    for (std::size_t i = 0; i < count; ++i) {
        const BodyAppearance& a = appearances[i];
        const core::BodyKindTag tag = snapshot.kind_tag(i);
        const glm::dvec3 world = world_position(snapshot, i);
        const glm::dvec3 rel = camera.to_render_double(world);
        const double distance = glm::length(rel);

        BodyScreenInfo info;
        info.id = snapshot.ids()[i];
        info.index = i;
        info.distance = distance;
        info.physical_radius = physical_radius(snapshot.radii()[i], a);
        const double min_px = min_screen_radius_px(tag, settings.min_radius_scale);
        info.visual_radius =
            visual_radius(info.physical_radius, distance, pps, min_px, settings.scale_mode);
        const bool inside = !(distance > info.visual_radius * 1.000001);
        const bool in_view = !inside && std::isfinite(distance) &&
                             sphere_in_frustum(frustum, rel, info.visual_radius);
        const double depth = -glm::dot(rel, frustum.back);
        if (depth > 0.0) {
            const double ndc_x =
                glm::dot(rel, frustum.right) / (depth * frustum.tan_half_fov_y * aspect);
            const double ndc_y = glm::dot(rel, frustum.up) / (depth * frustum.tan_half_fov_y);
            info.pixel = {0.5 * (ndc_x + 1.0) * frame.width, 0.5 * (1.0 - ndc_y) * frame.height};
        }
        info.visible = in_view && depth > 0.0;
        if (!in_view) {
            ++out.culled;
            out.screen.push_back(info);
            continue;
        }
        // Screen bounds from the camera-space geometry: the off-axis ellipse
        // for labels and sprites, its worst magnification for the LOD.
        const ProjectedSphere projected = project_sphere(info.visual_radius, distance, depth, pps);
        info.radius_px = projected.radius_px;
        info.lod = select_lod(projected.lod_scale_px, settings.lod_bias);
        // Sprites stay finite when the sphere reaches the eye plane.
        const double disc_px = std::min(
            info.radius_px, 4.0 * static_cast<double>(std::max(frame.width, frame.height)));

        BodyInstance inst;
        inst.centre_radius = vec4_of(rel, info.visual_radius);
        inst.albedo_style = vec4_of(a.albedo, static_cast<double>(a.style));
        inst.secondary_specular = vec4_of(a.secondary, a.specular);
        inst.emission = vec4_of(a.emission, 0.0);
        inst.axis = vec4_of(a.spin_axis, 0.0);
        // Marker weight: how far the clamp enlarged the body (0 = true size).
        const double true_px = projected_radius_px(info.physical_radius, distance, pps);
        const double marker = settings.scale_mode == ScaleMode::clamped
                                  ? std::clamp(1.0 - true_px / min_px, 0.0, 1.0)
                                  : 0.0;
        const double seed = std::fmod(static_cast<double>(info.id.value) * 0.6180339887498949, 1.0);

        // APPROXIMATION: lighting geometry - one directional light per body,
        // from the star that gives it the highest irradiance, evaluated at the
        // body centre, with irradiance L (1 AU / d)^p and a display default of
        // p = 0.5 instead of the physical 2 - exact direction for a single
        // star (the star subtends far less than the body-star distance);
        // brightness ratios between planets are compressed unless p = 2.
        // Brightest star as seen from this body (never the body itself). A
        // star at the body's own position has no direction and is skipped;
        // with no other star the headlight below is used.
        const Light* best = nullptr;
        double best_irradiance = 0.0;
        glm::dvec3 best_direction{0.0};
        for (const Light& light : lights) {
            if (light.index == i) {
                continue;
            }
            const std::optional<glm::dvec3> direction = unit_direction(light.world - world);
            if (!direction) {
                continue;
            }
            const double e = relative_irradiance(light.luminosity, glm::length(light.world - world),
                                                 settings.light_falloff_exponent);
            if (e > best_irradiance) {
                best_irradiance = e;
                best = &light;
                best_direction = *direction;
            }
        }
        if (best != nullptr) {
            inst.light_seed = vec4_of(best_direction, seed);
            inst.irradiance = vec4_of(best->color * best_irradiance * kIrradianceAt1Au, marker);
        } else {
            // No star: a soft headlight from the camera keeps bodies readable.
            const glm::dvec3 to_eye = distance > 0.0 ? -rel / distance : glm::dvec3(0.0, 0.0, 1.0);
            inst.light_seed = vec4_of(to_eye, seed);
            inst.irradiance = vec4_of(glm::dvec3(0.6 * kIrradianceAt1Au), marker);
        }
        by_lod[info.lod].push_back(inst);

        if (a.style == SurfaceStyle::star && settings.star_glow) {
            GlowInstance glow;
            glow.centre_kind = vec4_of(rel, 0.0);
            glow.color_disc = vec4_of(a.emission * kStarGlowStrength, disc_px);
            out.glows.push_back(glow);
        } else if (a.style == SurfaceStyle::black_hole) {
            GlowInstance glow;
            glow.centre_kind = vec4_of(rel, 1.0);
            glow.color_disc = vec4_of(a.secondary * 0.35, disc_px);
            out.glows.push_back(glow);
        }
        out.screen.push_back(info);
    }

    for (std::size_t lod = 0; lod < kLodCount; ++lod) {
        out.lod_first[lod] = out.instances.size();
        out.lod_count[lod] = by_lod[lod].size();
        out.instances.insert(out.instances.end(), by_lod[lod].begin(), by_lod[lod].end());
    }
    return out;
}

struct SpaceRenderer::Gpu {
    Program body;
    Program glow;
    Program stars;
    Program milky_way;
    Program trail;
    Buffer sphere_vertices;
    Buffer sphere_indices;
    std::array<LodRange, kLodCount> lods{};
    VertexArray body_vao;
    Buffer star_vertices;
    VertexArray star_vao;
    VertexArray glow_vao;
    VertexArray trail_vao;
    VertexArray empty_vao;
    std::optional<Buffer> body_instances;
    std::optional<Buffer> glow_instances;
    std::optional<Buffer> trail_instances;
    std::vector<TrailInstance> trail_scratch;
};

SpaceRenderer::SpaceRenderer(std::unique_ptr<Gpu> gpu, std::size_t star_count) noexcept
    : gpu_(std::move(gpu)), star_count_(star_count) {}
SpaceRenderer::SpaceRenderer(SpaceRenderer&& other) noexcept = default;
SpaceRenderer& SpaceRenderer::operator=(SpaceRenderer&& other) noexcept = default;
SpaceRenderer::~SpaceRenderer() = default;

core::Result<SpaceRenderer, std::string> SpaceRenderer::create(const StarfieldParams& starfield) {
    const ShaderSource log_depth{shaders::log_depth.name, shaders::log_depth.source};
    const ShaderSource noise{shaders::noise.name, shaders::noise.source};

    auto body = build_program(shaders::body_vert, shaders::body_frag, {log_depth, noise});
    if (!body) {
        return core::failure("space renderer: " + body.error());
    }
    auto glow = build_program(shaders::glow_vert, shaders::glow_frag, {log_depth});
    if (!glow) {
        return core::failure("space renderer: " + glow.error());
    }
    auto stars = build_program(shaders::stars_vert, shaders::stars_frag, {});
    if (!stars) {
        return core::failure("space renderer: " + stars.error());
    }
    auto milky_way = build_program(shaders::fullscreen_vert, shaders::milky_way_frag, {noise});
    if (!milky_way) {
        return core::failure("space renderer: " + milky_way.error());
    }
    auto trail = build_program(shaders::trail_vert, shaders::trail_frag, {log_depth});
    if (!trail) {
        return core::failure("space renderer: " + trail.error());
    }

    // All LOD meshes in one vertex and one index buffer.
    std::vector<glm::vec3> vertices;
    std::vector<std::uint32_t> indices;
    std::array<LodRange, kLodCount> lods{};
    for (std::size_t lod = 0; lod < kLodCount; ++lod) {
        const IcosphereMesh mesh = make_icosphere(static_cast<int>(lod));
        lods[lod] = {indices.size(), mesh.indices.size(), vertices.size()};
        vertices.insert(vertices.end(), mesh.positions.begin(), mesh.positions.end());
        indices.insert(indices.end(), mesh.indices.begin(), mesh.indices.end());
    }
    auto sphere_vertices = Buffer::from(std::span<const glm::vec3>(vertices));
    if (!sphere_vertices) {
        return core::failure("space renderer sphere vertices: " + sphere_vertices.error());
    }
    auto sphere_indices = Buffer::from(std::span<const std::uint32_t>(indices));
    if (!sphere_indices) {
        return core::failure("space renderer sphere indices: " + sphere_indices.error());
    }

    // Starfield: sprite width and peak radiance from the magnitude.
    const std::vector<StarPoint> points = generate_starfield(starfield);
    std::vector<StarVertex> star_data;
    star_data.reserve(points.size());
    for (const StarPoint& p : points) {
        const auto m = static_cast<double>(p.magnitude);
        // APPROXIMATION: star brightness on screen - flux 10^(-0.4 * 0.85 m)
        // instead of 10^(-0.4 m) and a sprite width growing for bright stars -
        // compresses the 11.5-magnitude range of the sky into what a display
        // can show while keeping the brightness order.
        const double sigma = 0.42 + 0.6 * smoothstep(5.5, -1.5, m);
        const double flux = std::pow(10.0, -0.4 * kStarMagnitudeGamma * m);
        star_data.push_back(
            {glm::vec4(p.direction, static_cast<float>(sigma)),
             glm::vec4(p.color, static_cast<float>(kStarPeak * flux / (sigma * sigma)))});
    }
    std::optional<Buffer> star_vertices;
    if (!star_data.empty()) {
        auto created = Buffer::from(std::span<const StarVertex>(star_data));
        if (!created) {
            return core::failure("space renderer starfield: " + created.error());
        }
        star_vertices = std::move(created).value();
    } else {
        auto created = Buffer::create_uninitialized(sizeof(StarVertex));
        if (!created) {
            return core::failure("space renderer starfield: " + created.error());
        }
        star_vertices = std::move(created).value();
    }

    auto gpu = std::make_unique<Gpu>(Gpu{std::move(body).value(),
                                         std::move(glow).value(),
                                         std::move(stars).value(),
                                         std::move(milky_way).value(),
                                         std::move(trail).value(),
                                         std::move(sphere_vertices).value(),
                                         std::move(sphere_indices).value(),
                                         lods,
                                         VertexArray::create(),
                                         std::move(*star_vertices),
                                         VertexArray::create(),
                                         VertexArray::create(),
                                         VertexArray::create(),
                                         VertexArray::create(),
                                         std::nullopt,
                                         std::nullopt,
                                         std::nullopt,
                                         {}});

    // Bodies: binding 0 = unit sphere, binding 1 = instances.
    gpu->body_vao.set_vertex_buffer(0, gpu->sphere_vertices, 0, sizeof(glm::vec3));
    gpu->body_vao.set_float_attribute(0, 0, 3, 0);
    for (std::uint32_t k = 0; k < 7; ++k) {
        gpu->body_vao.set_float_attribute(1 + k, 1, 4,
                                          static_cast<std::uint32_t>(k * sizeof(glm::vec4)));
    }
    gpu->body_vao.set_binding_divisor(1, 1);
    gpu->body_vao.set_index_buffer(gpu->sphere_indices);

    gpu->star_vao.set_vertex_buffer(0, gpu->star_vertices, 0, sizeof(StarVertex));
    gpu->star_vao.set_float_attribute(0, 0, 4, 0);
    gpu->star_vao.set_float_attribute(1, 0, 4, sizeof(glm::vec4));

    gpu->glow_vao.set_float_attribute(0, 0, 4, 0);
    gpu->glow_vao.set_float_attribute(1, 0, 4, sizeof(glm::vec4));
    gpu->glow_vao.set_binding_divisor(0, 1);

    gpu->trail_vao.set_float_attribute(0, 0, 4, 0);
    gpu->trail_vao.set_float_attribute(1, 0, 4, sizeof(glm::vec4));
    gpu->trail_vao.set_float_attribute(2, 0, 4, 2 * sizeof(glm::vec4));
    gpu->trail_vao.set_binding_divisor(0, 1);

    return SpaceRenderer(std::move(gpu), star_data.size());
}

SpaceDrawStats SpaceRenderer::draw(const SpaceView& view, const FrameInfo& frame) {
    SpaceDrawStats stats;
    if (view.snapshot == nullptr || view.camera == nullptr || !gpu_) {
        return stats;
    }
    Gpu& gpu = *gpu_;
    const RenderSettings settings = sanitized(view.settings);
    const Camera& camera = *view.camera;
    const core::SimSnapshot& snapshot = *view.snapshot;
    const double aspect = Camera::usable_aspect(frame.aspect());
    const glm::mat4 view_projection = camera.view_projection_matrix(aspect);
    const auto log_scale = static_cast<float>(log_depth_scale(camera.lens().log_depth_far));
    const auto near = static_cast<float>(camera.lens().near_plane);
    const glm::vec2 viewport(static_cast<float>(frame.width), static_cast<float>(frame.height));

    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);

    // 1. Sky at infinity, additive over the clear colour, no depth.
    if (settings.show_starfield) {
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);
        if (settings.milky_way_brightness > 0.0) {
            gpu.milky_way.set("u_inverse_view_projection",
                              camera.inverse_view_projection_matrix(aspect));
            gpu.milky_way.set("u_ecliptic_to_galactic", glm::mat3(ecliptic_to_galactic()));
            gpu.milky_way.set("u_intensity", static_cast<float>(kMilkyWayIntensity *
                                                                settings.milky_way_brightness));
            gpu.milky_way.use();
            gpu.empty_vao.bind();
            glDrawArrays(GL_TRIANGLES, 0, 3);
        }
        if (star_count_ > 0 && settings.star_brightness > 0.0) {
            glEnable(GL_PROGRAM_POINT_SIZE);
            gpu.stars.set("u_view_projection", view_projection);
            gpu.stars.set("u_brightness", static_cast<float>(settings.star_brightness));
            gpu.stars.set("u_pixel_scale",
                          static_cast<float>(std::clamp(frame.height / 1080.0, 0.75, 2.0)));
            gpu.stars.use();
            gpu.star_vao.bind();
            glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(star_count_));
            glDisable(GL_PROGRAM_POINT_SIZE);
            stats.stars = star_count_;
        }
        glDisable(GL_BLEND);
    }

    // 2. Bodies.
    const PreparedBodies prepared = prepare_bodies(snapshot, camera, frame, settings);
    stats.bodies_culled = prepared.culled;
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);
    Buffer* const body_instances =
        prepared.instances.empty()
            ? nullptr
            : ensure_capacity(gpu.body_instances, prepared.instances.size() * sizeof(BodyInstance),
                              "body instances");
    if (body_instances != nullptr) {
        body_instances->update(0, std::as_bytes(std::span(prepared.instances)));
        gpu.body_vao.set_vertex_buffer(1, *body_instances, 0, sizeof(BodyInstance));
        gpu.body.set("u_view_projection", view_projection);
        gpu.body.set("u_log_depth_scale", log_scale);
        gpu.body.set("u_ambient", kAmbientFraction);
        gpu.body.use();
        gpu.body_vao.bind();
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glFrontFace(GL_CCW);
        for (std::size_t lod = 0; lod < kLodCount; ++lod) {
            const std::size_t n = prepared.lod_count[lod];
            stats.per_lod[lod] = n;
            if (n == 0) {
                continue;
            }
            const LodRange& range = gpu.lods[lod];
            glDrawElementsInstancedBaseVertexBaseInstance(
                GL_TRIANGLES, static_cast<GLsizei>(range.index_count), GL_UNSIGNED_INT,
                // NOLINTNEXTLINE(performance-no-int-to-ptr): GL takes a byte offset as a pointer.
                reinterpret_cast<const void*>(range.first_index * sizeof(std::uint32_t)),
                static_cast<GLsizei>(n), static_cast<GLint>(range.base_vertex),
                static_cast<GLuint>(prepared.lod_first[lod]));
            stats.bodies_drawn += n;
        }
        glDisable(GL_CULL_FACE);
    }

    // 3. Trails: alpha blended, depth tested against the bodies, no depth write.
    if (settings.show_trails && view.trails != nullptr) {
        gpu.trail_scratch.clear();
        for (std::size_t i = 0; i < snapshot.size(); ++i) {
            const TrailRing* ring = view.trails->find(snapshot.ids()[i]);
            if (ring == nullptr || ring->empty()) {
                continue;
            }
            const BodyAppearance appearance =
                appearance_for(snapshot.names()[i], snapshot.kinds()[i], snapshot.masses()[i]);
            const glm::vec4 color = trail_color(appearance, settings.trail_brightness);
            segment_scratch_.clear();
            append_trail_segments(*ring, world_position(snapshot, i), camera, segment_scratch_);
            for (const TrailSegment& s : segment_scratch_) {
                gpu.trail_scratch.push_back(
                    {glm::vec4(s.p0, s.fade0), glm::vec4(s.p1, s.fade1), color});
            }
        }
        Buffer* const trail_instances =
            gpu.trail_scratch.empty()
                ? nullptr
                : ensure_capacity(gpu.trail_instances,
                                  gpu.trail_scratch.size() * sizeof(TrailInstance),
                                  "trail segments");
        if (trail_instances != nullptr) {
            trail_instances->update(0, std::as_bytes(std::span(gpu.trail_scratch)));
            gpu.trail_vao.set_vertex_buffer(0, *trail_instances, 0, sizeof(TrailInstance));
            gpu.trail.set("u_view_projection", view_projection);
            gpu.trail.set("u_viewport", viewport);
            gpu.trail.set("u_half_width", static_cast<float>(0.5 * settings.trail_width_px));
            gpu.trail.set("u_near", near);
            gpu.trail.set("u_log_depth_scale", log_scale);
            gpu.trail.use();
            gpu.trail_vao.bind();
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glDepthMask(GL_FALSE);
            glDrawArraysInstanced(GL_TRIANGLES, 0, 6,
                                  static_cast<GLsizei>(gpu.trail_scratch.size()));
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
            stats.trail_segments = gpu.trail_scratch.size();
        }
    }

    // 4. Glow sprites, additive, hidden by bodies in front.
    Buffer* const glow_instances =
        prepared.glows.empty()
            ? nullptr
            : ensure_capacity(gpu.glow_instances, prepared.glows.size() * sizeof(GlowInstance),
                              "glow sprites");
    if (glow_instances != nullptr) {
        glow_instances->update(0, std::as_bytes(std::span(prepared.glows)));
        gpu.glow_vao.set_vertex_buffer(0, *glow_instances, 0, sizeof(GlowInstance));
        gpu.glow.set("u_view_projection", view_projection);
        gpu.glow.set("u_viewport", viewport);
        gpu.glow.set("u_near", near);
        gpu.glow.set("u_log_depth_scale", log_scale);
        gpu.glow.use();
        gpu.glow_vao.bind();
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);
        glDepthMask(GL_FALSE);
        glDrawArraysInstanced(GL_TRIANGLES, 0, 6, static_cast<GLsizei>(prepared.glows.size()));
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        stats.glows = prepared.glows.size();
    }

    glBindVertexArray(0);
    glUseProgram(0);
    return stats;
}

} // namespace orrery::render
