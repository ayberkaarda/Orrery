#include "orrery/render/entry_view.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <span>
#include <utility>

#include <glm/ext/matrix_double4x4.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>

#include "gl_loader.h"
#include "orrery/render/embedded_entry_shaders.h"
#include "orrery/render/embedded_shaders.h"
#include "orrery/render/icosphere.h"
#include "orrery/render/log_depth.h"

namespace orrery::render {

namespace {

constexpr int kLutSize = 256;
constexpr int kPlanetSubdivisions = 6;
constexpr double kGridStepDeg = 10.0;
constexpr double kLineDepthBias = 2.0e-5;
// Ribbon outline width beyond the core on each side [px at 1080 rows].
constexpr double kOutlinePx = 1.5;
constexpr double kGroundHalfWidthPx = 1.1;
constexpr double kCurtainOpacity = 0.22;
// Optical depth of one scale height of the haze (entry_atmosphere.frag.glsl).
constexpr double kHazeStrength = 0.035;

// Display-referred colours of the overlay-like layers (linear).
constexpr glm::dvec3 kOutline{0.012, 0.014, 0.020};
constexpr glm::dvec3 kGroundTrack{0.80, 0.78, 0.70};
constexpr glm::dvec3 kCurtain{0.55, 0.62, 0.72};

struct PlanetLook {
    glm::dvec3 day;
    glm::dvec3 night;
    glm::dvec3 grid;
    glm::dvec3 haze;
};

PlanetLook look(EntryPlanetStyle style) {
    // Muted, dark albedos: the coloured ribbon must stay the brightest and
    // most saturated thing in the scene.
    if (style == EntryPlanetStyle::mars) {
        return {
            {0.115, 0.056, 0.034}, {0.006, 0.004, 0.004}, {0.34, 0.24, 0.18}, {0.80, 0.55, 0.38}};
    }
    return {{0.030, 0.045, 0.068}, {0.004, 0.005, 0.008}, {0.26, 0.32, 0.40}, {0.36, 0.56, 1.0}};
}

ShaderSource source(const entry_shaders::EmbeddedShader& s) {
    return {s.name, s.source};
}
ShaderSource source(const shaders::EmbeddedShader& s) {
    return {s.name, s.source};
}

core::Result<Program, std::string> build(const ShaderSource& vertex, const ShaderSource& fragment,
                                         bool log_depth) {
    std::vector<ShaderSource> includes;
    if (log_depth) {
        includes.push_back(source(shaders::log_depth));
    }
    const std::array<StageSource, 2> stages{StageSource{ShaderStage::vertex, vertex, {}},
                                            StageSource{ShaderStage::fragment, fragment, includes}};
    return Program::create(stages);
}

float to_float(double value) noexcept {
    return static_cast<float>(value);
}

glm::vec3 to_vec3(const glm::dvec3& v) noexcept {
    return {to_float(v.x), to_float(v.y), to_float(v.z)};
}

bool finite(const EntryTrackPoint& p) noexcept {
    return std::isfinite(p.latitude_deg) && std::isfinite(p.longitude_deg) &&
           std::isfinite(p.altitude_m) && std::isfinite(p.value);
}

core::Result<Texture, std::string> make_lut(double exposure) {
    auto texture = Texture::create_2d(kLutSize, 1, TextureFormat::rgba32f);
    if (!texture) {
        return core::failure("colour map texture: " + texture.error());
    }
    std::vector<float> texels;
    texels.reserve(static_cast<std::size_t>(kLutSize) * 4U);
    for (const glm::dvec3& c : colormap_table(Colormap::sequential, kLutSize)) {
        const glm::dvec3 scene = scene_linear_for_display(c, exposure);
        texels.insert(texels.end(),
                      {to_float(scene.x), to_float(scene.y), to_float(scene.z), 1.0F});
    }
    glTextureSubImage2D(texture.value().id(), 0, 0, 0, kLutSize, 1, GL_RGBA, GL_FLOAT,
                        texels.data());
    return std::move(texture).value();
}

// A vertex buffer read as consecutive pairs (segment i = points i, i + 1).
struct SegmentsGpu {
    Buffer points;
    VertexArray vao;
    GLsizei segments = 0;
};

core::Result<std::optional<SegmentsGpu>, std::string>
upload_segments(const std::vector<glm::vec4>& points) {
    if (points.size() < 2) {
        return std::optional<SegmentsGpu>{};
    }
    auto buffer = Buffer::from(std::span<const glm::vec4>(points));
    if (!buffer) {
        return core::failure("track points: " + buffer.error());
    }
    VertexArray vao = VertexArray::create();
    constexpr std::size_t kStride = sizeof(glm::vec4);
    vao.set_vertex_buffer(0, buffer.value(), 0, kStride);
    vao.set_vertex_buffer(1, buffer.value(), kStride, kStride);
    vao.set_float_attribute(0, 0, 4, 0);
    vao.set_float_attribute(1, 1, 4, 0);
    vao.set_binding_divisor(0, 1);
    vao.set_binding_divisor(1, 1);
    return std::optional<SegmentsGpu>{SegmentsGpu{std::move(buffer).value(), std::move(vao),
                                                  static_cast<GLsizei>(points.size() - 1)}};
}

struct StripGpu {
    Buffer vertices;
    VertexArray vao;
    GLsizei count = 0;
};

struct DrawContext {
    const Camera* camera = nullptr;
    glm::mat4 view_projection{1.0F};
    glm::vec3 anchor{0.0F}; // track anchor in render space
    float depth_scale = 0.0F;
    glm::vec2 viewport{1.0F};
    float near_w = 0.0F;
    double exposure = 1.0;
    double px = 1.0; // pixels per reference pixel (frame height / 1080)
    glm::vec3 sun{0.0F, 0.0F, 1.0F};

    [[nodiscard]] glm::vec3 display(const glm::dvec3& color) const {
        return to_vec3(scene_linear_for_display(color, exposure));
    }
};

} // namespace

HdrSettings entry_hdr_settings() {
    HdrSettings settings;
    settings.exposure = 1.0;
    settings.samples = 4;
    settings.clear_color = glm::dvec3(0.0025, 0.0030, 0.0050);
    return settings;
}

struct EntryView::State {
    State(Program planet_program, Program atmosphere_program, Program line_program,
          Program curtain_program, Program marker_program, Buffer planet_vertices,
          Buffer planet_indices, VertexArray planet_array, GLsizei planet_count)
        : planet(std::move(planet_program)), atmosphere(std::move(atmosphere_program)),
          line(std::move(line_program)), curtain(std::move(curtain_program)),
          marker(std::move(marker_program)), planet_vbo(std::move(planet_vertices)),
          planet_ibo(std::move(planet_indices)), planet_vao(std::move(planet_array)),
          planet_index_count(planet_count) {}

    Program planet;
    Program atmosphere;
    Program line;
    Program curtain;
    Program marker;
    Buffer planet_vbo;
    Buffer planet_ibo;
    VertexArray planet_vao;
    GLsizei planet_index_count = 0;
    VertexArray empty_vao = VertexArray::create();
    std::optional<Texture> lut;
    double lut_exposure = -1.0;

    std::vector<EntryTrackPoint> track;
    EntryGlobe globe;
    EntryFrame frame;
    glm::dvec3 anchor{0.0}; // view frame [m]
    std::optional<SegmentsGpu> ribbon;
    std::optional<SegmentsGpu> ground;
    std::optional<StripGpu> curtain_strip;

    [[nodiscard]] glm::vec3 relative(const EntryTrackPoint& p, double altitude) const {
        const glm::dvec3 v = view_position(frame, globe, p.latitude_deg, p.longitude_deg, altitude);
        return to_vec3(v - anchor);
    }

    core::Result<std::monostate, std::string> upload() {
        ribbon.reset();
        ground.reset();
        curtain_strip.reset();
        std::vector<glm::vec4> air;
        std::vector<glm::vec4> surface;
        std::vector<glm::vec4> strip;
        air.reserve(track.size());
        surface.reserve(track.size());
        strip.reserve(track.size() * 2U);
        for (const EntryTrackPoint& p : track) {
            const glm::vec3 a = relative(p, p.altitude_m);
            const glm::vec3 g = relative(p, 0.0);
            air.emplace_back(a, to_float(p.value));
            surface.emplace_back(g, 0.0F);
            strip.emplace_back(g, 0.0F);
            strip.emplace_back(a, 1.0F);
        }
        auto made_air = upload_segments(air);
        if (!made_air) {
            return core::failure(std::string(made_air.error()));
        }
        auto made_ground = upload_segments(surface);
        if (!made_ground) {
            return core::failure(std::string(made_ground.error()));
        }
        ribbon = std::move(made_air).value();
        ground = std::move(made_ground).value();
        if (strip.size() >= 4) {
            auto buffer = Buffer::from(std::span<const glm::vec4>(strip));
            if (!buffer) {
                return core::failure("curtain: " + buffer.error());
            }
            VertexArray vao = VertexArray::create();
            vao.set_vertex_buffer(0, buffer.value(), 0, sizeof(glm::vec4));
            vao.set_float_attribute(0, 0, 4, 0);
            curtain_strip = StripGpu{std::move(buffer).value(), std::move(vao),
                                     static_cast<GLsizei>(strip.size())};
        }
        return std::monostate{};
    }

    core::Result<std::monostate, std::string> ensure_lut(double exposure) {
        if (lut && lut_exposure == exposure) {
            return std::monostate{};
        }
        auto made = make_lut(exposure);
        if (!made) {
            return core::failure(std::string(made.error()));
        }
        lut = std::move(made).value();
        lut_exposure = exposure;
        return std::monostate{};
    }

    void draw_planet(const DrawContext& c, const EntryViewOptions& options) const {
        const PlanetLook l = look(options.style);
        const glm::dmat4 rotation(frame.to_view);
        const glm::dmat4 model = glm::translate(glm::dmat4(1.0), c.camera->to_render_double({})) *
                                 rotation * glm::scale(glm::dmat4(1.0), glm::dvec3(globe.radius_m));
        planet.set("u_model", glm::mat4(model));
        planet.set("u_rotation", glm::mat3(glm::dmat3(frame.to_view)));
        planet.set("u_view_projection", c.view_projection);
        planet.set("u_log_depth_scale", c.depth_scale);
        planet.set("u_sun", c.sun);
        planet.set("u_day", to_vec3(l.day));
        planet.set("u_night", to_vec3(l.night));
        planet.set("u_grid_color", to_vec3(l.grid));
        planet.set("u_grid_step", options.graticule ? to_float(kGridStepDeg) : 0.0F);
        planet.set("u_grid_strength", 0.55F);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        planet.use();
        planet_vao.bind();
        glDrawElements(GL_TRIANGLES, planet_index_count, GL_UNSIGNED_INT, nullptr);
    }

    void draw_atmosphere(const DrawContext& c, const FrameInfo& f,
                         const EntryViewOptions& options) const {
        const Camera& camera = *c.camera;
        const double tan_half = std::tan(0.5 * camera.lens().vertical_fov);
        const double aspect = Camera::usable_aspect(f.aspect());
        const glm::dquat& q = camera.orientation();
        atmosphere.set("u_right", to_vec3(q * glm::dvec3(tan_half * aspect, 0.0, 0.0)));
        atmosphere.set("u_up", to_vec3(q * glm::dvec3(0.0, tan_half, 0.0)));
        atmosphere.set("u_forward", to_vec3(q * glm::dvec3(0.0, 0.0, -1.0)));
        atmosphere.set("u_center", to_vec3(camera.to_render_double({}) / globe.radius_m));
        atmosphere.set("u_shell", to_float(1.0 + globe.atmosphere_top_m * globe.altitude_scale /
                                                     globe.radius_m));
        atmosphere.set("u_sun", c.sun);
        atmosphere.set("u_color", to_vec3(look(options.style).haze));
        atmosphere.set("u_strength", to_float(kHazeStrength));
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        atmosphere.use();
        empty_vao.bind();
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glEnable(GL_DEPTH_TEST);
    }

    void draw_curtain(const DrawContext& c) const {
        if (!curtain_strip) {
            return;
        }
        curtain.set("u_view_projection", c.view_projection);
        curtain.set("u_offset", c.anchor);
        curtain.set("u_log_depth_scale", c.depth_scale);
        curtain.set("u_color", c.display(kCurtain));
        curtain.set("u_opacity", to_float(kCurtainOpacity));
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        curtain.use();
        curtain_strip->vao.bind();
        glDrawArrays(GL_TRIANGLE_STRIP, 0, curtain_strip->count);
    }

    // One pass of segments: mapped colours (lut) or `color`.
    void draw_segments(const DrawContext& c, const SegmentsGpu& segments, double half_width,
                       const std::optional<glm::vec3>& color, const ScalarRange& range,
                       float opacity) const {
        line.set("u_view_projection", c.view_projection);
        line.set("u_offset", c.anchor);
        line.set("u_viewport", c.viewport);
        line.set("u_half_width", to_float(half_width));
        line.set("u_near", c.near_w);
        line.set("u_range", glm::vec2(to_float(range.min), to_float(range.max)));
        line.set("u_map_start", to_float(kEntryMapStart));
        line.set("u_use_colormap", color ? 0 : 1);
        line.set("u_color", color.value_or(glm::vec3(1.0F)));
        line.set("u_opacity", opacity);
        line.set("u_depth_bias", to_float(kLineDepthBias));
        line.set("u_log_depth_scale", c.depth_scale);
        if (lut) {
            lut->bind(0);
        }
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        line.use();
        segments.vao.bind();
        glDrawArraysInstanced(GL_TRIANGLES, 0, 6, segments.segments);
    }

    // Markers with a non-finite place or radius are skipped; a failed upload is
    // reported.
    core::Result<std::size_t, std::string> draw_markers(const DrawContext& c,
                                                        const EntryViewOptions& options) const {
        if (options.markers.empty()) {
            return std::size_t{0};
        }
        std::vector<glm::vec4> centers;
        std::vector<glm::vec3> colors;
        for (const EntryMarker& m : options.markers) {
            if (!std::isfinite(m.latitude_deg) || !std::isfinite(m.longitude_deg) ||
                !std::isfinite(m.altitude_m) || !(m.radius_px > 0.0) ||
                !std::isfinite(m.radius_px)) {
                continue;
            }
            const glm::dvec3 v =
                view_position(frame, globe, m.latitude_deg, m.longitude_deg, m.altitude_m);
            centers.emplace_back(to_vec3(v - anchor), to_float(m.radius_px * c.px));
            colors.push_back(c.display(m.color));
        }
        if (centers.empty()) {
            return std::size_t{0};
        }
        auto center_buffer = Buffer::from(std::span<const glm::vec4>(centers));
        auto color_buffer = Buffer::from(std::span<const glm::vec3>(colors));
        if (!center_buffer) {
            return core::failure("markers: " + center_buffer.error());
        }
        if (!color_buffer) {
            return core::failure("markers: " + color_buffer.error());
        }
        const VertexArray vao = VertexArray::create();
        vao.set_vertex_buffer(0, center_buffer.value(), 0, sizeof(glm::vec4));
        vao.set_vertex_buffer(1, color_buffer.value(), 0, sizeof(glm::vec3));
        vao.set_float_attribute(0, 0, 4, 0);
        vao.set_float_attribute(1, 1, 3, 0);
        vao.set_binding_divisor(0, 1);
        vao.set_binding_divisor(1, 1);
        marker.set("u_view_projection", c.view_projection);
        marker.set("u_offset", c.anchor);
        marker.set("u_viewport", c.viewport);
        marker.set("u_near", c.near_w);
        marker.set("u_ring", c.display(kOutline));
        marker.set("u_depth_bias", to_float(2.0 * kLineDepthBias));
        marker.set("u_log_depth_scale", c.depth_scale);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        marker.use();
        vao.bind();
        glDrawArraysInstanced(GL_TRIANGLES, 0, 6, static_cast<GLsizei>(centers.size()));
        return centers.size();
    }
};

EntryView::EntryView(std::unique_ptr<State> state) noexcept : state_(std::move(state)) {}
EntryView::EntryView(EntryView&& other) noexcept = default;
EntryView& EntryView::operator=(EntryView&& other) noexcept = default;
EntryView::~EntryView() = default;

core::Result<EntryView, std::string> EntryView::create() {
    using namespace entry_shaders;
    auto planet = build(source(entry_planet_vert), source(entry_planet_frag), true);
    auto atmosphere = build(source(shaders::fullscreen_vert), source(entry_atmosphere_frag), false);
    auto line = build(source(entry_line_vert), source(entry_line_frag), true);
    auto curtain = build(source(entry_curtain_vert), source(entry_curtain_frag), true);
    auto marker = build(source(entry_marker_vert), source(entry_marker_frag), true);
    for (const auto* program : {&planet, &atmosphere, &line, &curtain, &marker}) {
        if (!*program) {
            return core::failure("entry view: " + program->error());
        }
    }
    const IcosphereMesh sphere = make_icosphere(kPlanetSubdivisions);
    auto vbo = Buffer::from(std::span<const glm::vec3>(sphere.positions));
    if (!vbo) {
        return core::failure("entry view: planet vertices: " + vbo.error());
    }
    auto ibo = Buffer::from(std::span<const std::uint32_t>(sphere.indices));
    if (!ibo) {
        return core::failure("entry view: planet indices: " + ibo.error());
    }
    VertexArray vao = VertexArray::create();
    vao.set_vertex_buffer(0, vbo.value(), 0, sizeof(glm::vec3));
    vao.set_float_attribute(0, 0, 3, 0);
    vao.set_index_buffer(ibo.value());
    auto state = std::make_unique<State>(
        std::move(planet).value(), std::move(atmosphere).value(), std::move(line).value(),
        std::move(curtain).value(), std::move(marker).value(), std::move(vbo).value(),
        std::move(ibo).value(), std::move(vao), static_cast<GLsizei>(sphere.indices.size()));
    state->frame = make_entry_frame({}, state->globe);
    return EntryView(std::move(state));
}

core::Result<std::monostate, std::string> EntryView::set_track(std::vector<EntryTrackPoint> track,
                                                               const EntryGlobe& globe) {
    State& s = *state_;
    if (!(globe.radius_m > 0.0) || !std::isfinite(globe.radius_m) ||
        !(globe.atmosphere_top_m > 0.0) || !std::isfinite(globe.atmosphere_top_m) ||
        !(globe.altitude_scale >= 1.0) || !std::isfinite(globe.altitude_scale)) {
        return core::failure(std::string("entry view: the globe needs a positive radius and "
                                         "atmosphere and an altitude scale >= 1"));
    }
    for (std::size_t i = 0; i < track.size(); ++i) {
        if (!finite(track[i])) {
            return core::failure("entry view: track point " + std::to_string(i) + " is not finite");
        }
    }
    s.track = std::move(track);
    s.globe = globe;
    s.frame = make_entry_frame(s.track, s.globe);
    const geom::Bounds& b = s.frame.bounds;
    s.anchor = 0.5 * glm::dvec3(b.min.x + b.max.x, b.min.y + b.max.y, b.min.z + b.max.z);
    if (auto uploaded = s.upload(); !uploaded) {
        s.track.clear();
        s.ribbon.reset();
        s.ground.reset();
        s.curtain_strip.reset();
        return core::failure("entry view: " + uploaded.error());
    }
    return std::monostate{};
}

const EntryFrame& EntryView::frame() const noexcept {
    return state_->frame;
}

const EntryGlobe& EntryView::globe() const noexcept {
    return state_->globe;
}

std::size_t EntryView::track_size() const noexcept {
    return state_->track.size();
}

core::Result<EntryDrawStats, std::string>
EntryView::draw(const Camera& camera, const FrameInfo& frame, const EntryViewOptions& options) {
    State& s = *state_;
    if (auto ready = s.ensure_lut(options.exposure); !ready) {
        return core::failure("entry view: " + ready.error());
    }
    DrawContext c;
    c.camera = &camera;
    c.view_projection = camera.view_projection_matrix(frame.aspect());
    c.anchor = to_vec3(camera.to_render_double(s.anchor));
    c.depth_scale = to_float(log_depth_scale(camera.lens().log_depth_far));
    c.viewport = {static_cast<float>(frame.width), static_cast<float>(frame.height)};
    c.near_w = to_float(camera.lens().near_plane);
    c.exposure = options.exposure;
    c.px = std::clamp(static_cast<double>(frame.height) / 1080.0, 0.5, 4.0);
    const double sun_length = glm::length(options.sun);
    c.sun = to_vec3(sun_length > 0.0 && std::isfinite(sun_length) ? options.sun / sun_length
                                                                  : glm::dvec3(0.0, 0.0, 1.0));

    EntryDrawStats stats;
    glDisable(GL_CULL_FACE);
    s.draw_planet(c, options);
    stats.planet_triangles = static_cast<std::size_t>(s.planet_index_count) / 3U;
    if (options.atmosphere) {
        s.draw_atmosphere(c, frame, options);
    }
    if (options.curtain) {
        s.draw_curtain(c);
    }
    if (options.ground_track && s.ground) {
        s.draw_segments(c, *s.ground, kOutlinePx * 0.5 * c.px + kGroundHalfWidthPx * c.px,
                        c.display(kOutline), options.range, 0.85F);
        s.draw_segments(c, *s.ground, kGroundHalfWidthPx * c.px, c.display(kGroundTrack),
                        options.range, 0.95F);
        stats.ground_segments = static_cast<std::size_t>(s.ground->segments);
    }
    if (options.ribbon && s.ribbon) {
        const double half = 0.5 * options.ribbon_width_px * c.px;
        s.draw_segments(c, *s.ribbon, half + kOutlinePx * c.px, c.display(kOutline), options.range,
                        0.9F);
        s.draw_segments(c, *s.ribbon, half, std::nullopt, options.range, 1.0F);
        stats.ribbon_segments = static_cast<std::size_t>(s.ribbon->segments);
    }
    auto markers = s.draw_markers(c, options);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glBindVertexArray(0);
    if (!markers) {
        return core::failure("entry view: " + markers.error());
    }
    stats.markers = markers.value();
    return stats;
}

} // namespace orrery::render
