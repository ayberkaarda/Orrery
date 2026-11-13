#include "orrery/render/aero_view.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <map>
#include <span>
#include <sstream>
#include <string>
#include <utility>

#include <glm/ext/matrix_double4x4.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>

#include "../gl_loader.h"
#include "aero_internal.h"
#include "aero_overlay.h"
#include "orrery/render/aero_compute.h"
#include "orrery/render/aero_isosurface.h"
#include "orrery/render/embedded_aero_shaders.h"
#include "orrery/render/embedded_shaders.h"
#include "orrery/render/log_depth.h"
#include "orrery/render/ui_theme.h"

namespace orrery::render {

namespace {

constexpr int kLutSize = 256;
constexpr double kCreaseCos = 0.7071067811865476; // 45 degrees
// Lighting of the colour-mapped surfaces (aero_surface.frag.glsl).
constexpr double kAmbient = 0.42;
constexpr double kSpecular = 0.35;
constexpr double kRim = 0.05;
constexpr double kContourStrength = 0.16;
// Key light in camera axes (x right, y up, z towards the viewer).
constexpr glm::dvec3 kKeyLightView{-0.42, 0.55, 0.72};

ShaderSource source(const aero_shaders::EmbeddedShader& s) {
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

void set_uint(const Program& program, std::string_view name, std::uint32_t value) {
    glProgramUniform1ui(program.id(), program.uniform_location(name), value);
}

glm::vec3 to_vec3(const glm::dvec3& v) {
    return {detail::to_float(v.x), detail::to_float(v.y), detail::to_float(v.z)};
}

glm::dvec3 to_glm(const geom::Vector3& v) {
    return {v.x, v.y, v.z};
}

core::Result<Texture, std::string> make_lut(Colormap map, double exposure) {
    auto texture = Texture::create_2d(kLutSize, 1, TextureFormat::rgba32f);
    if (!texture) {
        return core::failure("colour map texture: " + texture.error());
    }
    std::vector<float> texels;
    texels.reserve(static_cast<std::size_t>(kLutSize) * 4U);
    for (const glm::dvec3& c : colormap_table(map, kLutSize)) {
        const glm::dvec3 scene = scene_linear_for_display(c, exposure);
        texels.insert(texels.end(), {detail::to_float(scene.x), detail::to_float(scene.y),
                                     detail::to_float(scene.z), 1.0F});
    }
    glTextureSubImage2D(texture.value().id(), 0, 0, 0, kLutSize, 1, GL_RGBA, GL_FLOAT,
                        texels.data());
    return std::move(texture).value();
}

// One vertex of a drawn surface (aero_surface.vert.glsl).
struct SurfaceVertex {
    glm::vec3 position{0.0F};
    glm::vec3 normal{0.0F};
    glm::vec2 value{0.0F}; // value, weight
};
static_assert(sizeof(SurfaceVertex) == 32);

struct MeshGpu {
    Buffer vertices;
    std::optional<Buffer> indices;
    VertexArray vao;
    GLsizei count = 0;
};

core::Result<MeshGpu, std::string> upload_mesh(const std::vector<SurfaceVertex>& vertices,
                                               const std::vector<std::uint32_t>& indices) {
    auto vbo = Buffer::from(std::span<const SurfaceVertex>(vertices));
    if (!vbo) {
        return core::failure("mesh vertices: " + vbo.error());
    }
    VertexArray vao = VertexArray::create();
    vao.set_vertex_buffer(0, vbo.value(), 0, sizeof(SurfaceVertex));
    vao.set_float_attribute(0, 0, 3, 0);
    vao.set_float_attribute(1, 0, 3, 3 * sizeof(float));
    vao.set_float_attribute(2, 0, 2, 6 * sizeof(float));
    std::optional<Buffer> ibo;
    if (!indices.empty()) {
        auto made = Buffer::from(std::span<const std::uint32_t>(indices));
        if (!made) {
            return core::failure("mesh indices: " + made.error());
        }
        ibo = std::move(made).value();
        vao.set_index_buffer(*ibo);
    }
    const std::size_t count = indices.empty() ? vertices.size() : indices.size();
    return MeshGpu{std::move(vbo).value(), std::move(ibo), std::move(vao),
                   static_cast<GLsizei>(count)};
}

// Triangle soup of a body with crease-aware smooth normals: a corner takes
// the angle-weighted normals of the triangles around its vertex that are
// within 45 degrees of its own triangle.
std::vector<SurfaceVertex> surface_vertices(const AeroSurface& surface, bool smooth) {
    const geom::TriangleMesh& mesh = surface.mesh;
    const std::size_t n = mesh.triangles.size();
    std::vector<glm::dvec3> face(n);
    std::vector<std::array<double, 3>> angle(n);
    std::vector<std::vector<std::uint32_t>> around(mesh.vertices.size());
    for (std::size_t t = 0; t < n; ++t) {
        const geom::Triangle& tri = mesh.triangles[t];
        const std::array<glm::dvec3, 3> p{to_glm(mesh.vertices[tri[0]]),
                                          to_glm(mesh.vertices[tri[1]]),
                                          to_glm(mesh.vertices[tri[2]])};
        const glm::dvec3 cross = glm::cross(p[1] - p[0], p[2] - p[0]);
        const double length = glm::length(cross);
        face[t] = length > 0.0 ? cross / length : glm::dvec3(0.0, 0.0, 1.0);
        for (std::size_t c = 0; c < 3; ++c) {
            const glm::dvec3 e1 = p[(c + 1) % 3] - p[c];
            const glm::dvec3 e2 = p[(c + 2) % 3] - p[c];
            angle[t][c] = std::atan2(glm::length(glm::cross(e1, e2)), glm::dot(e1, e2));
            around[tri[c]].push_back(static_cast<std::uint32_t>(t));
        }
    }
    const bool has_values = surface.cp.size() == n;
    std::vector<double> smooth_values;
    if (has_values && smooth) {
        smooth_values = vertex_values(mesh, surface.cp);
    }
    std::vector<SurfaceVertex> out;
    out.reserve(3 * n);
    for (std::size_t t = 0; t < n; ++t) {
        const geom::Triangle& tri = mesh.triangles[t];
        const bool mapped = has_values && std::isfinite(surface.cp[t]);
        for (std::size_t c = 0; c < 3; ++c) {
            const std::uint32_t v = tri[c];
            glm::dvec3 normal = face[t];
            if (smooth) {
                glm::dvec3 sum(0.0);
                for (const std::uint32_t other : around[v]) {
                    if (glm::dot(face[other], face[t]) < kCreaseCos) {
                        continue;
                    }
                    const geom::Triangle& o = mesh.triangles[other];
                    const std::size_t corner = o[0] == v ? 0 : (o[1] == v ? 1 : 2);
                    sum += face[other] * angle[other][corner];
                }
                if (glm::length(sum) > 0.0) {
                    normal = glm::normalize(sum);
                }
            }
            double value = 0.0;
            if (mapped) {
                value =
                    smooth && std::isfinite(smooth_values[v]) ? smooth_values[v] : surface.cp[t];
            }
            out.push_back({to_vec3(to_glm(mesh.vertices[v])), to_vec3(normal),
                           glm::vec2(detail::to_float(value), mapped ? 1.0F : 0.0F)});
        }
    }
    return out;
}

// An arrow along `direction` from `origin`: shaft cylinder, cone head and
// base disc, lit like the body.
void append_arrow(std::vector<SurfaceVertex>& out, const glm::dvec3& origin,
                  const glm::dvec3& direction, double length, double radius) {
    constexpr int kSides = 24;
    constexpr double kTwoPi = 6.283185307179586;
    const glm::dvec3 axis = glm::normalize(direction);
    const glm::dvec3 helper =
        std::abs(axis.z) < 0.9 ? glm::dvec3(0.0, 0.0, 1.0) : glm::dvec3(1.0, 0.0, 0.0);
    const glm::dvec3 u = glm::normalize(glm::cross(helper, axis));
    const glm::dvec3 w = glm::cross(axis, u);
    const double head = std::min(length * 0.35, radius * 7.0);
    const double head_radius = radius * 2.6;
    const double shaft = length - head;
    const auto ring = [&](int i) {
        const double a = kTwoPi * i / kSides;
        return u * std::cos(a) + w * std::sin(a);
    };
    const auto push = [&](const glm::dvec3& p, const glm::dvec3& n) {
        out.push_back({to_vec3(p), to_vec3(n), glm::vec2(0.0F)});
    };
    const glm::dvec3 tip = origin + axis * length;
    const glm::dvec3 neck = origin + axis * shaft;
    const double slope = head_radius / head;
    for (int i = 0; i < kSides; ++i) {
        const glm::dvec3 r0 = ring(i);
        const glm::dvec3 r1 = ring(i + 1);
        // Shaft.
        push(origin + r0 * radius, r0);
        push(origin + r1 * radius, r1);
        push(neck + r1 * radius, r1);
        push(origin + r0 * radius, r0);
        push(neck + r1 * radius, r1);
        push(neck + r0 * radius, r0);
        // Head underside.
        push(neck, -axis);
        push(neck + r1 * head_radius, -axis);
        push(neck + r0 * head_radius, -axis);
        // Cone.
        const glm::dvec3 n0 = glm::normalize(r0 + axis * slope);
        const glm::dvec3 n1 = glm::normalize(r1 + axis * slope);
        push(neck + r0 * head_radius, n0);
        push(neck + r1 * head_radius, n1);
        push(tip, glm::normalize(n0 + n1));
    }
}

struct SectionGpu {
    SectionPlane plane;
    Texture values;
    SectionImage image; // geometry only (values released)
    ScalarRange range;
    FieldQuantity quantity = FieldQuantity::speed;
};

struct StreamlinesGpu {
    std::vector<glm::dvec3> seeds;
    StreamlineSettings settings;
    std::optional<Buffer> segments;
    VertexArray vao;
    GLsizei count = 0;
    std::size_t lines = 0;
    std::size_t points = 0;
};

struct ParticlesGpu {
    std::size_t count = 0;
    std::size_t trace_points = 0;
    std::vector<glm::dvec3> seeds;
    std::uint32_t random_seed = 0;
    aero::compute::ScopedBuffer seed_buffer;
    aero::compute::ScopedBuffer state;
    aero::compute::ScopedBuffer traces;
    std::uint32_t head = 0;
    std::uint32_t frame = 0;
};

struct IsoGpu {
    double level = 0.0;
    bool absolute = false;
    std::optional<MeshGpu> mesh;
    std::size_t triangles = 0;
};

bool same_settings(const StreamlineSettings& a, const StreamlineSettings& b) {
    return a.step == b.step && a.max_points == b.max_points && a.direction == b.direction &&
           a.stagnation_fraction == b.stagnation_fraction;
}

bool same_plane(const SectionPlane& a, const SectionPlane& b) {
    return a.axis == b.axis && a.position == b.position && a.quantity == b.quantity &&
           a.scale.mode == b.scale.mode && a.scale.min == b.scale.min &&
           a.scale.max == b.scale.max && a.scale.round_to_ticks == b.scale.round_to_ticks &&
           a.colormap == b.colormap;
}

ColorScale effective_scale(const SectionPlane& plane) {
    ColorScale scale = plane.scale;
    if (is_signed(plane.quantity) && scale.mode != ScaleMode::fixed) {
        scale.mode = ScaleMode::symmetric;
    }
    return scale;
}

Colormap colormap_of(const SectionPlane& plane) {
    if (plane.colormap) {
        return *plane.colormap;
    }
    return is_signed(plane.quantity) ? Colormap::diverging_dark : Colormap::sequential;
}

// Legend step for contour lines on a value range.
glm::vec2 contour_spacing(const ScalarRange& range) {
    const std::vector<LegendTick> ticks = legend_ticks(range, 5);
    if (ticks.size() < 2) {
        return {0.0F, 0.0F};
    }
    const double step = ticks[1].value - ticks[0].value;
    return {detail::to_float(ticks[0].value), detail::to_float(step * 0.5)};
}

std::string format_coefficient(const char* name, double value, int decimals) {
    std::array<char, 64> text{};
    (void)std::snprintf(text.data(), text.size(), "%s %.*f", name, decimals, value);
    return text.data();
}

UiColor lift_ui_color() {
    return default_theme().accent;
}
UiColor drag_ui_color() {
    return default_theme().accent_secondary;
}

glm::dvec3 display_linear(const UiColor& c) {
    const auto decode = [](float v) {
        const double x = std::clamp(static_cast<double>(v), 0.0, 1.0);
        return x <= 0.04045 ? x / 12.92 : std::pow((x + 0.055) / 1.055, 2.4);
    };
    return {decode(c.r), decode(c.g), decode(c.b)};
}

} // namespace

struct AeroView::State {
    std::unique_ptr<GlComputeBackend> compute;
    Program surface_program;
    Program section_program;
    Program floor_program;
    Program line_program;
    Program particle_program;
    Program overlay_program;
    Program backdrop_program;
    VertexArray empty_vao;
    std::optional<Texture> lut_diverging;
    std::optional<Texture> lut_diverging_dark;
    std::optional<Texture> lut_sequential;
    double lut_exposure = -1.0;

    // Body.
    std::optional<AeroSurface> surface;
    std::optional<MeshGpu> surface_mesh;
    bool surface_smooth = true;
    std::optional<ScalarRange> cp_range_cache;
    ColorScale cp_scale_cache;

    // Flow.
    std::optional<FlowField> flow;
    std::uint64_t flow_version = 0;
    std::map<std::pair<FieldQuantity, Axis>, ScalarGrid> derived;
    std::map<std::pair<FieldQuantity, int>, ScalarRange> ranges;
    std::vector<SectionGpu> sections;
    std::optional<StreamlinesGpu> streamlines;
    std::uint64_t streamline_version = 0;
    std::optional<detail::FieldBuffers> field_buffers;
    std::uint64_t field_buffer_version = 0;
    std::optional<ParticlesGpu> particles;
    std::optional<IsoGpu> iso;
    std::uint64_t iso_version = 0;

    // Forces.
    std::optional<AeroForces> forces;
    std::optional<MeshGpu> lift_arrow;
    std::optional<MeshGpu> drag_arrow;
    std::optional<MeshGpu> side_arrow;
    std::string arrow_key;

    // Overlay.
    std::optional<Texture> overlay_texture;
    std::string overlay_key;
    glm::ivec2 overlay_size{0};
    glm::ivec2 overlay_position{0};

    const ScalarGrid& derived_grid(FieldQuantity quantity, Axis axis) {
        const Axis key_axis = flow->velocity.grid.is_2d() ? Axis::z : axis;
        const Axis normal = quantity == FieldQuantity::vorticity ? key_axis : Axis::z;
        const auto key = std::make_pair(quantity, normal);
        auto found = derived.find(key);
        if (found == derived.end()) {
            found = derived.emplace(key, derive(*flow, quantity, normal)).first;
        }
        return found->second;
    }

    // Range of a quantity over the fluid nodes of the whole field.
    ScalarRange field_range(FieldQuantity quantity, Axis axis, const ColorScale& scale) {
        const ScalarGrid& grid = derived_grid(quantity, axis);
        std::vector<double> fluid;
        fluid.reserve(grid.values.size());
        for (std::size_t n = 0; n < grid.values.size(); ++n) {
            if (flow->solid.empty() || flow->solid[n] == 0) {
                fluid.push_back(grid.values[n]);
            }
        }
        return resolve_range(scale, fluid);
    }

    ScalarRange cp_range(const ColorScale& scale) {
        if (!cp_range_cache || cp_scale_cache.mode != scale.mode ||
            cp_scale_cache.min != scale.min || cp_scale_cache.max != scale.max ||
            cp_scale_cache.round_to_ticks != scale.round_to_ticks) {
            cp_range_cache = resolve_range(scale, surface ? surface->cp : std::vector<double>{});
            cp_scale_cache = scale;
        }
        return *cp_range_cache;
    }

    bool surface_has_cp() const {
        return surface && !surface->cp.empty() &&
               std::ranges::any_of(surface->cp, [](double v) { return std::isfinite(v); });
    }

    std::vector<glm::dvec3> seeds_for(const StreamlineLayer& layer) const {
        if (!layer.seeds.empty()) {
            return layer.seeds;
        }
        const GridGeometry& g = flow->velocity.grid;
        const double inset = 0.5 * g.spacing;
        if (g.is_2d()) {
            const glm::dvec3 hi = g.upper_corner();
            return seed_line({g.origin.x + inset, g.origin.y + inset, g.origin.z},
                             {g.origin.x + inset, hi.y - inset, g.origin.z}, 24);
        }
        return seed_plane(g, Axis::x, g.origin.x + inset, 12, 6);
    }

    core::Result<std::monostate, std::string> ensure_field_buffers() {
        if (field_buffers && field_buffer_version == flow_version) {
            return std::monostate{};
        }
        auto uploaded = detail::upload_field(*compute, flow->velocity, flow->solid);
        if (!uploaded) {
            return core::failure("aero view: " + uploaded.error());
        }
        field_buffers = std::move(uploaded).value();
        field_buffer_version = flow_version;
        return std::monostate{};
    }
};

HdrSettings aero_hdr_settings() {
    HdrSettings settings;
    settings.exposure = 1.0;
    settings.samples = 4;
    settings.clear_color = AeroViewOptions{}.backdrop_bottom;
    return settings;
}

TurntableMode frame_bounds(const geom::Bounds& bounds, double aspect, const CameraLens& lens,
                           double yaw, double pitch) {
    const glm::dvec3 lo = to_glm(bounds.min);
    const glm::dvec3 hi = to_glm(bounds.max);
    const double radius = std::max(0.5 * glm::length(hi - lo), 1.0e-6);
    const double vertical = lens.vertical_fov;
    const double horizontal =
        2.0 * std::atan(std::tan(0.5 * vertical) * Camera::usable_aspect(aspect));
    const double fov = std::min(vertical, horizontal);
    TurntableMode mode;
    mode.focus = 0.5 * (lo + hi);
    mode.distance = 1.04 * radius / std::sin(0.5 * fov);
    mode.yaw = yaw;
    mode.pitch = pitch;
    return mode;
}

AeroView::AeroView(std::unique_ptr<State> state) noexcept : state_(std::move(state)) {}
AeroView::AeroView(AeroView&& other) noexcept = default;
AeroView& AeroView::operator=(AeroView&& other) noexcept = default;
AeroView::~AeroView() = default;

core::Result<AeroView, std::string> AeroView::create() {
    auto compute = make_aero_compute_backend();
    if (!compute) {
        return core::failure("aero view: " + compute.error());
    }
    using namespace aero_shaders;
    auto surface = build(source(aero_surface_vert), source(aero_surface_frag), true);
    auto section = build(source(aero_plane_vert), source(aero_section_frag), true);
    auto floor = build(source(aero_plane_vert), source(aero_floor_frag), true);
    auto line = build(source(aero_line_vert), source(aero_line_frag), true);
    auto particle = build(source(aero_particle_vert), source(aero_line_frag), true);
    auto overlay = build(source(aero_overlay_vert), source(aero_overlay_frag), false);
    auto backdrop =
        build(source(shaders::fullscreen_vert), source(shaders::background_frag), false);
    for (const auto* program :
         {&surface, &section, &floor, &line, &particle, &overlay, &backdrop}) {
        if (!*program) {
            return core::failure("aero view: " + program->error());
        }
    }
    auto state = std::make_unique<State>(
        State{std::move(compute).value(), std::move(surface).value(), std::move(section).value(),
              std::move(floor).value(), std::move(line).value(), std::move(particle).value(),
              std::move(overlay).value(), std::move(backdrop).value(), VertexArray::create()});
    return AeroView(std::move(state));
}

core::Result<std::monostate, std::string> AeroView::set_surface(const AeroSurface& surface) {
    State& s = *state_;
    if (!surface.cp.empty() && surface.cp.size() != surface.mesh.triangles.size()) {
        return core::failure(std::string("aero view: Cp count differs from the triangles"));
    }
    for (const geom::Triangle& t : surface.mesh.triangles) {
        if (t[0] >= surface.mesh.vertices.size() || t[1] >= surface.mesh.vertices.size() ||
            t[2] >= surface.mesh.vertices.size()) {
            return core::failure(std::string("aero view: triangle index outside the mesh"));
        }
    }
    s.surface_mesh.reset();
    s.cp_range_cache.reset();
    if (surface.mesh.triangles.empty()) {
        s.surface.reset();
        return std::monostate{};
    }
    s.surface = surface;
    return std::monostate{};
}

core::Result<std::monostate, std::string> AeroView::set_flow(FlowField field) {
    if (auto bad = validate(field)) {
        return core::failure("aero view: " + *bad);
    }
    State& s = *state_;
    const bool same_grid = s.flow && s.flow->velocity.grid.dims == field.velocity.grid.dims &&
                           s.flow->velocity.grid.origin == field.velocity.grid.origin &&
                           s.flow->velocity.grid.spacing == field.velocity.grid.spacing;
    s.flow = std::move(field);
    ++s.flow_version;
    s.derived.clear();
    s.ranges.clear();
    s.sections.clear();
    s.streamlines.reset();
    s.iso.reset();
    if (!same_grid) {
        s.particles.reset();
    }
    return std::monostate{};
}

void AeroView::clear_flow() {
    State& s = *state_;
    s.flow.reset();
    ++s.flow_version;
    s.derived.clear();
    s.ranges.clear();
    s.sections.clear();
    s.streamlines.reset();
    s.iso.reset();
    s.particles.reset();
    s.field_buffers.reset();
}

void AeroView::set_forces(const std::optional<AeroForces>& forces) {
    state_->forces = forces;
    state_->arrow_key.clear();
}

bool AeroView::has_flow() const noexcept {
    return state_->flow.has_value();
}

void AeroView::reset_particles() {
    state_->particles.reset();
}

core::Result<std::monostate, std::string> AeroView::advance(double dt,
                                                            const AeroViewOptions& options) {
    using aero::compute::BufferBinding;
    using aero::compute::DispatchCommand;
    using aero::compute::Float32Uniform;
    using aero::compute::Uniform;
    State& s = *state_;
    const ParticleLayer& layer = options.particles;
    if (!s.flow || !layer.enabled || layer.count == 0 || !(dt > 0.0) || !std::isfinite(dt)) {
        return std::monostate{};
    }
    if (auto ready = s.ensure_field_buffers(); !ready) {
        return ready;
    }
    const GridGeometry& g = s.flow->velocity.grid;
    const std::size_t trace_points = std::clamp<std::size_t>(layer.trace_points, 2, 64);
    const std::vector<glm::dvec3> seeds = s.seeds_for(options.streamlines);
    if (seeds.empty()) {
        return std::monostate{};
    }
    if (!s.particles || s.particles->count != layer.count ||
        s.particles->trace_points != trace_points || s.particles->seeds != seeds ||
        s.particles->random_seed != layer.random_seed) {
        // Fresh particles spread over the fluid with random ages, so the view
        // is filled at once; they are released from the seeds afterwards.
        ParticlesGpu p;
        p.count = layer.count;
        p.trace_points = trace_points;
        p.seeds = seeds;
        p.random_seed = layer.random_seed;
        std::uint64_t state = 0x9e3779b97f4a7c15ULL ^ layer.random_seed;
        const auto random = [&state] {
            state = state * 6364136223846793005ULL + 1442695040888963407ULL;
            return static_cast<double>(state >> 11U) * (1.0 / 9007199254740992.0);
        };
        std::vector<glm::vec4> initial;
        initial.reserve(layer.count);
        const glm::dvec3 last = g.upper_corner() - g.origin;
        for (std::size_t i = 0; i < layer.count; ++i) {
            glm::dvec3 world{0.0};
            for (int attempt = 0; attempt < 16; ++attempt) {
                world = g.origin + glm::dvec3(random() * last.x, random() * last.y,
                                              g.is_2d() ? 0.0 : random() * last.z);
                if (advect_particle(s.flow->velocity, s.flow->solid, world, 0.0)) {
                    break;
                }
            }
            glm::vec4 index = detail::to_index_space(g, world);
            index.w = detail::to_float(random() * layer.lifetime);
            initial.push_back(index);
        }
        std::vector<glm::vec4> traces;
        traces.reserve(layer.count * trace_points);
        for (const glm::vec4& v : initial) {
            for (std::size_t k = 0; k < trace_points; ++k) {
                traces.emplace_back(v.x, v.y, v.z, k == 0 ? v.w : -1.0F);
            }
        }
        std::vector<glm::vec4> seed_points;
        seed_points.reserve(seeds.size());
        for (const glm::dvec3& seed : seeds) {
            seed_points.push_back(detail::to_index_space(g, seed));
        }
        auto seed_buffer =
            detail::upload_new(*s.compute, std::as_bytes(std::span<const glm::vec4>(seed_points)));
        auto state_buffer =
            detail::upload_new(*s.compute, std::as_bytes(std::span<const glm::vec4>(initial)));
        auto trace_buffer =
            detail::upload_new(*s.compute, std::as_bytes(std::span<const glm::vec4>(traces)));
        if (!seed_buffer || !state_buffer || !trace_buffer) {
            return core::failure(std::string("aero view: could not create the particle buffers"));
        }
        p.seed_buffer = std::move(seed_buffer).value();
        p.state = std::move(state_buffer).value();
        p.traces = std::move(trace_buffer).value();
        s.particles = std::move(p);
    }
    ParticlesGpu& p = *s.particles;
    const double time = dt * layer.time_scale;
    // Substeps keep each step below half a grid spacing.
    const double cells = time * static_cast<double>(s.field_buffers->max_speed) / g.spacing;
    const int substeps = std::clamp(static_cast<int>(std::ceil(cells / 0.5)), 1, 64);
    const double step = time / substeps;
    p.head = (p.head + 1) % static_cast<std::uint32_t>(p.trace_points);
    for (int i = 0; i < substeps; ++i) {
        ++p.frame;
        const std::array<Uniform, 13> uniforms{
            Uniform{"u_nx", static_cast<std::uint32_t>(g.dims[0])},
            Uniform{"u_ny", static_cast<std::uint32_t>(g.dims[1])},
            Uniform{"u_nz", static_cast<std::uint32_t>(g.dims[2])},
            Uniform{"u_count", static_cast<std::uint32_t>(p.count)},
            Uniform{"u_seed_count", static_cast<std::uint32_t>(p.seeds.size())},
            Uniform{"u_trace_points", static_cast<std::uint32_t>(p.trace_points)},
            Uniform{"u_head", p.head},
            Uniform{"u_release", std::uint32_t{1}},
            Uniform{"u_frame", p.frame * 0x632be5abU ^ p.random_seed},
            Uniform{"u_dt", Float32Uniform{step}},
            Uniform{"u_dt_index", Float32Uniform{step / g.spacing}},
            Uniform{"u_lifetime", Float32Uniform{layer.lifetime}},
            Uniform{"u_jitter", Float32Uniform{1.0}},
        };
        const std::array<BufferBinding, 5> buffers{
            BufferBinding{0, s.field_buffers->velocity.id()},
            BufferBinding{1, s.field_buffers->solid.id()},
            BufferBinding{2, p.seed_buffer.id()},
            BufferBinding{3, p.state.id()},
            BufferBinding{4, p.traces.id()},
        };
        const DispatchCommand command{
            kernel_particles, {static_cast<std::uint32_t>(p.count), 1, 1}, uniforms, buffers};
        if (auto done = s.compute->dispatch(command); !done) {
            return core::failure("aero view: " + done.error());
        }
        s.compute->barrier();
    }
    return std::monostate{};
}

std::vector<Legend> AeroView::legends(const AeroViewOptions& options) const {
    State& s = *state_;
    std::vector<Legend> result;
    if (options.surface.enabled && s.surface_has_cp()) {
        result.push_back(make_legend("Cp", Colormap::diverging, s.cp_range(options.surface.scale)));
    }
    if (!s.flow) {
        return result;
    }
    std::vector<FieldQuantity> shown;
    for (const SectionPlane& plane : options.sections) {
        if (!plane.enabled || std::ranges::find(shown, plane.quantity) != shown.end()) {
            continue;
        }
        if (plane.quantity == FieldQuantity::pressure && !s.flow->pressure) {
            continue;
        }
        shown.push_back(plane.quantity);
        result.push_back(
            make_legend(quantity_label(plane.quantity, s.flow->pressure_label), colormap_of(plane),
                        s.field_range(plane.quantity, plane.axis, effective_scale(plane))));
    }
    if (options.streamlines.enabled && options.streamlines.color_by_speed &&
        std::ranges::find(shown, FieldQuantity::speed) == shown.end()) {
        result.push_back(make_legend(quantity_label(FieldQuantity::speed), Colormap::sequential,
                                     s.field_range(FieldQuantity::speed, Axis::z, {})));
    }
    return result;
}

geom::Bounds AeroView::scene_bounds() const noexcept {
    const State& s = *state_;
    bool any = false;
    glm::dvec3 lo(std::numeric_limits<double>::infinity());
    glm::dvec3 hi(-std::numeric_limits<double>::infinity());
    if (s.surface) {
        const geom::Bounds b = geom::bounds(s.surface->mesh);
        lo = glm::min(lo, to_glm(b.min));
        hi = glm::max(hi, to_glm(b.max));
        any = true;
    }
    if (s.flow) {
        lo = glm::min(lo, s.flow->velocity.grid.origin);
        hi = glm::max(hi, s.flow->velocity.grid.upper_corner());
        any = true;
    }
    if (!any) {
        return {};
    }
    return {{lo.x, lo.y, lo.z}, {hi.x, hi.y, hi.z}};
}

core::Result<AeroDrawStats, std::string>
AeroView::draw(const Camera& camera, const FrameInfo& frame, const AeroViewOptions& options) {
    State& s = *state_;
    AeroDrawStats stats;
    if (s.lut_exposure != options.exposure || !s.lut_diverging) {
        auto diverging = make_lut(Colormap::diverging, options.exposure);
        auto dark = make_lut(Colormap::diverging_dark, options.exposure);
        auto sequential = make_lut(Colormap::sequential, options.exposure);
        for (const auto* made : {&diverging, &dark, &sequential}) {
            if (!*made) {
                return core::failure("aero view: " + made->error());
            }
        }
        s.lut_diverging = std::move(diverging).value();
        s.lut_diverging_dark = std::move(dark).value();
        s.lut_sequential = std::move(sequential).value();
        s.lut_exposure = options.exposure;
    }
    const auto lut = [&s](Colormap map) -> const Texture& {
        switch (map) {
            case Colormap::diverging:
                return *s.lut_diverging;
            case Colormap::diverging_dark:
                return *s.lut_diverging_dark;
            case Colormap::sequential:
                break;
        }
        return *s.lut_sequential;
    };
    const auto precompensate = [&](const glm::dvec3& display) {
        return to_vec3(scene_linear_for_display(display, options.exposure));
    };
    const glm::mat4 view_projection = camera.view_projection_matrix(frame.aspect());
    const glm::dvec3 offset = camera.to_render_double(glm::dvec3(0.0));
    const glm::mat4 model(glm::translate(glm::dmat4(1.0), offset));
    const auto depth_scale = detail::to_float(log_depth_scale(camera.lens().log_depth_far));
    const glm::vec3 key = to_vec3(glm::normalize(camera.orientation() * kKeyLightView));
    const glm::vec2 viewport(static_cast<float>(frame.width), static_cast<float>(frame.height));
    const auto near_w = detail::to_float(camera.lens().near_plane);

    glDisable(GL_CULL_FACE);
    // 1. Backdrop.
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glDisable(GL_BLEND);
    s.backdrop_program.set("u_horizon", to_vec3(options.backdrop_bottom));
    s.backdrop_program.set("u_zenith", to_vec3(options.backdrop_top));
    s.backdrop_program.use();
    s.empty_vao.bind();
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    const geom::Bounds bounds = scene_bounds();
    const bool flat_flow = s.flow && s.flow->velocity.grid.is_2d();

    // 2. Floor grid under a 3D scene.
    if ((s.surface || s.flow) && !flat_flow) {
        const glm::dvec3 lo = to_glm(bounds.min);
        const glm::dvec3 hi = to_glm(bounds.max);
        const double size = std::max(glm::length(hi - lo), 1.0e-3);
        const double raw_step = size / 12.0;
        const double magnitude = std::pow(10.0, std::floor(std::log10(raw_step)));
        const double step = raw_step / magnitude < 2.0   ? magnitude
                            : raw_step / magnitude < 5.0 ? 2.0 * magnitude
                                                         : 5.0 * magnitude;
        const glm::dvec3 centre = 0.5 * (lo + hi);
        const double half = 1.6 * size;
        const double z = lo.z - 0.12 * size;
        const glm::dvec3 corner(centre.x - half, centre.y - half, z);
        s.floor_program.set("u_corner", to_vec3(camera.to_render_double(corner)));
        s.floor_program.set("u_edge_u", glm::vec3(detail::to_float(2.0 * half), 0.0F, 0.0F));
        s.floor_program.set("u_edge_v", glm::vec3(0.0F, detail::to_float(2.0 * half), 0.0F));
        s.floor_program.set("u_uv_min", glm::vec2(to_vec3(corner)));
        s.floor_program.set("u_uv_max", glm::vec2(to_vec3(corner + glm::dvec3(2.0 * half))));
        s.floor_program.set("u_view_projection", view_projection);
        s.floor_program.set("u_log_depth_scale", depth_scale);
        s.floor_program.set("u_step", detail::to_float(step));
        s.floor_program.set("u_color", precompensate({0.055, 0.065, 0.085}));
        s.floor_program.set("u_opacity", 0.55F);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        s.floor_program.use();
        s.empty_vao.bind();
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glDisable(GL_BLEND);
    }
    glDepthMask(GL_TRUE);

    const auto draw_surface_mesh = [&](const MeshGpu& mesh, const glm::vec3& neutral,
                                       const ScalarRange& range, Colormap map, float opacity,
                                       glm::vec2 contours) {
        s.surface_program.set("u_model", model);
        s.surface_program.set("u_view_projection", view_projection);
        s.surface_program.set("u_log_depth_scale", depth_scale);
        s.surface_program.set("u_range",
                              glm::vec2(detail::to_float(range.min), detail::to_float(range.max)));
        s.surface_program.set("u_neutral", neutral);
        s.surface_program.set("u_key_direction", key);
        s.surface_program.set("u_ambient", detail::to_float(kAmbient));
        s.surface_program.set("u_specular", detail::to_float(kSpecular));
        s.surface_program.set("u_rim", detail::to_float(kRim));
        s.surface_program.set("u_opacity", opacity);
        s.surface_program.set("u_contours", contours);
        s.surface_program.set("u_contour_strength", detail::to_float(kContourStrength));
        lut(map).bind(0);
        s.surface_program.use();
        mesh.vao.bind();
        if (mesh.indices) {
            glDrawElements(GL_TRIANGLES, mesh.count, GL_UNSIGNED_INT, nullptr);
        } else {
            glDrawArrays(GL_TRIANGLES, 0, mesh.count);
        }
    };

    // 3. Body.
    if (s.surface && options.surface.enabled) {
        if (!s.surface_mesh || s.surface_smooth != options.surface.smooth) {
            auto mesh = upload_mesh(surface_vertices(*s.surface, options.surface.smooth), {});
            if (!mesh) {
                return core::failure("aero view: " + mesh.error());
            }
            s.surface_mesh = std::move(mesh).value();
            s.surface_smooth = options.surface.smooth;
        }
        const ScalarRange range = s.cp_range(options.surface.scale);
        const glm::vec2 contours =
            options.surface.smooth && s.surface_has_cp() ? contour_spacing(range) : glm::vec2(0.0F);
        draw_surface_mesh(*s.surface_mesh, precompensate(options.surface.neutral_albedo), range,
                          Colormap::diverging, 1.0F, contours);
        stats.surface_triangles = s.surface->mesh.triangles.size();
    }

    // 4. Section planes.
    if (s.flow) {
        std::vector<SectionGpu> kept;
        for (const SectionPlane& plane : options.sections) {
            if (!plane.enabled ||
                (plane.quantity == FieldQuantity::pressure && !s.flow->pressure) ||
                (flat_flow && plane.axis != Axis::z)) {
                continue;
            }
            auto cached = std::ranges::find_if(
                s.sections, [&](const SectionGpu& g) { return same_plane(g.plane, plane); });
            if (cached != s.sections.end()) {
                kept.push_back(std::move(*cached));
                s.sections.erase(cached);
                continue;
            }
            const ScalarGrid& grid = s.derived_grid(plane.quantity, plane.axis);
            SectionImage image = sample_section(grid, s.flow->solid, plane.axis, plane.position);
            if (image.values.empty()) {
                continue;
            }
            auto texture =
                Texture::create_2d(static_cast<int>(image.width), static_cast<int>(image.height),
                                   TextureFormat::rgba32f);
            if (!texture) {
                return core::failure("aero view: section texture: " + texture.error());
            }
            std::vector<float> texels;
            texels.reserve(image.values.size() * 4);
            for (std::size_t i = 0; i < image.values.size(); ++i) {
                texels.insert(texels.end(), {detail::to_float(image.values[i]),
                                             image.solid[i] != 0 ? 1.0F : 0.0F, 0.0F, 0.0F});
            }
            glTextureSubImage2D(texture.value().id(), 0, 0, 0, static_cast<GLsizei>(image.width),
                                static_cast<GLsizei>(image.height), GL_RGBA, GL_FLOAT,
                                texels.data());
            image.values.clear();
            image.solid.clear();
            const ScalarRange range =
                s.field_range(plane.quantity, plane.axis, effective_scale(plane));
            kept.push_back(
                SectionGpu{plane, std::move(texture).value(), image, range, plane.quantity});
        }
        s.sections = std::move(kept);
        const glm::vec3 solid_color = precompensate(options.surface.neutral_albedo * 0.85);
        for (const bool translucent : {false, true}) {
            for (const SectionGpu& g : s.sections) {
                const SectionPlane* plane = nullptr;
                for (const SectionPlane& p : options.sections) {
                    if (same_plane(p, g.plane)) {
                        plane = &p;
                    }
                }
                const double opacity =
                    plane != nullptr ? std::clamp(plane->opacity, 0.0, 1.0) : 1.0;
                if ((opacity < 0.999) != translucent) {
                    continue;
                }
                const glm::dvec3 corner = g.image.corner;
                const glm::dvec3 edge_u = g.image.u_step * static_cast<double>(g.image.width - 1);
                const glm::dvec3 edge_v = g.image.v_step * static_cast<double>(g.image.height - 1);
                const glm::vec2 uv_min(0.5F / static_cast<float>(g.image.width),
                                       0.5F / static_cast<float>(g.image.height));
                const glm::vec2 uv_max(1.0F - uv_min.x, 1.0F - uv_min.y);
                s.section_program.set("u_corner", to_vec3(camera.to_render_double(corner)));
                s.section_program.set("u_edge_u", to_vec3(edge_u));
                s.section_program.set("u_edge_v", to_vec3(edge_v));
                s.section_program.set("u_uv_min", uv_min);
                s.section_program.set("u_uv_max", uv_max);
                s.section_program.set("u_view_projection", view_projection);
                s.section_program.set("u_log_depth_scale", depth_scale);
                s.section_program.set("u_range", glm::vec2(detail::to_float(g.range.min),
                                                           detail::to_float(g.range.max)));
                s.section_program.set("u_solid_color", solid_color);
                s.section_program.set("u_opacity", detail::to_float(opacity));
                s.section_program.set("u_contours", is_signed(g.quantity) ? contour_spacing(g.range)
                                                                          : glm::vec2(0.0F));
                s.section_program.set("u_contour_strength", 0.0F);
                lut(colormap_of(g.plane)).bind(0);
                g.values.bind(1);
                if (translucent) {
                    glEnable(GL_BLEND);
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                    glDepthMask(GL_FALSE);
                }
                s.section_program.use();
                s.empty_vao.bind();
                glDrawArrays(GL_TRIANGLES, 0, 6);
                glDepthMask(GL_TRUE);
                glDisable(GL_BLEND);
                ++stats.section_planes;
            }
        }
    }

    // 5. Force arrows.
    if (s.forces && options.forces.enabled) {
        std::ostringstream key_text;
        key_text << options.forces.length_per_coefficient << options.forces.drag
                 << options.forces.lift << options.forces.side;
        if (s.arrow_key != key_text.str()) {
            s.lift_arrow.reset();
            s.drag_arrow.reset();
            s.side_arrow.reset();
            const double radius =
                0.006 * std::max(glm::length(to_glm(bounds.max) - to_glm(bounds.min)), 1.0e-3);
            const auto make = [&](const ForceComponent& c,
                                  std::optional<MeshGpu>& slot) -> core::Result<int, std::string> {
                const double length = c.coefficient * options.forces.length_per_coefficient;
                if (!(std::abs(length) > 4.0 * radius) || glm::length(c.direction) == 0.0) {
                    return 0;
                }
                std::vector<SurfaceVertex> vertices;
                append_arrow(vertices, s.forces->application_point,
                             c.direction * (length < 0.0 ? -1.0 : 1.0), std::abs(length), radius);
                auto mesh = upload_mesh(vertices, {});
                if (!mesh) {
                    return core::failure(std::string(mesh.error()));
                }
                slot = std::move(mesh).value();
                return 1;
            };
            if (options.forces.lift) {
                if (auto made = make(s.forces->lift, s.lift_arrow); !made) {
                    return core::failure("aero view: " + made.error());
                }
            }
            if (options.forces.drag) {
                if (auto made = make(s.forces->drag, s.drag_arrow); !made) {
                    return core::failure("aero view: " + made.error());
                }
            }
            if (options.forces.side) {
                if (auto made = make(s.forces->side, s.side_arrow); !made) {
                    return core::failure("aero view: " + made.error());
                }
            }
            s.arrow_key = key_text.str();
        }
        const ScalarRange none{0.0, 1.0};
        if (s.lift_arrow) {
            draw_surface_mesh(*s.lift_arrow, precompensate(display_linear(lift_ui_color()) * 0.9),
                              none, Colormap::diverging, 1.0F, glm::vec2(0.0F));
            ++stats.force_arrows;
        }
        if (s.drag_arrow) {
            draw_surface_mesh(*s.drag_arrow, precompensate(display_linear(drag_ui_color()) * 0.9),
                              none, Colormap::diverging, 1.0F, glm::vec2(0.0F));
            ++stats.force_arrows;
        }
        if (s.side_arrow) {
            draw_surface_mesh(*s.side_arrow, precompensate({0.55, 0.45, 0.75}), none,
                              Colormap::diverging, 1.0F, glm::vec2(0.0F));
            ++stats.force_arrows;
        }
    }

    // 6. Q-criterion iso-surface (translucent: back faces, then front faces).
    if (s.flow && options.iso.enabled && !flat_flow) {
        if (!s.iso || s.iso->level != options.iso.level ||
            s.iso->absolute != options.iso.absolute) {
            const ScalarGrid& q = s.derived_grid(FieldQuantity::q_criterion, Axis::z);
            double peak = 0.0;
            for (const double v : q.values) {
                peak = std::max(peak, v);
            }
            const double level =
                options.iso.absolute ? options.iso.level : options.iso.level * peak;
            IsoGpu iso;
            iso.level = options.iso.level;
            iso.absolute = options.iso.absolute;
            if (peak > 0.0 && level > 0.0) {
                auto surface = marching_cubes_gpu(*s.compute, q, level);
                if (!surface) {
                    return core::failure("aero view: " + surface.error());
                }
                const IsoSurface& m = surface.value();
                std::vector<SurfaceVertex> vertices;
                vertices.reserve(m.positions.size());
                for (std::size_t v = 0; v < m.positions.size(); ++v) {
                    vertices.push_back(
                        {to_vec3(m.positions[v]), to_vec3(m.normals[v]), glm::vec2(0.0F)});
                }
                if (!m.indices.empty()) {
                    auto mesh = upload_mesh(vertices, m.indices);
                    if (!mesh) {
                        return core::failure("aero view: " + mesh.error());
                    }
                    iso.mesh = std::move(mesh).value();
                    iso.triangles = m.triangle_count();
                }
            }
            s.iso = std::move(iso);
        }
        if (s.iso->mesh) {
            const auto opacity = detail::to_float(std::clamp(options.iso.opacity, 0.0, 1.0));
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glDepthMask(GL_FALSE);
            glEnable(GL_CULL_FACE);
            for (const GLenum cull : {GL_BACK, GL_FRONT}) {
                glCullFace(cull == GL_BACK ? GL_FRONT : GL_BACK);
                draw_surface_mesh(*s.iso->mesh, precompensate(options.iso.color), {0.0, 1.0},
                                  Colormap::diverging, opacity, glm::vec2(0.0F));
            }
            glDisable(GL_CULL_FACE);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
            stats.iso_triangles = s.iso->triangles;
        }
    }

    // 7. Streamlines.
    if (s.flow && options.streamlines.enabled) {
        const std::vector<glm::dvec3> seeds = s.seeds_for(options.streamlines);
        if (!s.streamlines || s.streamlines->seeds != seeds ||
            !same_settings(s.streamlines->settings, options.streamlines.settings)) {
            auto lines = trace_streamlines_gpu(*s.compute, s.flow->velocity, s.flow->solid, seeds,
                                               options.streamlines.settings);
            if (!lines) {
                return core::failure("aero view: " + lines.error());
            }
            StreamlinesGpu gpu{
                seeds, options.streamlines.settings, std::nullopt, VertexArray::create(), 0, 0, 0};
            struct Segment {
                glm::vec4 p0;
                glm::vec4 p1;
                glm::vec2 alpha;
            };
            std::vector<Segment> segments;
            for (const Streamline& line : lines.value()) {
                if (line.points.size() < 2) {
                    continue;
                }
                ++gpu.lines;
                gpu.points += line.points.size();
                const std::size_t n = line.points.size();
                for (std::size_t i = 0; i + 1 < n; ++i) {
                    // Ends taper over the first and last few segments.
                    const auto taper = [n](std::size_t k) {
                        const double d = static_cast<double>(std::min(k, n - 1 - k));
                        return detail::to_float(std::min(1.0, 0.25 + d / 6.0));
                    };
                    segments.push_back(
                        {glm::vec4(to_vec3(line.points[i]), detail::to_float(line.speed[i])),
                         glm::vec4(to_vec3(line.points[i + 1]),
                                   detail::to_float(line.speed[i + 1])),
                         glm::vec2(taper(i), taper(i + 1))});
                }
            }
            if (!segments.empty()) {
                auto buffer = Buffer::from(std::span<const Segment>(segments));
                if (!buffer) {
                    return core::failure("aero view: streamlines: " + buffer.error());
                }
                gpu.segments = std::move(buffer).value();
                gpu.vao.set_vertex_buffer(0, *gpu.segments, 0, sizeof(Segment));
                gpu.vao.set_float_attribute(0, 0, 4, 0);
                gpu.vao.set_float_attribute(1, 0, 4, 4 * sizeof(float));
                gpu.vao.set_float_attribute(2, 0, 2, 8 * sizeof(float));
                gpu.vao.set_binding_divisor(0, 1);
                gpu.count = static_cast<GLsizei>(segments.size());
            }
            s.streamlines = std::move(gpu);
        }
        if (s.streamlines->count > 0) {
            const ScalarRange range = s.field_range(FieldQuantity::speed, Axis::z, {});
            const auto half_width = detail::to_float(0.5 * options.streamlines.width_px);
            s.line_program.set("u_view_projection", view_projection);
            s.line_program.set("u_offset", to_vec3(offset));
            s.line_program.set("u_viewport", viewport);
            s.line_program.set("u_half_width", half_width);
            s.line_program.set("u_near", near_w);
            s.line_program.set("u_range",
                               glm::vec2(detail::to_float(range.min), detail::to_float(range.max)));
            s.line_program.set("u_use_colormap", options.streamlines.color_by_speed ? 1 : 0);
            s.line_program.set("u_color", precompensate(options.streamlines.color));
            s.line_program.set("u_log_depth_scale", depth_scale);
            lut(Colormap::sequential).bind(0);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glDepthMask(GL_FALSE);
            s.line_program.use();
            s.streamlines->vao.bind();
            glDrawArraysInstanced(GL_TRIANGLES, 0, 6, s.streamlines->count);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
            stats.streamlines = s.streamlines->lines;
            stats.streamline_points = s.streamlines->points;
        }
    }

    // 8. Particles, straight from the kernel's trace buffer.
    if (s.flow && options.particles.enabled && s.particles) {
        const ParticlesGpu& p = *s.particles;
        const GridGeometry& g = s.flow->velocity.grid;
        s.particle_program.set("u_view_projection", view_projection);
        s.particle_program.set("u_grid_origin", to_vec3(camera.to_render_double(g.origin)));
        s.particle_program.set("u_spacing", detail::to_float(g.spacing));
        set_uint(s.particle_program, "u_trace_points", static_cast<std::uint32_t>(p.trace_points));
        set_uint(s.particle_program, "u_head", p.head);
        s.particle_program.set("u_viewport", viewport);
        s.particle_program.set("u_half_width", detail::to_float(0.5 * options.particles.width_px));
        s.line_program.set("u_half_width", detail::to_float(0.5 * options.particles.width_px));
        s.particle_program.set("u_near", near_w);
        s.particle_program.set("u_lifetime", detail::to_float(options.particles.lifetime));
        s.particle_program.set("u_color", precompensate(options.particles.color));
        s.particle_program.set("u_opacity",
                               detail::to_float(std::clamp(options.particles.opacity, 0.0, 1.0)));
        s.particle_program.set("u_log_depth_scale", depth_scale);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, s.compute->buffer_name(p.traces.id()));
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        s.particle_program.use();
        s.empty_vao.bind();
        glDrawArraysInstanced(GL_TRIANGLES, 0, 6,
                              static_cast<GLsizei>(p.count * (p.trace_points - 1)));
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, 0);
        stats.particles = p.count;
    }
    glBindVertexArray(0);
    return stats;
}

void AeroView::draw_overlay(const FrameInfo& frame, const AeroViewOptions& options) {
    State& s = *state_;
    if (!options.legend.enabled || frame.width <= 0 || frame.height <= 0) {
        return;
    }
    const std::vector<Legend> shown = legends(options);
    std::vector<std::pair<std::string, UiColor>> readout;
    if (s.forces && options.forces.enabled) {
        if (options.forces.lift) {
            readout.emplace_back(format_coefficient("CL", s.forces->lift.coefficient, 3),
                                 lift_ui_color());
        }
        if (options.forces.drag) {
            readout.emplace_back(format_coefficient("CD", s.forces->drag.coefficient, 4),
                                 drag_ui_color());
        }
    }
    if (shown.empty() && readout.empty()) {
        return;
    }
    // Metrics scale with the frame height (reference 720 rows).
    const double scale = std::clamp(static_cast<double>(frame.height) / 720.0, 0.75, 3.0);
    const double pad = 12.0 * scale;
    const double title_cap = 9.5 * scale;
    const double label_cap = 8.0 * scale;
    const double bar_w = options.legend.thickness_px * scale * 0.85;
    const double bar_h = options.legend.length_px * scale * 0.62;
    const double tick = 4.0 * scale;
    const double gap = 12.0 * scale;
    const double margin = options.legend.margin_px * scale * 0.6;

    std::ostringstream key;
    key << frame.width << 'x' << frame.height << ';' << scale << ';';
    for (const Legend& l : shown) {
        key << l.title << ';' << l.range.min << ';' << l.range.max << ';'
            << static_cast<int>(l.colormap) << ';';
    }
    for (const auto& r : readout) {
        key << r.first << ';';
    }
    if (key.str() != s.overlay_key) {
        // Panel sizes.
        struct Panel {
            double w = 0.0;
            double h = 0.0;
        };
        std::vector<Panel> panels;
        for (const Legend& l : shown) {
            double label_w = 0.0;
            for (const LegendTick& t : l.ticks) {
                label_w = std::max(label_w, detail::Canvas::text_width(t.label, label_cap));
            }
            const double body = bar_w + tick + 4.0 * scale + label_w;
            const double title_w = detail::Canvas::text_width(l.title, title_cap);
            panels.push_back({std::max(body, title_w) + 2.0 * pad,
                              pad + title_cap + 1.6 * title_cap + bar_h + 0.5 * label_cap + pad});
        }
        Panel readout_panel;
        if (!readout.empty()) {
            double w = 0.0;
            for (const auto& r : readout) {
                w = std::max(w, detail::Canvas::text_width(r.first, title_cap));
            }
            readout_panel = {w + 2.0 * pad + 14.0 * scale,
                             2.0 * pad + static_cast<double>(readout.size()) * title_cap * 2.0 -
                                 title_cap};
        }
        double total_w = 0.0;
        double total_h = readout_panel.h;
        for (const Panel& p : panels) {
            total_w += p.w + gap;
            total_h = std::max(total_h, p.h);
        }
        if (!readout.empty()) {
            total_w += readout_panel.w + gap;
        }
        total_w = std::max(total_w - gap, 1.0);
        const int width = static_cast<int>(std::ceil(total_w)) + 2;
        const int height = static_cast<int>(std::ceil(total_h)) + 2;
        detail::Canvas canvas(width, height);
        const detail::OverlayStyle style = detail::overlay_style();
        const double radius = default_theme().window_rounding * scale;
        double x = 1.0;
        // Read-out first (left), then the legends.
        if (!readout.empty()) {
            const double y = static_cast<double>(height) - 1.0 - readout_panel.h;
            canvas.fill_rounded_rect(x, y, readout_panel.w, readout_panel.h, radius, style.panel);
            canvas.stroke_rounded_rect(x, y, readout_panel.w, readout_panel.h, radius, 1.0,
                                       style.border);
            double baseline = y + pad + title_cap;
            for (const auto& [text, color] : readout) {
                const glm::dvec4 swatch{color.r, color.g, color.b, 1.0};
                canvas.fill_rounded_rect(x + pad, baseline - title_cap, 6.0 * scale, title_cap,
                                         1.5 * scale, swatch);
                canvas.text(x + pad + 14.0 * scale, baseline, text, title_cap, style.text);
                baseline += 2.0 * title_cap;
            }
            x += readout_panel.w + gap;
        }
        for (std::size_t i = 0; i < shown.size(); ++i) {
            const Legend& l = shown[i];
            const Panel& p = panels[i];
            const double y = static_cast<double>(height) - 1.0 - p.h;
            canvas.fill_rounded_rect(x, y, p.w, p.h, radius, style.panel);
            canvas.stroke_rounded_rect(x, y, p.w, p.h, radius, 1.0, style.border);
            canvas.text(x + pad, y + pad + title_cap, l.title, title_cap, style.text);
            const double bar_x = x + pad;
            const double bar_y = y + pad + title_cap + 1.6 * title_cap;
            canvas.fill_colour_bar(bar_x, bar_y, bar_w, bar_h, l.colormap);
            canvas.stroke_rounded_rect(bar_x - 0.5, bar_y - 0.5, bar_w + 1.0, bar_h + 1.0, 1.0, 1.0,
                                       style.bar_outline);
            for (const LegendTick& t : l.ticks) {
                const double ty = bar_y + (1.0 - t.position) * bar_h;
                canvas.line({bar_x + bar_w + 1.0, ty}, {bar_x + bar_w + tick, ty}, 1.0,
                            style.muted);
                canvas.text(bar_x + bar_w + tick + 4.0 * scale, ty + 0.5 * label_cap, t.label,
                            label_cap, style.muted);
            }
            x += p.w + gap;
        }
        auto texture = Texture::create_2d(width, height, TextureFormat::rgba8);
        if (!texture) {
            return;
        }
        const std::vector<std::uint8_t> bytes = canvas.to_rgba8_bottom_up();
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTextureSubImage2D(texture.value().id(), 0, 0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE,
                            bytes.data());
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        s.overlay_texture = std::move(texture).value();
        s.overlay_key = key.str();
        s.overlay_size = {width, height};
        s.overlay_position = {
            static_cast<int>(std::lround(static_cast<double>(frame.width) - margin - width)),
            static_cast<int>(std::lround(static_cast<double>(frame.height) - margin - height))};
    }
    if (!s.overlay_texture) {
        return;
    }
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    s.overlay_program.set("u_rect", glm::vec4(static_cast<float>(s.overlay_position.x),
                                              static_cast<float>(s.overlay_position.y),
                                              static_cast<float>(s.overlay_size.x),
                                              static_cast<float>(s.overlay_size.y)));
    s.overlay_program.set(
        "u_viewport", glm::vec2(static_cast<float>(frame.width), static_cast<float>(frame.height)));
    s.overlay_texture->bind(0);
    s.overlay_program.use();
    s.empty_vao.bind();
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glDisable(GL_BLEND);
    glBindVertexArray(0);
}

} // namespace orrery::render
