#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include <glm/ext/vector_double3.hpp>

#include "orrery/core/result.h"
#include "orrery/geom/triangle_mesh.h"
#include "orrery/render/aero_colormap.h"
#include "orrery/render/aero_field.h"
#include "orrery/render/aero_streamlines.h"
#include "orrery/render/aero_surface.h"
#include "orrery/render/camera.h"
#include "orrery/render/hdr_pipeline.h"

// Aero view (docs/aero-visualisation.md): draws an aerodynamic result into
// the HDR scene target of the Phase 4a pipeline, like SpaceRenderer does for
// the space scene.
//
//   1. studio backdrop (dark vertical gradient, no depth)
//   2. section planes: axis-aligned cuts through the flow field coloured by
//      speed, vorticity, pressure or Q (unlit, colour map in display space)
//   3. the body: mesh coloured by Cp with the diverging map (or the neutral
//      body colour), soft key/fill lighting, log depth
//   4. Q-criterion iso-surface (marching cubes on the GPU), translucent
//   5. streamlines (traced on the GPU) as anti-aliased screen-space lines
//   6. particles advected on the GPU with fading traces
//   7. drag, lift and side force arrows
// and, in the display-referred overlay after tone mapping, the colour
// legends with their numeric range (draw_overlay). A user interface may draw
// the legends itself from legends() instead.
//
// Plain data in (AeroSurface, FlowField, AeroForces, AeroViewOptions), no
// user-interface code. Bloom is off in the aero view (spec 7.6): nothing
// here adds glow, and the colour-mapped layers are pre-compensated for the
// display transform so that their colours match the legend
// (scene_linear_for_display).
namespace orrery::render {

// Recommended pipeline settings for the aero view: exposure 1 (the colour
// pre-compensation assumes AeroViewOptions::exposure equals the pipeline
// exposure), 4x MSAA, the backdrop's bottom colour as clear colour.
[[nodiscard]] HdrSettings aero_hdr_settings();

struct SurfaceLayer {
    bool enabled = true;
    // Cp scale; symmetric keeps Cp = 0 at the neutral centre of the map.
    ColorScale scale{ScaleMode::symmetric};
    // Interpolate Cp to the vertices (vertex_values) instead of flat
    // per-triangle colours.
    bool smooth = true;
    // Linear albedo of a body without Cp (and of NaN triangles).
    glm::dvec3 neutral_albedo{0.36, 0.38, 0.42};
};

struct SectionPlane {
    bool enabled = true;
    Axis axis = Axis::z;
    double position = 0.0; // world coordinate along `axis` [m], clamped into the grid
    FieldQuantity quantity = FieldQuantity::speed;
    // Symmetric is used for signed quantities whatever the mode says unless
    // the mode is fixed.
    ColorScale scale;
    // Empty: diverging_dark for signed quantities, sequential otherwise.
    std::optional<Colormap> colormap;
    double opacity = 1.0;
};

struct StreamlineLayer {
    bool enabled = false;
    // Seed points (world, m); empty: a rake of 12 x 6 seeds on the upstream
    // face of the grid (a line of 24 seeds for a 2D field).
    std::vector<glm::dvec3> seeds;
    StreamlineSettings settings;
    double width_px = 1.6;
    double opacity = 0.9;
    // Colour by speed with the sequential map; otherwise `color`.
    bool color_by_speed = true;
    glm::dvec3 color{0.85, 0.88, 0.92}; // display-linear
};

struct ParticleLayer {
    bool enabled = false;
    std::size_t count = 4096;
    // Particles are released on the seeds of StreamlineLayer (or its default
    // rake), spread over half a grid spacing, and live this long [s of
    // animation time] before they are released again.
    double lifetime = 4.0;
    // Animation seconds per second of advance(dt) time.
    double time_scale = 1.0;
    // Trace length: positions kept per particle (>= 2, <= 64).
    std::size_t trace_points = 12;
    double width_px = 1.4;
    glm::dvec3 color{0.92, 0.94, 0.97}; // display-linear
    double opacity = 0.85;
    std::uint32_t random_seed = 1;
};

struct IsoSurfaceLayer {
    bool enabled = false;
    // Q level as a fraction of the largest Q of the field (> 0), or the
    // absolute value [1/s^2] when `absolute` is true.
    double level = 0.05;
    bool absolute = false;
    glm::dvec3 color{0.62, 0.68, 0.76}; // display-linear
    double opacity = 0.35;
};

struct ForceLayer {
    bool enabled = false;
    // Arrow length per unit force coefficient [m].
    double length_per_coefficient = 1.0;
    bool drag = true;
    bool lift = true;
    bool side = false;
    double width_px = 3.0;
};

struct LegendLayer {
    bool enabled = true;
    // Bar length and thickness at a 1080-row frame, scaled with the frame
    // height [px].
    double length_px = 320.0;
    double thickness_px = 14.0;
    // Margin from the right and bottom edges [px at 1080 rows].
    double margin_px = 36.0;
};

struct AeroViewOptions {
    SurfaceLayer surface;
    std::vector<SectionPlane> sections;
    StreamlineLayer streamlines;
    ParticleLayer particles;
    IsoSurfaceLayer iso;
    ForceLayer forces;
    LegendLayer legend;
    // Studio backdrop, scene-linear.
    glm::dvec3 backdrop_top{0.004, 0.005, 0.008};
    glm::dvec3 backdrop_bottom{0.028, 0.032, 0.042};
    // Exposure of the HDR pipeline the view draws into.
    double exposure = 1.0;
};

struct AeroDrawStats {
    std::size_t surface_triangles = 0;
    std::size_t section_planes = 0;
    std::size_t streamlines = 0;
    std::size_t streamline_points = 0;
    std::size_t particles = 0;
    std::size_t iso_triangles = 0;
    std::size_t force_arrows = 0;
};

// Camera that frames `bounds` (world, m) for a view from (yaw, pitch) with
// the given lens and aspect, focus at the bounds centre.
[[nodiscard]] TurntableMode frame_bounds(const geom::Bounds& bounds, double aspect,
                                         const CameraLens& lens, double yaw = 0.6,
                                         double pitch = 0.35);

class AeroView {
public:
    // Requires a current OpenGL 4.6 context; compiles the shaders and builds
    // its compute backend (make_aero_compute_backend).
    [[nodiscard]] static core::Result<AeroView, std::string> create();

    AeroView(AeroView&& other) noexcept;
    AeroView& operator=(AeroView&& other) noexcept;
    AeroView(const AeroView&) = delete;
    AeroView& operator=(const AeroView&) = delete;
    // Releases the GL objects: the creating context must be current.
    ~AeroView();

    // Replaces the body (an empty mesh removes it). Fails when cp is neither
    // empty nor one value per triangle.
    [[nodiscard]] core::Result<std::monostate, std::string> set_surface(const AeroSurface& surface);
    // Replaces the flow field (validate(FlowField)); streamlines, the
    // iso-surface and the particles are rebuilt from it on the next draw.
    [[nodiscard]] core::Result<std::monostate, std::string> set_flow(FlowField field);
    void clear_flow();
    void set_forces(const std::optional<AeroForces>& forces);
    [[nodiscard]] bool has_flow() const noexcept;

    // Advances the particles by dt seconds (scaled by time_scale) on the GPU.
    // Does nothing without a flow field or with particles disabled.
    [[nodiscard]] core::Result<std::monostate, std::string> advance(double dt,
                                                                    const AeroViewOptions& options);
    // Releases every particle again from its seed.
    void reset_particles();

    // Draws into the bound HDR scene framebuffer (inside the scene callback
    // of OffscreenRenderer::render, or after HdrPipeline::begin_scene).
    [[nodiscard]] core::Result<AeroDrawStats, std::string>
    draw(const Camera& camera, const FrameInfo& frame, const AeroViewOptions& options);
    // Draws the legends into the bound sRGB overlay target (inside the
    // overlay callback, or after HdrPipeline::begin_overlay).
    void draw_overlay(const FrameInfo& frame, const AeroViewOptions& options);

    // The legends draw_overlay shows: the Cp legend when the surface has Cp,
    // then one per enabled section plane with a different quantity.
    [[nodiscard]] std::vector<Legend> legends(const AeroViewOptions& options) const;
    // Bounds of the surface and the flow grid together (all zero when empty).
    [[nodiscard]] geom::Bounds scene_bounds() const noexcept;

private:
    struct State;
    explicit AeroView(std::unique_ptr<State> state) noexcept;
    std::unique_ptr<State> state_;
};

} // namespace orrery::render
