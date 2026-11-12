#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <glm/ext/vector_double2.hpp>
#include <glm/ext/vector_float4.hpp>

#include "orrery/core/body.h"
#include "orrery/core/result.h"
#include "orrery/core/sim_snapshot.h"
#include "orrery/render/body_visuals.h"
#include "orrery/render/camera.h"
#include "orrery/render/gl_resources.h"
#include "orrery/render/hdr_pipeline.h"
#include "orrery/render/render_settings.h"
#include "orrery/render/starfield.h"
#include "orrery/render/trails.h"

// Space view (docs/rendering.md, "Bodies", "Starfield", "Trails"): draws a
// simulation snapshot into the HDR scene target of the Phase 4a pipeline.
//
//   1. Milky Way band and starfield, at infinity (rotation only, no depth)
//   2. bodies: instanced icospheres, LOD per body from the projected size,
//      floating origin, log depth, PBR-lite lighting, emissive stars
//   3. orbit trails: anti-aliased screen-space lines, fading with age
//   4. glow sprites: star halos and the black-hole ring (additive)
//
// Frame preparation (visual radius, LOD, culling, lighting, instance data)
// is done by prepare_bodies on the CPU without OpenGL and unit-tested.
namespace orrery::render {

// GPU layout of one body instance (body.vert.glsl, locations 1..7).
struct BodyInstance {
    glm::vec4 centre_radius{0.0F};      // render-space centre [m], visual radius [m]
    glm::vec4 albedo_style{0.0F};       // linear albedo, SurfaceStyle
    glm::vec4 secondary_specular{0.0F}; // secondary colour, specular weight
    glm::vec4 light_seed{0.0F};         // unit vector towards the light, noise seed
    glm::vec4 irradiance{0.0F};         // light irradiance (scene-linear), marker weight
    glm::vec4 emission{0.0F};           // emitted radiance (scene-linear)
    glm::vec4 axis{0.0F};               // unit spin axis (world axes)
};

// GPU layout of one glow sprite (glow.vert.glsl).
struct GlowInstance {
    glm::vec4 centre_kind{0.0F}; // render-space centre [m], 0 star / 1 black hole
    glm::vec4 color_disc{0.0F};  // scene-linear colour, disc radius [px]
};

// Where a body ended up on screen (for labels, picking and tests).
struct BodyScreenInfo {
    core::BodyId id;
    std::size_t index = 0;        // index in the snapshot
    bool visible = false;         // in front of the camera and inside the frustum
    glm::dvec2 pixel{0.0};        // projected centre, x right, y down, origin top-left [px]
    double distance = 0.0;        // eye to centre [m]
    double physical_radius = 0.0; // [m]
    double visual_radius = 0.0;   // drawn radius [m]
    double radius_px = 0.0;       // drawn radius on screen [px]
    std::size_t lod = 0;
};

struct PreparedBodies {
    // Instances grouped by LOD: lod_first[i] .. lod_first[i] + lod_count[i].
    std::vector<BodyInstance> instances;
    std::array<std::size_t, kLodCount> lod_first{};
    std::array<std::size_t, kLodCount> lod_count{};
    std::vector<GlowInstance> glows;
    std::vector<BodyScreenInfo> screen; // one per snapshot body, snapshot order
    std::size_t culled = 0;
};

// World position [m] of body `index` (AU -> metres in double).
[[nodiscard]] glm::dvec3 world_position(const core::SimSnapshot& snapshot, std::size_t index);

// CPU preparation of the bodies of one frame.
[[nodiscard]] PreparedBodies prepare_bodies(const core::SimSnapshot& snapshot, const Camera& camera,
                                            const FrameInfo& frame, const RenderSettings& settings);

struct SpaceView {
    const core::SimSnapshot* snapshot = nullptr;
    const TrailHistory* trails = nullptr; // optional
    const Camera* camera = nullptr;
    RenderSettings settings;
};

struct SpaceDrawStats {
    std::size_t bodies_drawn = 0;
    std::size_t bodies_culled = 0;
    std::array<std::size_t, kLodCount> per_lod{};
    std::size_t trail_segments = 0;
    std::size_t stars = 0;
    std::size_t glows = 0;
};

class SpaceRenderer {
public:
    // Requires a current OpenGL 4.6 context; generates the starfield.
    [[nodiscard]] static core::Result<SpaceRenderer, std::string>
    create(const StarfieldParams& starfield = {});

    SpaceRenderer(SpaceRenderer&& other) noexcept;
    SpaceRenderer& operator=(SpaceRenderer&& other) noexcept;
    SpaceRenderer(const SpaceRenderer&) = delete;
    SpaceRenderer& operator=(const SpaceRenderer&) = delete;
    // Releases the GL objects: the creating context must be current.
    ~SpaceRenderer();

    // Draws into the bound HDR scene framebuffer (call inside the scene
    // callback of OffscreenRenderer::render or after HdrPipeline::begin_scene).
    // The exposure setting is applied by the caller to the HDR pipeline.
    SpaceDrawStats draw(const SpaceView& view, const FrameInfo& frame);

    [[nodiscard]] std::size_t star_count() const noexcept { return star_count_; }

private:
    struct Gpu;
    SpaceRenderer(std::unique_ptr<Gpu> gpu, std::size_t star_count) noexcept;

    std::unique_ptr<Gpu> gpu_;
    std::size_t star_count_ = 0;
    std::vector<TrailSegment> segment_scratch_;
};

// Colour and opacity of a body's trail.
[[nodiscard]] glm::vec4 trail_color(const BodyAppearance& appearance, double brightness) noexcept;

} // namespace orrery::render
