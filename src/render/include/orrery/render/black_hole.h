#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include <glm/ext/vector_double2.hpp>
#include <glm/ext/vector_double3.hpp>
#include <glm/ext/vector_float4.hpp>

#include "orrery/core/body.h"
#include "orrery/core/schwarzschild_geodesic.h"
#include "orrery/core/sim_snapshot.h"
#include "orrery/render/camera.h"
#include "orrery/render/hdr_pipeline.h"
#include "orrery/render/render_settings.h"

// Black-hole lensing pass, CPU side (docs/rendering.md, "Black hole";
// ADR-003, ADR-017). The GPU pass traces one Schwarzschild null geodesic per
// half-resolution sample (black_hole_trace.comp.glsl, CPU twin
// core::geodesic::trace_ray), then upsamples to full resolution and looks
// the escaping directions up in an environment cube map of the sky
// (black_hole_composite.frag.glsl). This header holds what can be computed
// and tested without OpenGL: which hole is lensed, its apparent size, the
// sample grid and the ray of every pixel.
namespace orrery::render {

// Thin disk around the lensed hole, in units of M = G M / c^2: from the ISCO
// (6 M = 3 r_s) to kDiskOuterRadius, in the plane normal to world +Z (the
// reference plane of the scenario; spin is not modelled, ADR-003).
inline constexpr double kDiskOuterRadius = 30.0;
[[nodiscard]] core::geodesic::DiskModel lensing_disk() noexcept;

// The pass runs when the hole's Einstein angle for a source at infinity,
// sqrt(4 M / D), covers at least this many pixels: below it the deflection
// of the sky stays under a quarter of a pixel everywhere.
inline constexpr double kLensingMinEinsteinPx = 0.25;

struct LensingTarget {
    std::size_t index = 0;        // snapshot index of the hole
    core::BodyId id;              // its id
    double mass_length = 0.0;     // M = G M / c^2 [m]
    glm::dvec3 hole_render{0.0};  // hole centre in render space (camera at the origin) [m]
    double observer_radius = 0.0; // camera distance from the hole [M]
    double einstein_radius_px = 0.0;
    double shadow_radius_px = 0.0; // apparent radius of the shadow at the image centre [px]
};

// Angular radius of the shadow seen by a static observer at radius r [M]:
// asin(3 sqrt(3) sqrt(1 - 2/r) / r) for r >= 3 (pi/2 at the photon sphere);
// inside the photon sphere the shadow covers more than half of the sky.
[[nodiscard]] double shadow_angular_radius(double observer_radius) noexcept;
// Einstein angle for a source at infinity, sqrt(4 / r) for r >> 1 [rad].
[[nodiscard]] double einstein_angle(double observer_radius) noexcept;

// The black hole the pass lenses: the one with the largest Einstein angle in
// pixels, if lensing is enabled and that angle reaches kLensingMinEinsteinPx.
[[nodiscard]] std::optional<LensingTarget>
select_lensed_black_hole(const core::SimSnapshot& snapshot, const Camera& camera,
                         const FrameInfo& frame, const RenderSettings& settings);

// Half-resolution sample grid of the trace: sample (i, j) sits at the
// full-resolution pixel corner (2i + 1, 2j + 1) (OpenGL window coordinates,
// origin at the bottom left), so that every pixel centre has four samples
// around it. The grid has width / 2 + 1 by height / 2 + 1 samples.
struct TraceGrid {
    int width = 0;
    int height = 0;
};
[[nodiscard]] TraceGrid trace_grid(const FrameInfo& frame) noexcept;
[[nodiscard]] glm::dvec2 trace_sample_pixel(int i, int j) noexcept;

// World direction of the ray through a point of the image (OpenGL window
// coordinates in pixels), as the shaders compute it.
[[nodiscard]] glm::dvec3 pixel_ray(const Camera& camera, const FrameInfo& frame,
                                   const glm::dvec2& window_pixel);

// Geometry read back from the trace for tests (GPU against the CPU
// reference): per sample, row by row from the bottom,
//   disk: first on-disk crossing radius [M], its g, number of on-disk
//         crossings, fate (0 escaped, 1 captured, 2 exhausted)
//   sky:  escape direction (world), impact parameter [M]
// The first crossing radius and g are 0 when the ray never meets the disk.
struct GeodesicTraceImage {
    int width = 0;
    int height = 0;
    std::vector<glm::vec4> disk;
    std::vector<glm::vec4> sky;
};

} // namespace orrery::render
