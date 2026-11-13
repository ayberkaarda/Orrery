#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <glm/ext/vector_double3.hpp>

#include "orrery/aero/compute/compute_backend.h"
#include "orrery/core/result.h"
#include "orrery/render/aero_field.h"

// Streamlines and particle traces (docs/aero-visualisation.md,
// "Streamlines and particles").
//
// Streamlines are integrated in arc length, dx/ds = u / |u|, with the
// classical fourth-order Runge-Kutta scheme and a fixed step; the velocity
// between nodes is trilinear (bilinear on a 2D grid). A line stops when it
// leaves the grid, reaches a solid node, falls below the stagnation speed or
// has max_points points.
// Particles move in time, dx/dt = u, with the second-order midpoint rule.
//
// The CPU functions here (double) are the reference of the GPU kernels
// aero_vis_flow_streamlines and aero_vis_flow_particles (binary32).
namespace orrery::render {

enum class TraceDirection : std::uint8_t { forward, backward, both };

struct StreamlineSettings {
    // Arc length per step, in grid spacings.
    double step = 0.5;
    // Points per line (both directions together), >= 2.
    std::size_t max_points = 400;
    TraceDirection direction = TraceDirection::forward;
    // A line stops where |u| falls below this fraction of the largest speed
    // of the field.
    double stagnation_fraction = 1.0e-3;
};

// Points from the upstream end to the downstream end and |u| at each.
struct Streamline {
    std::vector<glm::dvec3> points;
    std::vector<double> speed;
};

[[nodiscard]] std::vector<Streamline> trace_streamlines(const VectorGrid& velocity,
                                                        std::span<const std::uint8_t> solid,
                                                        std::span<const glm::dvec3> seeds,
                                                        const StreamlineSettings& settings);

// The same lines from the GPU kernel (binary32 positions and velocities);
// `backend` must provide kernel_streamlines (make_aero_compute_backend).
[[nodiscard]] core::Result<std::vector<Streamline>, std::string>
trace_streamlines_gpu(aero::compute::IComputeBackend& backend, const VectorGrid& velocity,
                      std::span<const std::uint8_t> solid, std::span<const glm::dvec3> seeds,
                      const StreamlineSettings& settings);

// `count` seeds evenly spaced on the segment from `a` to `b` (the ends
// included; a single seed sits at the midpoint).
[[nodiscard]] std::vector<glm::dvec3> seed_line(const glm::dvec3& a, const glm::dvec3& b,
                                                std::size_t count);
// A rake of count_u x count_v seeds on the plane normal to `axis` at
// `position`, inset by half a seed spacing from the edges of the grid.
[[nodiscard]] std::vector<glm::dvec3> seed_plane(const GridGeometry& grid, Axis axis,
                                                 double position, std::size_t count_u,
                                                 std::size_t count_v);

// One midpoint step of a particle over dt seconds; empty when the particle
// (or its midpoint) is outside the grid or in a solid node.
[[nodiscard]] std::optional<glm::dvec3> advect_particle(const VectorGrid& velocity,
                                                        std::span<const std::uint8_t> solid,
                                                        const glm::dvec3& position, double dt);

// The same step for many particles in the GPU kernel (binary32), without
// release or traces; `backend` must provide kernel_particles.
[[nodiscard]] core::Result<std::vector<std::optional<glm::dvec3>>, std::string>
advect_particles_gpu(aero::compute::IComputeBackend& backend, const VectorGrid& velocity,
                     std::span<const std::uint8_t> solid, std::span<const glm::dvec3> positions,
                     double dt);

} // namespace orrery::render
