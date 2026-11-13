#pragma once

#include <memory>
#include <string>
#include <string_view>

#include "orrery/core/result.h"
#include "orrery/render/gl_compute_backend.h"

// GPU kernels of the aero visualisation (docs/aero-visualisation.md, "GPU
// paths"). They run through aero::compute::IComputeBackend like the LBM
// kernels: GLSL 460 compute shaders embedded from src/render/shaders/aero/
// compute (aero_vis_*.comp.glsl; the aero_vis_flow_* kernels share
// aero_vis_flow_common.glsl), dispatched by name.
// The CPU references live in aero_streamlines.h and aero_isosurface.h.
namespace orrery::render {

inline constexpr std::string_view kernel_streamlines = "aero_vis_flow_streamlines";
inline constexpr std::string_view kernel_particles = "aero_vis_flow_particles";
inline constexpr std::string_view kernel_marching_cubes = "aero_vis_marching_cubes";

// An OpenGL compute backend with the LBM kernels and the aero visualisation
// kernels. Requires a current OpenGL 4.6 core context (see
// GlComputeBackend::create).
[[nodiscard]] core::Result<std::unique_ptr<GlComputeBackend>, std::string>
make_aero_compute_backend();

} // namespace orrery::render
