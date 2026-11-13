#pragma once

#include <optional>
#include <string>
#include <vector>

#include "orrery/aero/aero_result.h"
#include "orrery/app/aero_layers.h"
#include "orrery/core/result.h"
#include "orrery/render/aero_field.h"
#include "orrery/render/aero_surface.h"
#include "orrery/render/aero_view.h"
#include "orrery/render/camera.h"
#include "orrery/sim/aero_geometry.h"
#include "orrery/sim/aero_view_data.h"

// The render side of the Aero view, shared by the window and the headless
// renderer (docs/aero-workspace.md, "View frame"): the plain data of
// render::AeroView built from a prepared geometry, a run's view data and its
// result, all in the view frame of orrery/sim/aero_view_data.h.
namespace orrery::app {

// Span of the extruded body that shows a 2D section, in chords.
inline constexpr double kSectionViewSpan = 1.0;

// The body to draw: the mesh in the view frame and Cp per triangle (empty:
// neutral colour). A 2D section is extruded along the view's z axis.
[[nodiscard]] core::Result<render::AeroSurface, std::string>
view_surface(const sim::PreparedGeometry& geometry, const std::vector<double>& cp,
             const sim::ViewFrame& frame);

// The lattice flow in the view frame.
[[nodiscard]] render::FlowField view_flow(const sim::FlowSnapshot& snapshot);

// Drag, lift and side-force arrows from the moment centre.
[[nodiscard]] render::AeroForces view_forces(const aero::AeroResult& result,
                                             const aero::FlowConditions& conditions,
                                             const sim::PreparedGeometry& geometry,
                                             const sim::ViewFrame& frame);

// View options for the layers: 2D runs cut the field along z (the section
// plane), 3D runs along y through the body; streamlines seed upstream of
// the body; force arrows scale with the body size. The Cp colour range is
// symmetric about zero and reaches the 98th percentile of |Cp| (panel
// singularities at tips and trailing edges would otherwise wash the map
// out), at least [-1, 1].
[[nodiscard]] render::AeroViewOptions view_options(const AeroLayers& layers, bool two_dimensional,
                                                   const geom::Bounds& body_bounds,
                                                   const std::optional<render::GridGeometry>& grid,
                                                   const std::vector<double>& cp);

// The symmetric Cp range described above.
[[nodiscard]] double cp_range(const std::vector<double>& cp);

// Camera for the view: a turntable around the body (3D) or looking down on
// the section plane (2D), framing the body and, with a flow field, its near
// wake.
[[nodiscard]] render::TurntableMode view_camera(const geom::Bounds& body_bounds,
                                                bool two_dimensional, bool with_wake, double aspect,
                                                const render::CameraLens& lens);

// Bounds of a mesh in the view frame.
[[nodiscard]] geom::Bounds view_bounds(const geom::TriangleMesh& mesh);

} // namespace orrery::app
