#pragma once

#include <cstdint>
#include <variant>
#include <vector>

#include "orrery/aero/aero_result.h"
#include "orrery/aero/aero_solver.h"
#include "orrery/aero/flow_conditions.h"
#include "orrery/aero/lbm/lbm2d.h"
#include "orrery/aero/lbm3d/lbm3d.h"
#include "orrery/core/result.h"
#include "orrery/geom/naca.h"
#include "orrery/geom/triangle_mesh.h"
#include "orrery/sim/aero_geometry.h"

// What the Aero view shows of a run, prepared on the solver thread
// (docs/aero-workspace.md, "View frame"): surface pressure per drawn element
// and a snapshot of the lattice Boltzmann flow field, placed in the view
// frame. Graphics-free; orrery_app turns it into the render API's types.
//
// View frame: the wind frame of the free stream, so the flow always runs
// along +x and the body shows its angle of attack, as in a wind tunnel.
//   3D   (drag, side, lift) directions of FlowConditions: x downstream,
//        y to the side, z up (the body axes at zero angles).
//   2D   the section plane is the view's x-y plane: x downstream, y along
//        the lift direction, z = -(body span axis). This is the plane of the
//        2D lattice in the render API (flow_field_from(MacroFields2D)), and a
//        camera looking down -z sees the classic section picture.
namespace orrery::sim {

struct ViewFrame {
    bool two_dimensional = false;
    geom::Vector3 drag{1.0, 0.0, 0.0};
    geom::Vector3 side{0.0, 1.0, 0.0};
    geom::Vector3 lift{0.0, 0.0, 1.0};
};

// The frame for these conditions (a 2D section ignores the sideslip).
[[nodiscard]] ViewFrame view_frame(const aero::FlowConditions& conditions, bool two_dimensional);

// A body-axes point or direction in the view frame.
[[nodiscard]] geom::Vector3 to_view(const geom::Vector3& body, const ViewFrame& frame) noexcept;

// A section point (body x, z) in the wind frame's (x, z) plane: the in-plane
// coordinates of the 2D view.
[[nodiscard]] geom::SectionPoint section_to_wind(const geom::SectionPoint& point,
                                                 const ViewFrame& frame) noexcept;

// Pressure coefficient per triangle of `mesh` (body axes). A solver that
// reports one sample per triangle (panel method, Newtonian) is copied; a
// volume solver's wall samples are assigned to the nearest triangle centre's
// nearest sample. Empty when the surface has no samples.
[[nodiscard]] std::vector<double> triangle_cp(const geom::TriangleMesh& mesh,
                                              const aero::SurfaceField& surface);

// Pressure coefficient per panel of a section (panel i from point i to
// i + 1, the last closing back to point 0): copied from a 2D panel result,
// nearest wall sample for the 2D lattice Boltzmann solver.
[[nodiscard]] std::vector<double> panel_cp(const std::vector<geom::SectionPoint>& section,
                                           const aero::SurfaceField& surface);

// A lattice Boltzmann flow field placed in the view frame: lattice node
// (i, j, k) sits at origin + spacing (i, j, k); a lattice velocity u is
// velocity_scale * u m/s; reference_speed is the inlet speed in lattice
// units (the pressure-coefficient reference).
struct FlowSnapshot {
    std::variant<aero::lbm::MacroFields2D, aero::lbm::MacroFields3D> fields;
    geom::Vector3 origin;
    double spacing = 1.0;
    double velocity_scale = 1.0;
    double reference_speed = 0.0;
    std::uint64_t step = 0;
};

// Reads the flow of a lattice Boltzmann solver after setup(). Fails for any
// other solver, or when the lattice cannot be read.
[[nodiscard]] core::Result<FlowSnapshot, std::string>
capture_flow(aero::IAeroSolver& solver, const PreparedGeometry& geometry,
             const aero::FlowConditions& conditions, const aero::ReferenceValues& reference);

} // namespace orrery::sim
