#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "orrery/aero/aero_solver.h"
#include "orrery/geom/naca.h"
#include "orrery/geom/panel_mesh.h"

namespace orrery::aero {

// Potential-flow panel methods (Hess and Smith 1967; Katz and Plotkin,
// *Low-Speed Aerodynamics*, 2nd ed., 2001).
//
// APPROXIMATION (A5): inviscid, incompressible, attached potential flow with
// a fixed, flat wake. There is no boundary layer, no separation and no stall;
// drag is the pressure integral (induced drag in 3D, zero in 2D up to the
// discretisation error). Valid for thin bodies at low angle of attack, Mach
// <= 0.3 (regime.h); outside, the result carries a warning.

struct PanelOptions {
    // 3D wake: fixed strips along the free stream from the trailing edge,
    // `wake_length` reference lengths long, in `wake_rows` rows.
    double wake_length = 50.0;
    int wake_rows = 1;
    // Dense LU up to this many unknowns, restarted GMRES above.
    std::size_t dense_limit = 4000;
    double gmres_tolerance = 1e-10;
    int gmres_restart = 100;
    int gmres_max_iterations = 2000;
};

// Flat-plate skin-friction coefficient (average over a plate of length L at
// Re_L), after Schlichting (*Boundary-Layer Theory*): laminar Blasius
// 1.328 / sqrt(Re) up to Re = 5e5, above it the Prandtl-Schlichting turbulent
// law with the laminar-run correction, 0.455 / (log10 Re)^2.58 - 1700 / Re.
// APPROXIMATION (A6): flat plate at zero pressure gradient, transition at
// Re = 5e5; curvature, pressure gradients and separation are ignored.
// Throws std::invalid_argument for Re <= 0 or not finite.
[[nodiscard]] double flat_plate_skin_friction(double reynolds);

// Section geometry for the 2D solver: `airfoil` scaled to `chord` with
// 2 * stations panels (geom::Naca4::section), reference area chord * 1 m,
// reference length chord, moments about the quarter chord.
[[nodiscard]] AeroGeometry naca_section_geometry(const geom::Naca4& airfoil, double chord,
                                                 int stations);

// 2D Hess-Smith panel method: constant-strength source on each straight
// panel of AeroGeometry::section plus one vorticity strength common to all
// panels, set by the Kutta condition (equal tangential speed on the two
// panels at the trailing edge, points 0 and n-1 of the section). Sideslip is
// ignored. Per unit span: force in N/m, moment in N m/m, areas in m.
class PanelSolver2D final : public IAeroSolver {
public:
    explicit PanelSolver2D(PanelOptions options = {}) : options_(options) {}

    [[nodiscard]] SolverKind kind() const noexcept override { return SolverKind::panel; }
    void setup(const AeroGeometry& geometry, const FlowConditions& conditions) override;
    bool step() override;
    [[nodiscard]] const AeroResult& result() const noexcept override { return result_; }

    // Circulation (clockwise positive, m^2/s per unit span) of the last solve:
    // the Kutta-Joukowski lift is rho V circulation.
    [[nodiscard]] double circulation() const noexcept { return circulation_; }

private:
    PanelOptions options_;
    std::vector<geom::SectionPoint> section_;
    ReferenceValues reference_;
    FlowConditions conditions_;
    bool ready_ = false;
    AeroResult result_;
    double circulation_ = 0.0;
};

// 3D source + doublet panel method with the Dirichlet (zero internal
// perturbation potential) boundary condition (Katz and Plotkin, ch. 12):
// constant-strength source sigma = -n . V_inf and unknown constant doublet
// mu on every triangle, constant doublet wake strips shed from
// AeroGeometry::trailing_edge whose strength is mu_upper - mu_lower (Kutta
// condition). A body without a trailing edge carries no lift. Surface
// velocity is V_inf tangential plus the surface gradient of mu (least
// squares over neighbouring panels that do not cross the trailing edge);
// induced drag from the Trefftz plane.
class PanelSolver3D final : public IAeroSolver {
public:
    explicit PanelSolver3D(PanelOptions options = {}) : options_(options) {}

    [[nodiscard]] SolverKind kind() const noexcept override { return SolverKind::panel; }
    void setup(const AeroGeometry& geometry, const FlowConditions& conditions) override;
    bool step() override;
    [[nodiscard]] const AeroResult& result() const noexcept override { return result_; }

    // Panels and wake of the last setup(), and the doublet strengths of the
    // last solve (m^2/s, one per panel).
    [[nodiscard]] const geom::PanelMesh& panels() const noexcept { return panels_; }
    [[nodiscard]] const std::vector<double>& doublets() const noexcept { return doublets_; }
    // Lift coefficient from the Trefftz plane (rho V integral of the wake
    // doublet strength over the span), for cross-checking cl.
    [[nodiscard]] double trefftz_cl() const noexcept { return trefftz_cl_; }
    // True if the last solve used GMRES rather than dense LU.
    [[nodiscard]] bool used_gmres() const noexcept { return used_gmres_; }

private:
    PanelOptions options_;
    geom::TriangleMesh mesh_;
    std::vector<std::uint32_t> trailing_edge_;
    geom::PanelMesh panels_;
    ReferenceValues reference_;
    FlowConditions conditions_;
    bool ready_ = false;
    AeroResult result_;
    std::vector<double> doublets_;
    double trefftz_cl_ = 0.0;
    bool used_gmres_ = false;
};

// Influence of constant-strength unit distributions on a flat triangle
// (vertices counter-clockwise about its unit normal n) at point p:
//   source:  -1/(4 pi) integral dS / r
//   doublet: +1/(4 pi) integral h / r^3 dS, h = (p - q) . n
// The doublet potential tends to +1/2 on the side n points to and -1/2 on
// the other. Exact closed forms (solid angle by van Oosterom and Strackee
// 1983; source by the edge-logarithm formula of Hess and Smith 1967).
struct PanelInfluence {
    double source = 0.0;
    double doublet = 0.0;
};

[[nodiscard]] PanelInfluence triangle_influence(const geom::Vector3& p, const geom::Vector3& a,
                                                const geom::Vector3& b, const geom::Vector3& c);

} // namespace orrery::aero
