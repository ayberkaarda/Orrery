#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <iostream>
#include <numbers>

#include "orrery/aero/panel_method.h"
#include "orrery/geom/naca.h"

using namespace orrery::aero;

namespace {

constexpr double pi = std::numbers::pi;
constexpr double degree = pi / 180.0;

} // namespace

// Rectangular NACA 0012 wing of aspect ratio 6 at 5 degrees.
// Reference: Helmbold's lifting-surface equation for straight wings,
// a = a0 / (sqrt(1 + (a0 / (pi AR))^2) + a0 / (pi AR)) (Anderson,
// *Fundamentals of Aerodynamics*, ch. 5), here with the 2D slope a0 of the
// thick section from the 2D panel method itself, and the induced drag
// CDi = CL^2 / (pi AR e) with a span efficiency e between 0.85 and 1 (1 is
// the elliptic optimum; rectangular wings of this aspect ratio have about
// 0.9, Glauert).
TEST_CASE("3D panel method: rectangular wing lift and induced drag", "[aero_validation][panel]") {
    const auto airfoil = *orrery::geom::Naca4::from_designation("0012");
    const double chord = 1.0;
    const double span = 6.0;
    const double aspect = span / chord;
    const orrery::geom::NacaWing wing = orrery::geom::naca_wing(airfoil, chord, span, 16, 24);

    AeroGeometry g;
    g.mesh = wing.mesh;
    g.trailing_edge = wing.trailing_edge;
    g.reference = {chord * span, chord, {0.25 * chord, 0.0, 0.0}};
    FlowConditions c;
    c.speed = 50.0;
    c.angle_of_attack = 5.0 * degree;

    PanelSolver3D solver;
    solver.setup(g, c);
    REQUIRE(solver.step());
    const AeroResult& r = solver.result();

    // 2D slope of the same section from the 2D solver.
    PanelSolver2D section;
    section.setup(naca_section_geometry(airfoil, chord, 100), c);
    section.step();
    const double a0 = section.result().cl / c.angle_of_attack;
    const double k = a0 / (pi * aspect);
    const double helmbold = a0 / (std::sqrt(1.0 + k * k) + k);
    const double cl_ref = helmbold * c.angle_of_attack;
    const double e = r.cl * r.cl / (pi * aspect * r.cd_induced);
    std::cout << "wing AR 6, alpha 5 deg, " << solver.panels().panels.size() << " panels: CL "
              << r.cl << " (Helmbold " << cl_ref << "), Trefftz CL " << solver.trefftz_cl()
              << ", CDi " << r.cd_induced << " (e = " << e << "), CD pressure " << r.cd << ", Cm "
              << r.cm << ", residual " << r.residuals.front() << '\n';
    CHECK(r.converged);
    CHECK(r.warnings.empty());
    CHECK(std::abs(r.cl - cl_ref) < 0.10 * cl_ref);
    CHECK(std::abs(solver.trefftz_cl() - r.cl) < 0.02 * r.cl);
    CHECK(e > 0.85);
    CHECK(e < 1.05);
    // The diagonal split of the quads is not mirror-symmetric in y.
    CHECK(std::abs(r.cy) < 1e-3);

    // Zero incidence: no lift. The induced drag is not exactly zero: the
    // triangulation is not symmetric between upper and lower surface, which
    // leaves a small antisymmetric circulation at the tips (docs/aero.md).
    c.angle_of_attack = 0.0;
    solver.setup(g, c);
    solver.step();
    std::cout << "wing AR 6, alpha 0: CL " << solver.result().cl << ", CDi "
              << solver.result().cd_induced << '\n';
    CHECK(std::abs(solver.result().cl) < 1e-4);
    CHECK(std::abs(solver.result().cd_induced) < 1e-3);
}
