#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <numbers>

#include "orrery/aero/panel_method.h"
#include "orrery/geom/naca.h"

using namespace orrery::aero;

namespace {

constexpr double pi = std::numbers::pi;
constexpr double degree = pi / 180.0;

// Reference table for the NACA 0012 section, angle of attack 0 to 8 degrees.
//
// No XFOIL run is embedded: the reference values are analytic, so they are
// exact for their assumptions and can be recomputed by anyone.
//  - thin-airfoil theory: Cl = 2 pi alpha (Anderson, *Fundamentals of
//    Aerodynamics*, ch. 4; Abbott and von Doenhoff, *Theory of
//    Wing Sections*, 1959, ch. 4);
//  - inviscid thick symmetric section: Cl = 2 pi (1 + 0.77 t/c) alpha, the
//    lift slope of the symmetric Joukowski airfoil, 2 pi (1 + eps) with
//    thickness t/c = (3 sqrt(3) / 4) eps = 1.299 eps (Joukowski
//    transformation: Abbott and von Doenhoff, ch. 3; Katz and Plotkin,
//    *Low-Speed Aerodynamics*, ch. 6).
//    A potential-flow panel method should approach this value, not 2 pi.
// Measured sections lift less than both because of the boundary layer: the
// NACA 0012 wind-tunnel slope at Re = 3e6 to 9e6 is about 0.105 to 0.11 per
// degree, 6.0 to 6.3 per radian (Abbott and von Doenhoff, appendix IV;
// Ladson, NASA TM-4074, 1988). The project criterion (docs/physics.md,
// section 8 of the plan) is 2 pi per radian within 10 percent.
constexpr double thickness = 0.12;
constexpr double thin_slope = 2.0 * pi;
constexpr double thick_slope = 2.0 * pi * (1.0 + 0.77 * thickness);

struct ReferenceRow {
    double alpha_deg;
    double cl_thin;
    double cl_thick;
};
constexpr std::array<ReferenceRow, 5> reference_table{{
    {0.0, 0.0, 0.0},
    {2.0, thin_slope * 2.0 * degree, thick_slope * 2.0 * degree},
    {4.0, thin_slope * 4.0 * degree, thick_slope * 4.0 * degree},
    {6.0, thin_slope * 6.0 * degree, thick_slope * 6.0 * degree},
    {8.0, thin_slope * 8.0 * degree, thick_slope * 8.0 * degree},
}};

} // namespace

TEST_CASE("NACA 0012 lift slope is 2 pi per radian within 10 percent", "[aero_validation][panel]") {
    const auto airfoil = *orrery::geom::Naca4::from_designation("0012");
    PanelSolver2D solver;
    double sxx = 0.0;
    double sxy = 0.0;
    for (const ReferenceRow& row : reference_table) {
        FlowConditions c;
        c.speed = 50.0;
        c.angle_of_attack = row.alpha_deg * degree;
        solver.setup(naca_section_geometry(airfoil, 1.0, 100), c);
        REQUIRE(solver.step());
        const AeroResult& r = solver.result();
        std::cout << "NACA 0012 alpha " << row.alpha_deg << " deg: Cl " << r.cl << " (thin "
                  << row.cl_thin << ", Joukowski-thick " << row.cl_thick << "), Cm_c/4 " << r.cm
                  << ", Cd_p " << r.cd << ", Cd_f " << r.cd_friction << '\n';
        CHECK(r.warnings.empty());
        if (row.alpha_deg == 0.0) {
            CHECK(std::abs(r.cl) < 1e-10);
        } else {
            CHECK(std::abs(r.cl - row.cl_thick) < 0.03 * row.cl_thick);
        }
        sxx += c.angle_of_attack * c.angle_of_attack;
        sxy += c.angle_of_attack * r.cl;
    }
    // Least-squares slope through the origin.
    const double slope = sxy / sxx;
    std::cout << "NACA 0012 lift slope " << slope << " per rad, " << slope / thin_slope
              << " x 2 pi, " << slope / thick_slope << " x Joukowski thick-section slope\n";
    CHECK(std::abs(slope - thin_slope) < 0.10 * thin_slope);
    CHECK(std::abs(slope - thick_slope) < 0.02 * thick_slope);
}
