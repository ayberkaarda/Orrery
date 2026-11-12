#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <stdexcept>

#include "orrery/aero/newtonian.h"
#include "orrery/geom/primitives.h"

using namespace orrery::aero;
using orrery::geom::Vector3;

namespace {

constexpr double pi = std::numbers::pi;
constexpr double degree = pi / 180.0;

FlowConditions hypersonic(double mach, double alpha_deg) {
    FlowConditions c;
    c.temperature = 220.0;
    c.density = 1e-4;
    c.speed = mach * c.speed_of_sound();
    c.angle_of_attack = alpha_deg * degree;
    return c;
}

AeroResult solve(const AeroGeometry& g, const FlowConditions& c, NewtonianOptions o = {}) {
    NewtonianSolver solver(o);
    solver.setup(g, c);
    REQUIRE(solver.step());
    return solver.result();
}

} // namespace

TEST_CASE("Rayleigh pitot Cp_max", "[aero][newtonian]") {
    // Limit for gamma = 1.4: ((g+1)^2 / (4 g))^(g/(g-1)) * 4 / (g+1) = 1.8394.
    const double limit = std::pow(2.4 * 2.4 / 5.6, 3.5) * 4.0 / 2.4;
    CHECK(std::abs(limit - 1.8394) < 1e-4);
    CHECK(std::abs(newtonian_cp_max(1e5, 1.4) - limit) < 1e-8);
    const double m20 = newtonian_cp_max(20.0, 1.4);
    CHECK(m20 < limit);
    CHECK(m20 > 1.83);
    // Normal-shock and isentropic branches meet at Mach 1: 2/1.4 (1.2^3.5 - 1).
    const double at_one = 2.0 / 1.4 * (std::pow(1.2, 3.5) - 1.0);
    CHECK(std::abs(newtonian_cp_max(1.0, 1.4) - at_one) < 1e-14);
    CHECK(std::abs(newtonian_cp_max(1.0 - 1e-9, 1.4) - at_one) < 1e-8);
    // Incompressible limit: Cp_max -> 1.
    CHECK(std::abs(newtonian_cp_max(1e-3, 1.4) - 1.0) < 1e-6);
    // A configurable gamma changes it (gamma = 1.2: about 1.91 at Mach 20).
    CHECK(newtonian_cp_max(20.0, 1.2) > m20);
    CHECK_THROWS_AS(newtonian_cp_max(0.0, 1.4), std::invalid_argument);
    CHECK_THROWS_AS(newtonian_cp_max(5.0, 1.0), std::invalid_argument);
}

TEST_CASE("Newtonian sphere: Cd = Cp_max / 2", "[aero][newtonian]") {
    AeroGeometry g;
    g.mesh = orrery::geom::sphere_profile(1.0).mesh({128, 0.02});
    g.reference = {pi, 2.0, {}};
    const FlowConditions c = hypersonic(10.0, 0.0);
    const AeroResult r = solve(g, c);
    const double cp_max = newtonian_cp_max(c.mach(), c.gamma);
    INFO("Cd " << r.cd << " expected " << 0.5 * cp_max);
    CHECK(std::abs(r.cd - 0.5 * cp_max) < 2e-3 * cp_max);
    CHECK(std::abs(r.cl) < 1e-12);
    CHECK(std::abs(r.cm) < 1e-12);
    CHECK(r.warnings.empty());
    // Cp within [0, Cp_max] everywhere, zero on the lee side.
    for (std::size_t i = 0; i < r.surface.cp.size(); ++i) {
        CHECK(r.surface.cp[i] >= 0.0);
        CHECK(r.surface.cp[i] <= cp_max);
        if (r.surface.normals[i].x > 0.0) {
            CHECK(r.surface.cp[i] == 0.0);
        }
    }
}

TEST_CASE("Newtonian flat plate: normal force Cp_max sin^2(alpha)", "[aero][newtonian]") {
    AeroGeometry g;
    g.mesh = orrery::geom::flat_plate_mesh(2.0, 3.0, 1e-9);
    g.reference = {6.0, 2.0, {}};
    for (const double alpha : {90.0, 30.0, 10.0}) {
        const FlowConditions c = hypersonic(8.0, alpha);
        const AeroResult r = solve(g, c);
        const double cp_max = newtonian_cp_max(c.mach(), c.gamma);
        const double a = alpha * degree;
        const double cn = cp_max * std::sin(a) * std::sin(a);
        INFO("alpha " << alpha);
        CHECK(std::abs(r.cd - cn * std::sin(a)) < 1e-8);
        CHECK(std::abs(r.cl - cn * std::cos(a)) < 1e-8);
    }
}

TEST_CASE("Sutton-Graves stagnation heat flux", "[aero][newtonian]") {
    // 1.7415e-4 * sqrt(1e-4 / 1) * 7000^3 = 5.973e5 W/m^2.
    CHECK(std::abs(sutton_graves_heat_flux(1e-4, 7000.0, 1.0) - 5.97335e5) < 1.0);
    CHECK(sutton_graves_heat_flux(1e-4, 7000.0, 4.0) < sutton_graves_heat_flux(1e-4, 7000.0, 1.0));
    CHECK_THROWS_AS(sutton_graves_heat_flux(1e-4, 7000.0, 0.0), std::invalid_argument);

    AeroGeometry g;
    g.mesh = orrery::geom::sphere_profile(0.5).mesh({32, 0.1});
    g.reference = {pi * 0.25, 1.0, {}};
    const FlowConditions c = hypersonic(25.0, 0.0);
    CHECK_FALSE(solve(g, c).stagnation_heat_flux.has_value());
    NewtonianOptions o;
    o.nose_radius = 0.5;
    const AeroResult r = solve(g, c, o);
    REQUIRE(r.stagnation_heat_flux.has_value());
    CHECK(std::abs(*r.stagnation_heat_flux - sutton_graves_heat_flux(c.density, c.speed, 0.5)) <
          1e-6);
}

TEST_CASE("Newtonian warns below Mach 5 but still computes", "[aero][newtonian]") {
    AeroGeometry g;
    g.mesh = orrery::geom::sphere_profile(1.0).mesh({32, 0.1});
    g.reference = {pi, 2.0, {}};
    const AeroResult r = solve(g, hypersonic(3.0, 0.0));
    CHECK(r.has_warning("newtonian.mach"));
    CHECK(r.cd > 0.5);
    CHECK_FALSE(solve(g, hypersonic(6.0, 0.0)).has_warning("newtonian.mach"));
}

TEST_CASE("Newtonian rejects unusable input", "[aero][newtonian]") {
    NewtonianSolver solver;
    CHECK_THROWS_AS(solver.step(), std::logic_error);
    CHECK_THROWS_AS(solver.setup(AeroGeometry{}, hypersonic(10.0, 0.0)), std::invalid_argument);
    AeroGeometry g;
    g.mesh = orrery::geom::cube_mesh(1.0);
    g.mesh.triangles[0][0] = 1000;
    CHECK_THROWS_AS(solver.setup(g, hypersonic(10.0, 0.0)), std::invalid_argument);
}
