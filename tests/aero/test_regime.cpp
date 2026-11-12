#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>
#include <stdexcept>

#include "orrery/aero/aero_result.h"
#include "orrery/aero/flow_conditions.h"
#include "orrery/aero/regime.h"

using namespace orrery::aero;

namespace {

// Sea-level air (US Standard Atmosphere 1976).
FlowConditions air_at_mach(double mach) {
    FlowConditions c;
    c.speed = mach * c.speed_of_sound();
    return c;
}

constexpr double degree = std::numbers::pi / 180.0;

} // namespace

TEST_CASE("flow conditions derive Mach, Reynolds and dynamic pressure", "[aero][conditions]") {
    FlowConditions c;
    c.speed = 34.0;
    // sqrt(1.4 * 287.05 * 288.15) = 340.29 m/s at sea level.
    CHECK(std::abs(c.speed_of_sound() - std::sqrt(1.4 * 287.05 * 288.15)) < 1e-12);
    CHECK(std::abs(c.speed_of_sound() - 340.29) < 0.01);
    CHECK(std::abs(c.mach() - 34.0 / c.speed_of_sound()) < 1e-15);
    CHECK(std::abs(c.reynolds(2.0) - 1.225 * 34.0 * 2.0 / 1.789e-5) < 1e-6);
    CHECK(std::abs(c.dynamic_pressure() - 0.5 * 1.225 * 34.0 * 34.0) < 1e-12);
}

TEST_CASE("body axes follow angle of attack and sideslip", "[aero][conditions]") {
    FlowConditions c;
    c.speed = 10.0;
    c.angle_of_attack = 5.0 * degree;
    c.sideslip = 3.0 * degree;
    const auto d = c.drag_direction();
    const auto l = c.lift_direction();
    const auto s = c.side_direction();
    CHECK(std::abs(orrery::geom::norm(d) - 1.0) < 1e-15);
    CHECK(std::abs(orrery::geom::norm(l) - 1.0) < 1e-15);
    CHECK(std::abs(orrery::geom::norm(s) - 1.0) < 1e-15);
    CHECK(std::abs(orrery::geom::dot(d, l)) < 1e-15);
    CHECK(std::abs(orrery::geom::dot(d, s)) < 1e-15);
    CHECK(std::abs(orrery::geom::dot(l, s)) < 1e-15);
    // Positive angle of attack: the free stream rises, lift leans back.
    CHECK(d.z > 0.0);
    CHECK(l.x < 0.0);
    CHECK(l.y == 0.0);
    CHECK(s.y > 0.9);
    CHECK(std::abs(c.velocity().z - 10.0 * d.z) < 1e-14);
}

TEST_CASE("coefficients come from force and moment", "[aero][result]") {
    FlowConditions c;
    c.speed = 20.0;
    c.angle_of_attack = 4.0 * degree;
    const ReferenceValues ref{2.0, 0.5, {}};
    AeroResult r;
    const double q = c.dynamic_pressure();
    r.force = c.drag_direction() * (0.1 * q * 2.0) + c.lift_direction() * (0.8 * q * 2.0);
    r.moment = {0.0, -0.05 * q * 2.0 * 0.5, 0.0};
    set_coefficients(r, c, ref);
    CHECK(std::abs(r.cd - 0.1) < 1e-14);
    CHECK(std::abs(r.cl - 0.8) < 1e-14);
    CHECK(std::abs(r.cy) < 1e-14);
    CHECK(std::abs(r.cm + 0.05) < 1e-14);

    CHECK_THROWS_AS(set_coefficients(r, c, ReferenceValues{0.0, 1.0, {}}), std::invalid_argument);
    c.speed = 0.0;
    CHECK_THROWS_AS(set_coefficients(r, c, ref), std::invalid_argument);
}

TEST_CASE("regime selector recommends a solver from Mach and Reynolds", "[aero][regime]") {
    // Low-speed wing: incompressible, Re in the millions.
    FlowConditions wing = air_at_mach(0.15);
    RegimeChoice choice = select_solver(wing, 1.0);
    CHECK(choice.recommended == SolverKind::panel);
    CHECK(choice.solver == SolverKind::panel);
    CHECK_FALSE(choice.overridden);
    CHECK(choice.within_envelope);
    CHECK(choice.warnings.empty());

    // Small, slow body: viscous regime.
    FlowConditions small = air_at_mach(0.01);
    choice = select_solver(small, 0.01);
    CHECK(small.reynolds(0.01) < 1e5);
    CHECK(choice.recommended == SolverKind::lattice_boltzmann);
    CHECK(choice.within_envelope);

    // Entry capsule.
    FlowConditions entry = air_at_mach(20.0);
    choice = select_solver(entry, 3.9);
    CHECK(choice.recommended == SolverKind::newtonian);
    CHECK(choice.within_envelope);

    // Transonic and low supersonic: nothing is valid, the closest is chosen
    // and flagged.
    choice = select_solver(air_at_mach(0.8), 1.0);
    CHECK(choice.recommended == SolverKind::panel);
    CHECK_FALSE(choice.within_envelope);
    CHECK(choice.rationale.find("no solver is valid") != std::string::npos);
    choice = select_solver(air_at_mach(2.0), 1.0);
    CHECK(choice.recommended == SolverKind::newtonian);
    CHECK_FALSE(choice.within_envelope);
}

TEST_CASE("a user override is honoured and warned about, never blocked", "[aero][regime]") {
    const FlowConditions wing = air_at_mach(0.15);
    const RegimeChoice choice = select_solver(wing, 1.0, SolverKind::newtonian);
    CHECK(choice.recommended == SolverKind::panel);
    CHECK(choice.solver == SolverKind::newtonian);
    CHECK(choice.overridden);
    CHECK_FALSE(choice.within_envelope);
    REQUIRE(choice.warnings.size() == 1);
    CHECK(choice.warnings[0].code == "newtonian.mach");
    CHECK(choice.warnings[0].message.find("Mach number") != std::string::npos);

    // Overriding with the recommendation itself is not an override.
    CHECK_FALSE(select_solver(wing, 1.0, SolverKind::panel).overridden);
}

TEST_CASE("envelope checks name every violated bound", "[aero][regime]") {
    FlowConditions c = air_at_mach(0.5);
    c.angle_of_attack = 15.0 * degree;
    c.sideslip = -12.0 * degree;
    c.viscosity = 1.0; // Re far below 1e5
    const std::vector<ValidityWarning> w = check_validity(SolverKind::panel, c, 1.0);
    AeroResult r;
    r.warnings = w;
    CHECK(w.size() == 4);
    CHECK(r.has_warning("panel.mach"));
    CHECK(r.has_warning("panel.reynolds"));
    CHECK(r.has_warning("panel.angle_of_attack"));
    CHECK(r.has_warning("panel.sideslip"));
    CHECK_FALSE(r.has_warning("newtonian.mach"));

    // Bounds are inclusive.
    FlowConditions edge = air_at_mach(5.0);
    CHECK(check_validity(SolverKind::newtonian, edge, 1.0).empty());
    edge.speed *= 0.999;
    CHECK(check_validity(SolverKind::newtonian, edge, 1.0).size() == 1);

    // A custom envelope (any solver can define its own) uses the same codes.
    const ValidityEnvelope custom{0.0, 0.1, 0.0, 1e9, 0.5};
    const auto cw = check_envelope(custom, "lbm2d", air_at_mach(0.2), 1.0);
    REQUIRE(cw.size() == 1);
    CHECK(cw[0].code == "lbm2d.mach");
    CHECK(solver_name(SolverKind::lattice_boltzmann) == "lbm");
}
