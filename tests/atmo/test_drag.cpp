#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include "orrery/atmo/drag.h"
#include "orrery/core/units.h"

using orrery::atmo::AeroCoefficientTable;
using orrery::atmo::AtmosphereFrame;
using orrery::atmo::AtmosphericDrag;
using orrery::atmo::DragForce;
using orrery::atmo::Planet;
using orrery::core::Vec3;
namespace si = orrery::core::si;
namespace space = orrery::core::space;

namespace {

// A planet with a uniform-density atmosphere near the surface (exponential
// with a huge scale height) so the expected force is a closed form.
Planet test_planet() {
    Planet p = Planet::earth();
    orrery::atmo::ExponentialAtmosphereParameters params;
    params.surface_density = si::Density(1.2);
    params.scale_height = si::Length(1.0e12);
    params.temperature_k = 300.0;
    params.gas.gas_constant_j_kg_k = 300.0;
    params.gas.gamma = 1.4;
    auto atmosphere = orrery::atmo::ExponentialAtmosphere::create(params);
    REQUIRE(atmosphere);
    p.atmosphere = std::make_shared<const orrery::atmo::ExponentialAtmosphere>(atmosphere.value());
    return p;
}

AeroCoefficientTable table() {
    // Cd 1 at Mach 0, 2 at Mach 1 (speed of sound sqrt(1.4 * 300 * 300) = 355 m/s).
    auto t = AeroCoefficientTable::create({{0.0, 1.0}, {1.0, 2.0}}, si::Area(3.0));
    REQUIRE(t);
    return t.value();
}

Vec3<si::Length> surface_point(const Planet& p) {
    return {p.radius + si::Length(10.0), si::Length(0.0), si::Length(0.0)};
}

} // namespace

TEST_CASE("Drag force F = 1/2 rho v^2 Cd(Mach) A against the relative wind", "[atmo][drag]") {
    const Planet planet = test_planet();
    const AeroCoefficientTable cd = table();
    const double a = std::sqrt(1.4 * 300.0 * 300.0);
    const double v = 0.5 * a;
    const DragForce f = orrery::atmo::drag_force(
        planet, cd, surface_point(planet), {si::Velocity(0.0), si::Velocity(v), si::Velocity(0.0)},
        AtmosphereFrame::inertial);
    CHECK(std::abs(f.mach - 0.5) < 1e-12);
    CHECK(std::abs(f.cd - 1.5) < 1e-12);
    CHECK_FALSE(f.cd_clamped);
    const double expected = 0.5 * 1.2 * v * v * 1.5 * 3.0;
    CHECK(std::abs(f.dynamic_pressure.value() / (0.5 * 1.2 * v * v) - 1.0) < 1e-10);
    CHECK(std::abs(f.force.y.value() + expected) < 1e-9 * expected);
    CHECK(f.force.x.value() == 0.0);
    CHECK(f.force.z.value() == 0.0);
    CHECK(std::abs(f.altitude.value() - 10.0) < 1e-6);
}

TEST_CASE("Co-rotating atmosphere: the wind is omega x r", "[atmo][drag]") {
    const Planet planet = test_planet();
    const Vec3<si::Length> r = surface_point(planet);
    const double omega = planet.rotation_rate.value();
    const Vec3<si::Velocity> v{si::Velocity(0.0), si::Velocity(omega * r.x.value()),
                               si::Velocity(0.0)};
    const auto relative = orrery::atmo::air_relative_velocity(r, v, planet.rotation_rate,
                                                              AtmosphereFrame::co_rotating);
    CHECK(std::abs(orrery::core::norm(relative).value()) < 1e-9);
    CHECK(orrery::atmo::air_relative_velocity(r, v, planet.rotation_rate,
                                              AtmosphereFrame::inertial) == v);
    // At rest with the surface: no drag in a co-rotating atmosphere.
    const DragForce f =
        orrery::atmo::drag_force(planet, table(), r, v, AtmosphereFrame::co_rotating);
    CHECK(orrery::core::norm(f.force).value() == 0.0);
    CHECK(orrery::atmo::to_string(AtmosphereFrame::co_rotating) == "co_rotating");
}

TEST_CASE("No drag above the atmosphere", "[atmo][drag]") {
    const Planet earth = Planet::earth();
    const Vec3<si::Length> r{earth.radius + si::Length(2.0e6), si::Length(0.0), si::Length(0.0)};
    const DragForce f = orrery::atmo::drag_force(
        earth, table(), r, {si::Velocity(0.0), si::Velocity(7000.0), si::Velocity(0.0)},
        AtmosphereFrame::inertial);
    CHECK_FALSE(f.atmosphere.inside);
    CHECK(orrery::core::norm(f.force).value() == 0.0);
}

TEST_CASE("AtmosphericDrag as a core force model on probes", "[atmo][drag]") {
    const Planet planet = test_planet();
    orrery::core::BodyStore bodies;
    const auto planet_id =
        bodies.add({"earth", orrery::core::Planet{}, space::Mass(3.0e-6), {}, {}});
    REQUIRE(planet_id);
    const double v = 100.0;
    const auto probe = bodies.add(
        {"probe",
         orrery::core::Probe{orrery::core::AeroRef{0}},
         orrery::core::to_space(si::Mass(1000.0)),
         {orrery::core::to_space(surface_point(planet).x), space::Length(0.0), space::Length(0.0)},
         {space::Velocity(0.0), orrery::core::to_space(si::Velocity(v)), space::Velocity(0.0)}});
    REQUIRE(probe);
    auto drag = AtmosphericDrag::create(planet, planet_id.value(), {table()});
    REQUIRE(drag);
    const AtmosphericDrag& model = drag.value();
    CHECK(model.name() == "atmospheric_drag");

    std::vector<Vec3<space::Length>> positions{bodies.position(0), bodies.position(1)};
    std::vector<Vec3<space::Acceleration>> out(2);
    model.accelerations(bodies, positions, out);
    CHECK(out[0] == Vec3<space::Acceleration>{});
    const DragForce f = model.evaluate(orrery::core::AeroRef{0}, surface_point(planet),
                                       {si::Velocity(0.0), si::Velocity(v), si::Velocity(0.0)});
    const double expected = f.force.y.value() / 1000.0;
    CHECK(std::abs(orrery::core::to_si(out[1].y).value() / expected - 1.0) < 1e-12);
    CHECK(model.potential_energy(bodies, positions).value() == 0.0);

    // Contract violations throw and leave `out` unchanged.
    std::vector<Vec3<space::Acceleration>> short_out(1);
    CHECK_THROWS_AS(model.accelerations(bodies, positions, short_out), std::invalid_argument);
    auto no_table = AtmosphericDrag::create(planet, planet_id.value(), {});
    REQUIRE(no_table);
    out[1] = Vec3<space::Acceleration>{};
    CHECK_THROWS_AS(no_table.value().accelerations(bodies, positions, out), std::invalid_argument);
    CHECK(out[1] == Vec3<space::Acceleration>{});
    Planet bare = planet;
    bare.atmosphere.reset();
    CHECK_FALSE(AtmosphericDrag::create(bare, planet_id.value(), {table()}));
}
