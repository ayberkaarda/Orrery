#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>
#include <variant>

#include "orrery/core/body.h"
#include "orrery/core/kepler.h"
#include "orrery/core/units.h"
#include "orrery/core/vec3.h"
#include "orrery/scene/scenario.h"
#include "scene_test_support.h"

namespace core = orrery::core;
namespace scene = orrery::scene;
using scene::test::document_with_bodies;
using scene::test::relative_error;
using scene::test::scenario_path;
using scene::test::sun_json;

TEST_CASE("Sun-Earth scenario loads into a SimState", "[scene][scenario]") {
    const auto loaded = scene::load_scene_file(scenario_path("sun_earth.json"));
    REQUIRE(loaded.has_value());
    const scene::Scene& s = loaded.value();
    CHECK(s.name == "Sun and Earth-Moon barycentre");
    CHECK(s.metadata.epoch == "J2000.0 (JD 2451545.0 TDB)");
    CHECK_FALSE(s.metadata.source.empty());
    CHECK(s.state.time().value() == 0.0);

    const core::BodyStore& bodies = s.state.bodies();
    REQUIRE(bodies.size() == 2);
    CHECK(bodies.name(0) == "Sun");
    CHECK(bodies.name(1) == "Earth-Moon");
    CHECK(core::tag_of(bodies.kind(0)) == core::BodyKindTag::star);
    CHECK(core::tag_of(bodies.kind(1)) == core::BodyKindTag::planet);
    CHECK(bodies.mass(1).value() == 3.0404326e-6);

    // The heliocentric distance lies between perihelion and aphelion.
    const double a = 1.00000261;
    const double e = 0.01671123;
    const double r = core::norm(bodies.position(1) - bodies.position(0)).value();
    CHECK(r > a * (1.0 - e));
    CHECK(r < a * (1.0 + e));
    // Ecliptic orbit: no out-of-plane motion.
    CHECK(bodies.position(1).z.value() == 0.0);
    CHECK(bodies.velocity(1).z.value() == 0.0);
}

TEST_CASE("Keplerian elements are resolved relative to the parent chain", "[scene][scenario]") {
    const auto loaded = scene::load_scene_file(scenario_path("sun_earth_moon.json"));
    REQUIRE(loaded.has_value());
    const core::BodyStore& bodies = loaded.value().state.bodies();
    REQUIRE(bodies.size() == 3);
    CHECK(bodies.name(2) == "Moon");
    CHECK(core::tag_of(bodies.kind(2)) == core::BodyKindTag::moon);

    // Recover the Moon's elements from its state relative to the Earth.
    const core::CartesianState relative{bodies.position(2) - bodies.position(1),
                                        bodies.velocity(2) - bodies.velocity(1)};
    const core::space::GravitationalParameter mu =
        core::constants::gravitational_constant_space * (bodies.mass(1) + bodies.mass(2));
    const auto elements = core::to_elements(relative, mu);
    REQUIRE(elements.has_value());
    CHECK(relative_error(0.0025695553, elements.value().semi_major_axis.value()) < 1e-12);
    CHECK(relative_error(0.0549, elements.value().eccentricity) < 1e-12);
    CHECK(relative_error(5.145, elements.value().inclination.degrees()) < 1e-12);
}

TEST_CASE("a circular orbit has the circular speed of mu = G (M + m)", "[scene][scenario]") {
    const auto loaded = scene::load_scene(
        document_with_bodies(sun_json + R"(, {"name": "P", "kind": "planet", "mass": 0.001,
            "elements": {"parent": "Sun", "a": 2.0, "e": 0.0, "i": 0.0, "raan": 0.0,
                         "argp": 0.0, "ma": 0.0}})"));
    REQUIRE(loaded.has_value());
    const core::BodyStore& bodies = loaded.value().state.bodies();
    const double mu = core::constants::gravitational_constant_space.value() * 1.001;
    CHECK(norm(bodies.position(1)).value() == Catch::Approx(2.0).epsilon(1e-15));
    CHECK(norm(bodies.velocity(1)).value() == Catch::Approx(std::sqrt(mu / 2.0)).epsilon(1e-14));
}

TEST_CASE("black hole spin and probe aero tables are carried", "[scene][scenario]") {
    const auto loaded = scene::load_scene(document_with_bodies(
        R"({"name": "SgrA", "kind": "blackhole", "mass": 4.0e6, "spin": 0.5,
            "state": {"position": [0.0, 0.0, 0.0], "velocity": [0.0, 0.0, 0.0]}},
           {"name": "P1", "kind": "probe", "mass": 1e-27, "aero_ref": "capsule",
            "state": {"position": [1.0, 0.0, 0.0], "velocity": [0.0, 1.0, 0.0]}},
           {"name": "P2", "kind": "probe", "mass": 1e-27,
            "state": {"position": [2.0, 0.0, 0.0], "velocity": [0.0, 1.0, 0.0]}},
           {"name": "P3", "kind": "probe", "mass": 1e-27, "aero_ref": "capsule",
            "state": {"position": [3.0, 0.0, 0.0], "velocity": [0.0, 1.0, 0.0]}})"));
    REQUIRE(loaded.has_value());
    const scene::Scene& s = loaded.value();
    const core::BodyStore& bodies = s.state.bodies();
    REQUIRE(bodies.size() == 4);
    CHECK(std::get<core::BlackHole>(bodies.kind(0)).spin == 0.5);
    REQUIRE(s.aero_tables.size() == 1);
    CHECK(s.aero_tables[0] == "capsule");
    CHECK(std::get<core::Probe>(bodies.kind(1)).aero_ref.value == 1);
    CHECK(std::get<core::Probe>(bodies.kind(2)).aero_ref.value == 0);
    CHECK(std::get<core::Probe>(bodies.kind(3)).aero_ref.value == 1);
    CHECK_FALSE(bodies.exerts_gravity(1));
}

TEST_CASE("angles may be declared in radians", "[scene][scenario]") {
    const auto parsed = scene::parse_scenario(
        document_with_bodies(sun_json + R"(, {"name": "P", "kind": "planet", "mass": 0.0,
            "elements": {"parent": "Sun", "a": 1.0, "e": 0.1, "i": 0.5, "raan": 1.0,
                         "argp": 2.0, "ma": 3.0}})"));
    REQUIRE(parsed.has_value());
    CHECK(parsed.value().angle_unit == scene::AngleUnit::radians);
    const auto& kepler = std::get<scene::KeplerianSpec>(parsed.value().bodies[1].orbit);
    CHECK(kepler.parent == "Sun");
    CHECK(kepler.elements.inclination.radians() == 0.5);
    CHECK(kepler.elements.mean_anomaly.radians() == 3.0);
}
