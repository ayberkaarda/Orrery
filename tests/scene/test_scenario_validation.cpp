#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <cmath>
#include <limits>
#include <string>
#include <variant>

#include "orrery/core/kepler.h"
#include "orrery/scene/error.h"
#include "orrery/scene/experiment.h"
#include "orrery/scene/scenario.h"
#include "scene_test_support.h"

namespace core = orrery::core;
namespace scene = orrery::scene;
using scene::SceneErrorCode;
using scene::test::document_with_bodies;
using scene::test::fixture_path;
using scene::test::sun_json;

namespace {

scene::SceneError load_error(const std::string& json) {
    const auto loaded = scene::load_scene(json);
    REQUIRE_FALSE(loaded.has_value());
    return loaded.error();
}

scene::SceneError fixture_error(const std::string& name) {
    const auto loaded = scene::load_scene_file(fixture_path(name));
    REQUIRE_FALSE(loaded.has_value());
    return loaded.error();
}

std::string planet_with(const std::string& fields) {
    return sun_json + R"(, {"name": "P", "kind": "planet", )" + fields + "}";
}

const std::string circular_elements =
    R"("elements": {"parent": "Sun", "a": 1.0, "e": 0.0, "i": 0.0, "raan": 0.0, "argp": 0.0,
                    "ma": 0.0})";

} // namespace

TEST_CASE("invalid fixtures are rejected with the exact path", "[scene][validation]") {
    struct Case {
        const char* file;
        SceneErrorCode code;
        const char* path;
    };
    const Case c = GENERATE(
        Case{"unknown_key.json", SceneErrorCode::unknown_key, "bodies[1].colour"},
        Case{"negative_mass.json", SceneErrorCode::invalid_value, "bodies[1].mass"},
        Case{"infinite_mass.json", SceneErrorCode::invalid_value, "bodies[1].mass"},
        Case{"wrong_type_mass.json", SceneErrorCode::wrong_type, "bodies[1].mass"},
        Case{"negative_eccentricity.json", SceneErrorCode::invalid_value, "bodies[2].elements.e"},
        Case{"missing_parent.json", SceneErrorCode::missing_key, "bodies[1].elements.parent"},
        Case{"unknown_parent.json", SceneErrorCode::unknown_parent, "bodies[1].elements.parent"},
        Case{"parent_cycle.json", SceneErrorCode::parent_cycle, "bodies[1].elements.parent"},
        Case{"duplicate_name.json", SceneErrorCode::duplicate_name, "bodies[1].name"},
        Case{"parabolic_orbit.json", SceneErrorCode::kepler_error, "bodies[1].elements.e"},
        Case{"unsupported_version.json", SceneErrorCode::unsupported_version, "version"},
        Case{"unsupported_unit.json", SceneErrorCode::invalid_value, "units.mass"},
        Case{"duplicate_key.json", SceneErrorCode::duplicate_key, "bodies[0].mass"},
        Case{"syntax_error.json", SceneErrorCode::syntax_error, ""});
    CAPTURE(c.file);
    const scene::SceneError error = fixture_error(c.file);
    CAPTURE(scene::to_string(error));
    CHECK(error.code == c.code);
    CHECK(error.path == c.path);
    CHECK_FALSE(error.message.empty());
}

TEST_CASE("parent cycles name every body on the cycle", "[scene][validation]") {
    const scene::SceneError error = fixture_error("parent_cycle.json");
    CHECK(error.message.find("A -> B -> A") != std::string::npos);
}

TEST_CASE("unsupported orbit types propagate the KeplerError", "[scene][validation]") {
    const scene::SceneError parabolic = fixture_error("parabolic_orbit.json");
    REQUIRE(parabolic.kepler.has_value());
    CHECK(*parabolic.kepler == core::KeplerError::parabolic_orbit);

    // Hyperbolic eccentricity with a positive semi-major axis.
    const scene::SceneError inconsistent = load_error(document_with_bodies(planet_with(
        R"("mass": 0.0, "elements": {"parent": "Sun", "a": 1.0, "e": 1.5, "i": 0.0,
           "raan": 0.0, "argp": 0.0, "ma": 0.0})")));
    CHECK(inconsistent.code == SceneErrorCode::kepler_error);
    CHECK(inconsistent.path == "bodies[1].elements.a");
    REQUIRE(inconsistent.kepler.has_value());
    CHECK(*inconsistent.kepler == core::KeplerError::inconsistent_semi_major_axis);

    // Inclination beyond pi.
    const scene::SceneError inclined = load_error(document_with_bodies(planet_with(
        R"("mass": 0.0, "elements": {"parent": "Sun", "a": 1.0, "e": 0.1, "i": 4.0,
           "raan": 0.0, "argp": 0.0, "ma": 0.0})")));
    CHECK(inclined.path == "bodies[1].elements.i");
    CHECK(inclined.kepler == core::KeplerError::invalid_inclination);
}

TEST_CASE("schema errors carry precise paths", "[scene][validation]") {
    SECTION("wrong type of an element") {
        const auto error = load_error(document_with_bodies(planet_with(
            R"("mass": 0.0, "elements": {"parent": "Sun", "a": 1.0, "e": "0.1", "i": 0.0,
               "raan": 0.0, "argp": 0.0, "ma": 0.0})")));
        CHECK(error.code == SceneErrorCode::wrong_type);
        CHECK(error.path == "bodies[1].elements.e");
    }
    SECTION("wrong type of a state component") {
        const auto error = load_error(document_with_bodies(planet_with(
            R"("mass": 0.0, "state": {"position": [1.0, null, 0.0], "velocity": [0, 0, 0]})")));
        CHECK(error.code == SceneErrorCode::wrong_type);
        CHECK(error.path == "bodies[1].state.position[1]");
    }
    SECTION("state vector of the wrong length") {
        const auto error = load_error(document_with_bodies(planet_with(
            R"("mass": 0.0, "state": {"position": [1.0, 0.0], "velocity": [0, 0, 0]})")));
        CHECK(error.code == SceneErrorCode::invalid_value);
        CHECK(error.path == "bodies[1].state.position");
    }
    SECTION("neither state nor elements") {
        const auto error = load_error(document_with_bodies(planet_with(R"("mass": 0.0)")));
        CHECK(error.code == SceneErrorCode::missing_key);
        CHECK(error.path == "bodies[1]");
    }
    SECTION("both state and elements") {
        const auto error = load_error(document_with_bodies(planet_with(
            R"("mass": 0.0, "state": {"position": [1, 0, 0], "velocity": [0, 0, 0]}, )" +
            circular_elements)));
        CHECK(error.code == SceneErrorCode::invalid_value);
        CHECK(error.path == "bodies[1]");
    }
    SECTION("unknown kind") {
        const auto error = load_error(
            document_with_bodies(sun_json + R"(, {"name": "X", "kind": "comet", "mass": 0.0, )" +
                                 circular_elements + "}"));
        CHECK(error.code == SceneErrorCode::invalid_value);
        CHECK(error.path == "bodies[1].kind");
    }
    SECTION("spin on a body that is not a black hole") {
        const auto error = load_error(
            document_with_bodies(planet_with(R"("mass": 0.0, "spin": 0.1, )" + circular_elements)));
        CHECK(error.code == SceneErrorCode::unknown_key);
        CHECK(error.path == "bodies[1].spin");
    }
    SECTION("black hole spin outside [0, 1]") {
        const auto error = load_error(document_with_bodies(
            R"({"name": "BH", "kind": "blackhole", "mass": 10.0, "spin": 1.5,
                "state": {"position": [0, 0, 0], "velocity": [0, 0, 0]}})"));
        CHECK(error.code == SceneErrorCode::invalid_value);
        CHECK(error.path == "bodies[0].spin");
    }
    SECTION("probe with zero mass") {
        const auto error = load_error(document_with_bodies(
            R"({"name": "Probe", "kind": "probe", "mass": 0.0,
                "state": {"position": [1, 0, 0], "velocity": [0, 0, 0]}})"));
        CHECK(error.code == SceneErrorCode::invalid_value);
        CHECK(error.path == "bodies[0].mass");
    }
    SECTION("probe as a parent") {
        const auto error = load_error(document_with_bodies(
            R"({"name": "Sun", "kind": "probe", "mass": 1e-27,
                "state": {"position": [1, 0, 0], "velocity": [0, 0, 0]}},
               {"name": "P", "kind": "planet", "mass": 0.0, )" +
            circular_elements + "}"));
        CHECK(error.code == SceneErrorCode::invalid_value);
        CHECK(error.path == "bodies[1].elements.parent");
    }
    SECTION("self parent is a cycle") {
        const auto error = load_error(document_with_bodies(
            R"({"name": "Sun", "kind": "star", "mass": 1.0, )" + circular_elements + "}"));
        CHECK(error.code == SceneErrorCode::parent_cycle);
        CHECK(error.path == "bodies[0].elements.parent");
    }
    SECTION("unknown top-level key") {
        const auto error = load_error(R"({"version": 1, "extra": true})");
        CHECK(error.code == SceneErrorCode::unknown_key);
        CHECK(error.path == "extra");
    }
    SECTION("version of the wrong type") {
        const auto error = load_error(R"({"version": "1"})");
        CHECK(error.code == SceneErrorCode::wrong_type);
        CHECK(error.path == "version");
    }
    SECTION("missing version") {
        const auto error = load_error(R"({"bodies": []})");
        CHECK(error.code == SceneErrorCode::missing_key);
        CHECK(error.path == "version");
    }
    SECTION("root is not an object") {
        const auto error = load_error("[1, 2]");
        CHECK(error.code == SceneErrorCode::wrong_type);
        CHECK(error.path.empty());
    }
    SECTION("missing metadata source") {
        const auto error = load_error(R"({"version": 1,
            "units": {"length": "AU", "mass": "Msun", "time": "day", "angle": "deg"},
            "metadata": {"epoch": "J2000"}, "bodies": []})");
        CHECK(error.code == SceneErrorCode::missing_key);
        CHECK(error.path == "metadata.source");
    }
}

TEST_CASE("in-memory scenarios are validated like documents", "[scene][validation]") {
    scene::Scenario scenario;
    scenario.metadata = {"test", "t = 0", ""};
    scene::BodySpec body;
    body.name = "Sun";
    body.kind = core::BodyKindTag::star;
    body.mass = core::space::Mass(1.0);
    scenario.bodies.push_back(body);
    REQUIRE_FALSE(scene::validate(scenario).has_value());

    SECTION("NaN mass") {
        scenario.bodies[0].mass = core::space::Mass(std::numeric_limits<double>::quiet_NaN());
    }
    SECTION("infinite mass") {
        scenario.bodies[0].mass = core::space::Mass(std::numeric_limits<double>::infinity());
    }
    SECTION("negative mass") {
        scenario.bodies[0].mass = core::space::Mass(-1.0);
    }
    const auto error = scene::validate(scenario);
    REQUIRE(error.has_value());
    CHECK(error->code == SceneErrorCode::invalid_value);
    CHECK(error->path == "bodies[0].mass");
    const auto built = scene::build_scene(scenario);
    REQUIRE_FALSE(built.has_value());
    CHECK(built.error().path == "bodies[0].mass");
}

TEST_CASE("nonzero numbers that underflow to zero are rejected", "[scene][validation]") {
    SECTION("negative mass below the double range") {
        const auto error = load_error(
            R"({"version":1,"units":{"length":"AU","mass":"Msun","time":"day","angle":"rad"},)"
            R"("metadata":{"source":"test","epoch":"0"},"bodies":[{"name":"S","kind":"star",)"
            R"("mass":-1e-400,"state":{"position":[0,0,0],"velocity":[0,0,0]}}]})");
        CHECK(error.code == SceneErrorCode::invalid_value);
        CHECK(error.path == "bodies[0].mass");
    }
    SECTION("negative eccentricity below the double range") {
        const auto error = load_error(document_with_bodies(
            planet_with(R"("mass": 0.001, "elements": {"parent": "Sun", "a": 1.0, "e": -1e-400,
                           "i": 0.0, "raan": 0.0, "argp": 0.0, "ma": 0.0})")));
        CHECK(error.code == SceneErrorCode::invalid_value);
        CHECK(error.path == "bodies[1].elements.e");
    }
    SECTION("positive value below the double range") {
        const auto error = load_error(
            document_with_bodies(planet_with(R"("mass": 0.25e-330, "state": {"position": [1, 0, 0],
                           "velocity": [0, 0, 0]})")));
        CHECK(error.code == SceneErrorCode::invalid_value);
        CHECK(error.path == "bodies[1].mass");
    }
    SECTION("state component below the double range") {
        const auto error = load_error(
            document_with_bodies(planet_with(R"("mass": 0.0, "state": {"position": [1, 0, 0],
                           "velocity": [0, -7.5E-999, 0]})")));
        CHECK(error.code == SceneErrorCode::invalid_value);
        CHECK(error.path == "bodies[1].state.velocity[1]");
    }
}

TEST_CASE("explicit zeros, signed zeros and subnormals are accepted", "[scene][validation]") {
    // A zero lexeme, with or without a sign or an exponent, is an intentional
    // zero: -0.0 compares equal to 0.0, so it is a zero mass or a circular
    // orbit, and it is kept as -0.0 (bit for bit) in memory.
    const auto parsed = scene::parse_scenario(
        document_with_bodies(sun_json + R"(, {"name": "P", "kind": "planet", "mass": -0.0,
            "elements": {"parent": "Sun", "a": 1.0, "e": -0.0e-400, "i": 0E+5, "raan": 0.0,
                         "argp": 0.0, "ma": -0}},
           {"name": "D", "kind": "asteroid", "mass": 4.9e-324,
            "state": {"position": [2, -0.0, 0], "velocity": [0, 1, 0]}})"));
    REQUIRE(parsed.has_value());
    const scene::BodySpec& planet = parsed.value().bodies[1];
    CHECK(planet.mass.value() == 0.0);
    CHECK(std::signbit(planet.mass.value()));
    const auto& elements = std::get<scene::KeplerianSpec>(planet.orbit).elements;
    CHECK(elements.eccentricity == 0.0);
    CHECK(std::signbit(elements.eccentricity));
    const scene::BodySpec& dust = parsed.value().bodies[2];
    CHECK(dust.mass.value() > 0.0);
    CHECK(std::signbit(std::get<scene::CartesianSpec>(dust.orbit).position.y.value()));
    CHECK(scene::build_scene(parsed.value()).has_value());

    // A probe still needs a positive mass; -0.0 is not positive.
    const auto probe = load_error(document_with_bodies(
        R"({"name": "Probe", "kind": "probe", "mass": -0.0,
            "state": {"position": [1, 0, 0], "velocity": [0, 0, 0]}})"));
    CHECK(probe.code == SceneErrorCode::invalid_value);
    CHECK(probe.path == "bodies[0].mass");
}

TEST_CASE("experiment numbers that underflow to zero are rejected", "[scene][validation]") {
    const auto parsed = scene::parse_experiment(
        R"({"version": 1, "domain": "space", "inputs": {"dt": -1e-400},
            "solver": {"name": "s"}})");
    REQUIRE_FALSE(parsed.has_value());
    CHECK(parsed.error().code == SceneErrorCode::invalid_value);
    CHECK(parsed.error().path == "inputs.dt");
}

TEST_CASE("a missing file is an io error", "[scene][validation]") {
    const auto loaded = scene::load_scene_file(fixture_path("does_not_exist.json"));
    REQUIRE_FALSE(loaded.has_value());
    CHECK(loaded.error().code == SceneErrorCode::io_error);
}
