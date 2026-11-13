#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>

#include "orrery/atmo/entry.h"
#include "orrery/scene/entry_experiment.h"
#include "orrery/scene/experiment.h"
#include "scene_test_support.h"

// Phase 9a: the typed entry-domain record on the generic experiment schema
// (docs/atmosphere.md, "Experiment record"); schema version stays 1.

namespace scene = orrery::scene;
using scene::test::fixture_path;

namespace {

std::string read_file(const std::string& name) {
    std::ifstream in(fixture_path(name), std::ios::binary);
    REQUIRE(in);
    std::ostringstream text;
    text << in.rdbuf();
    return text.str();
}

scene::EntryExperiment custom_record() {
    scene::EntryExperiment e;
    e.name = "Mars, exponential atmosphere";
    e.environment.planet = "mars";
    e.environment.atmosphere = scene::EntryAtmosphereModel::exponential;
    e.environment.surface_density_kg_m3 = 0.0158;
    e.environment.scale_height_m = 11100.0;
    e.environment.temperature_k = 210.0;
    e.vehicle.name = "aeroshell";
    e.vehicle.mass_kg = 585.25;
    e.vehicle.reference_area_m2 = 5.515;
    e.vehicle.nose_radius_m = 0.6638;
    e.vehicle.cd_table_mach = {2.0, 5.0, 10.0, 20.0, 30.0};
    e.vehicle.cd_table_cd = {1.45, 1.62, 1.68, 1.70, 1.71};
    e.conditions.altitude_m = 125000.0;
    e.conditions.speed_m_s = 7480.0;
    e.conditions.flight_path_angle_deg = -14.06;
    e.conditions.latitude_deg = 19.13;
    e.conditions.longitude_deg = -33.22;
    e.conditions.heading_deg = 253.5;
    e.conditions.interface_altitude_m = 130000.0;
    e.conditions.frame = orrery::atmo::AtmosphereFrame::co_rotating;
    e.solver.integrator = orrery::atmo::EntryIntegrator::yoshida4;
    e.solver.tolerance = 1.0e-10;
    e.solver.time_step_s = 0.25;
    e.solver.max_coast_step_s = 10.0;
    e.solver.max_duration_s = 900.0;
    e.solver.stop_on_skip_out = false;
    return e;
}

scene::EntryExperiment roundtrip(const scene::EntryExperiment& e) {
    auto back = scene::parse_entry_experiment(scene::save_entry_experiment(e));
    INFO((back ? std::string() : back.error()));
    REQUIRE(back);
    return back.value();
}

std::string with_replaced(std::string text, const std::string& from, const std::string& to) {
    const auto at = text.find(from);
    REQUIRE(at != std::string::npos);
    return text.replace(at, from.size(), to);
}

} // namespace

TEST_CASE("Entry record round-trips through JSON", "[scene][entry]") {
    const scene::EntryExperiment defaults;
    CHECK(roundtrip(defaults) == defaults);
    const scene::EntryExperiment custom = custom_record();
    CHECK(roundtrip(custom) == custom);
    const scene::Experiment generic = scene::to_experiment(custom);
    CHECK(generic.domain == scene::Domain::entry);
    CHECK(generic.solver.name == "yoshida4");
    CHECK(generic.solver.version == "1");
    CHECK(scene::save_entry_experiment(custom).find("\"version\": 1") != std::string::npos);
}

TEST_CASE("Entry record with a run's result round-trips", "[scene][entry]") {
    scene::EntryExperiment e;
    e.vehicle.cd_table_mach = {0.0};
    e.vehicle.cd_table_cd = {1.3};
    e.conditions.flight_path_angle_deg = -5.7;
    e.solver.max_duration_s = 120.0;
    auto config = scene::to_entry_config(e);
    REQUIRE(config);
    auto run = orrery::atmo::EntryExperiment::create(config.value());
    REQUIRE(run);
    REQUIRE(run.value().run());
    e.result = scene::entry_result(run.value());
    REQUIRE(e.result);
    CHECK(e.result->outcome == orrery::atmo::EntryOutcome::time_limit);
    CHECK(e.result->entry_time_s == 0.0);
    CHECK_FALSE(e.result->impact_time_s.has_value());
    CHECK(e.result->series.size() == run.value().series().size());
    CHECK(e.result->peak_g_load > 1.0);
    CHECK(roundtrip(e) == e);
}

TEST_CASE("Entry fixture builds the Apollo-like configuration", "[scene][entry]") {
    auto record = scene::parse_entry_experiment(read_file("entry_apollo.json"));
    INFO((record ? std::string() : record.error()));
    REQUIRE(record);
    CHECK(record.value().name == "Apollo-like lunar return (ballistic)");
    auto config = scene::to_entry_config(record.value());
    REQUIRE(config);
    const orrery::atmo::EntryConfig preset = orrery::atmo::apollo_like_entry();
    const orrery::atmo::EntryConfig& c = config.value();
    CHECK(c.planet.name == "earth");
    CHECK(c.planet.atmosphere->name() == "us76");
    CHECK(c.vehicle.mass == preset.vehicle.mass);
    CHECK(std::abs(c.vehicle.aero.reference_area().value() /
                       preset.vehicle.aero.reference_area().value() -
                   1.0) < 1e-6);
    CHECK(c.vehicle.aero.cd(25.0) == 1.3);
    CHECK(c.initial.speed == preset.initial.speed);
    CHECK(std::abs(c.initial.flight_path_angle.degrees() + 5.7) < 1e-12);
    CHECK(c.integrator == orrery::atmo::EntryIntegrator::ias15);
    CHECK(c.planet.entry_interface_altitude.value() == 121920.0);
}

TEST_CASE("Entry configuration from the custom record", "[scene][entry]") {
    auto config = scene::to_entry_config(custom_record());
    REQUIRE(config);
    const orrery::atmo::EntryConfig& c = config.value();
    CHECK(c.planet.name == "mars");
    CHECK(c.planet.atmosphere->name() == "exponential");
    CHECK(c.planet.entry_interface_altitude.value() == 130000.0);
    CHECK(c.frame == orrery::atmo::AtmosphereFrame::co_rotating);
    CHECK(std::abs(c.vehicle.aero.cd(7.5) - 1.65) < 1e-12);
    CHECK(c.max_duration.value() == 900.0);
    CHECK_FALSE(c.stop_on_skip_out);
}

TEST_CASE("Entry record errors name the offending key", "[scene][entry]") {
    const std::string text = read_file("entry_apollo.json");
    const auto error_of = [](const std::string& json) {
        auto parsed = scene::parse_entry_experiment(json);
        REQUIRE_FALSE(parsed);
        return parsed.error();
    };
    CHECK(error_of(with_replaced(text, "\"mass_kg\": 5560", "\"mass_kg\": -1")) ==
          "inputs.mass_kg: must be positive and finite");
    CHECK(error_of(with_replaced(text, "\"planet\": \"earth\"", "\"planet\": \"venus\""))
              .find("inputs.planet") == 0);
    CHECK(error_of(with_replaced(text, "\"frame\": \"inertial\"", "\"frame\": \"spinning\""))
              .find("conditions.frame") == 0);
    CHECK(error_of(with_replaced(text, "\"name\": \"ias15\"", "\"name\": \"rk4\""))
              .find("solver.name") == 0);
    CHECK(error_of(with_replaced(text, "\"stop_on_skip_out\": 1", "\"stop_on_skip_out\": 2")) ==
          "solver.settings.stop_on_skip_out: must be 0 or 1");
    CHECK(
        error_of(with_replaced(text, "\"heading_deg\": 90", "\"heading_deg\": 90, \"wind\": 3")) ==
        "conditions.wind: unknown key");
    CHECK(error_of(with_replaced(text, "\"flight_path_angle_deg\": -5.7",
                                 "\"flight_path_angle_deg\": -95"))
              .find("conditions.flight_path_angle_deg") == 0);
    CHECK(error_of(with_replaced(text, "\"cd_table_mach\": [0], \"cd_table_cd\": [1.3]",
                                 "\"cd_table_mach\": [5, 2], \"cd_table_cd\": [1.3, 1.2]"))
              .find("series.cd_table") == 0);
    CHECK(error_of(with_replaced(text, "\"cd_table_cd\": [1.3]", "\"cd_table_cd\": [1.3, 1.2]"))
              .find("series") == 0);
    CHECK(error_of(with_replaced(text, "\"atmosphere\": \"us76\"",
                                 "\"atmosphere\": \"us76\", \"scale_height_m\": 7000"))
              .find("inputs.scale_height_m") == 0);
    CHECK(error_of(with_replaced(text, "\"series\": {", "\"series\": { \"g_load\": [1],"))
              .find("series.g_load") == 0);
    CHECK(error_of(with_replaced(text, "\"domain\": \"entry\"", "\"domain\": \"aero\""))
              .find("domain") == 0);
    // The schema version is still 1.
    CHECK_FALSE(
        scene::parse_entry_experiment(with_replaced(text, "\"version\": 1", "\"version\": 2")));
}
