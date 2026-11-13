#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <filesystem>
#include <string>
#include <utility>
#include <variant>

#include "orrery/scene/aero_experiment.h"
#include "orrery/scene/experiment.h"
#include "orrery/scene/scenario.h"
#include "scene_test_support.h"

// Phase 8: the optional "series" key of the experiment schema and the typed
// aero-domain record on top of it (docs/experiments.md).

namespace scene = orrery::scene;
using scene::test::fixture_path;

namespace {

scene::AeroExperiment naca_with_result() {
    scene::AeroExperiment e;
    e.name = "NACA 0012, 4 deg";
    e.geometry.shape = scene::AeroShape::naca_section;
    e.geometry.naca = "0012";
    e.geometry.dimensions.chord_m = 0.75;
    e.conditions.speed_m_s = 34.5;
    e.conditions.angle_of_attack_deg = 4.0;
    e.conditions.sideslip_deg = -0.1;
    e.solver.solver = orrery::aero::SolverKind::lattice_boltzmann;
    e.solver.reference_area = orrery::geom::ReferenceAreaKind::planform;
    e.solver.reference_length_m = 0.75;
    e.solver.lbm_cells_per_length = 64.0;
    e.solver.lbm_duration = 40.0;
    e.solver.steps_per_frame = 250;
    scene::AeroResultRecord r;
    r.solver = orrery::aero::SolverKind::lattice_boltzmann;
    r.cd = 0.0412345678901234;
    r.cl = 0.4382910571234567;
    r.cm = -0.0123;
    r.cd_pressure = 0.03;
    r.reynolds = 1.77e6;
    r.mach = 0.1013;
    r.reference_area_m2 = 0.75;
    r.reference_length_m = 0.75;
    r.iterations = 48'000;
    r.converged = true;
    r.strouhal = 0.1723;
    r.warnings = {"lbm.reynolds", "lbm.les"};
    r.residuals = {1.0, 0.1, 1e-3, 4.9406564584124654e-324};
    r.history_step = {10.0, 20.0, 30.0};
    r.history_cd = {0.5, 0.45, 0.41};
    r.history_cl = {0.1, 0.3, 0.43};
    e.result = r;
    return e;
}

std::filesystem::path bundled_experiment(const char* name) {
    return std::filesystem::path(ORRERY_SCENARIO_DIR).parent_path() / "experiments" / name;
}

} // namespace

TEST_CASE("experiment series round-trip and stay optional", "[scene][experiment][roundtrip]") {
    scene::Experiment e;
    e.domain = scene::Domain::aero;
    e.solver.name = "lbm";
    e.series = {{"residual", {1.0, 0.25, 1.0e-300}}, {"empty", {}}};
    const std::string text = scene::save_experiment(e);
    const auto parsed = scene::parse_experiment(text);
    REQUIRE(parsed.has_value());
    CHECK(parsed.value() == e);

    // Without series nothing is written, so older readers see the old shape.
    e.series.clear();
    CHECK(scene::save_experiment(e).find("series") == std::string::npos);
}

TEST_CASE("experiment series reject non-numbers with a path", "[scene][experiment]") {
    const auto wrong = scene::parse_experiment(
        R"({"version": 1, "domain": "aero", "solver": {"name": "x"},
            "series": {"residual": [1, "two"]}})");
    REQUIRE_FALSE(wrong.has_value());
    CHECK(wrong.error().code == scene::SceneErrorCode::wrong_type);
    CHECK(wrong.error().path == "series.residual[1]");

    const auto scalar = scene::parse_experiment(
        R"({"version": 1, "domain": "aero", "solver": {"name": "x"}, "series": {"cd": 3}})");
    REQUIRE_FALSE(scalar.has_value());
    CHECK(scalar.error().path == "series.cd");
}

TEST_CASE("bundled space experiments still load unchanged", "[scene][experiment]") {
    const char* name = GENERATE("solar_system.json", "black_hole.json", "figure_eight.json");
    CAPTURE(name);
    const auto text = scene::read_text_file(bundled_experiment(name));
    REQUIRE(text.has_value());
    const auto parsed = scene::parse_experiment(text.value());
    REQUIRE(parsed.has_value());
    CHECK(parsed.value().domain == scene::Domain::space);
    CHECK(parsed.value().series.empty());
    // Saving an old record writes no new key.
    CHECK(scene::save_experiment(parsed.value()).find("series") == std::string::npos);
    // The aero view refuses it with a clear message.
    const auto aero = scene::aero_from_experiment(parsed.value());
    REQUIRE_FALSE(aero.has_value());
    CHECK(aero.error() == "domain: expected 'aero', found 'space'");
}

TEST_CASE("aero experiments round-trip exactly through JSON",
          "[scene][experiment][aero][roundtrip]") {
    SECTION("primitive with a finished result") {
        const scene::AeroExperiment e = naca_with_result();
        const std::string text = scene::save_aero_experiment(e);
        CHECK(text.find("\"domain\": \"aero\"") != std::string::npos);
        CHECK(text.find("\"version\": 1") != std::string::npos);
        const auto parsed = scene::parse_aero_experiment(text);
        REQUIRE(parsed.has_value());
        CHECK(parsed.value() == e);
        // The generic record holds the documented keys.
        const scene::Experiment generic = scene::to_experiment(e);
        CHECK(std::get<std::string>(generic.inputs.at("shape")) == "naca_section");
        CHECK(std::get<double>(generic.inputs.at("chord_m")) == 0.75);
        CHECK_FALSE(generic.inputs.contains("diameter_m"));
        CHECK(std::get<std::string>(generic.result.at("warnings")) == "lbm.reynolds; lbm.les");
        CHECK(generic.series.at("history_cd").size() == 3);
    }
    SECTION("every primitive shape without a result") {
        const auto shape = GENERATE(scene::AeroShape::sphere, scene::AeroShape::cube,
                                    scene::AeroShape::cylinder, scene::AeroShape::cone,
                                    scene::AeroShape::ellipsoid, scene::AeroShape::flat_plate,
                                    scene::AeroShape::wedge, scene::AeroShape::capsule,
                                    scene::AeroShape::naca_wing, scene::AeroShape::naca_section);
        CAPTURE(scene::to_string(shape));
        scene::AeroExperiment e;
        e.geometry.shape = shape;
        e.geometry.capsule = "orion";
        e.geometry.naca = "2412";
        // Only the dimensions the shape stores take part in the round trip.
        for (const std::string_view key : scene::shape_dimensions(shape)) {
            CHECK_FALSE(key.empty());
        }
        const auto parsed = scene::parse_aero_experiment(scene::save_aero_experiment(e));
        REQUIRE(parsed.has_value());
        CHECK(parsed.value().geometry.shape == shape);
        CHECK_FALSE(parsed.value().result.has_value());
        CHECK_FALSE(parsed.value().solver.solver.has_value()); // "auto"
        CHECK(scene::aero_shape_from_string(scene::to_string(shape)) == shape);
    }
    SECTION("user model from a URL keeps its source and licence note") {
        scene::AeroExperiment e;
        e.geometry.source = scene::GeometrySource::url;
        e.geometry.url = "https://example.org/models/glider.glb";
        e.geometry.path = "cache/3f2a.glb";
        e.geometry.sha256 = "3f2a9c";
        e.geometry.license_note = "CC BY 4.0, example.org";
        e.geometry.unit_scale = 0.001;
        e.geometry.up_axis = orrery::geom::UpAxis::y_up;
        REQUIRE(e.geometry.user_model());
        const auto parsed = scene::parse_aero_experiment(scene::save_aero_experiment(e));
        REQUIRE(parsed.has_value());
        CHECK(parsed.value() == e);
    }
}

TEST_CASE("aero experiment fixture loads with defaults for absent keys",
          "[scene][experiment][aero]") {
    const auto text = scene::read_text_file(fixture_path("aero_sphere_minimal.json"));
    REQUIRE(text.has_value());
    const auto parsed = scene::parse_aero_experiment(text.value());
    REQUIRE(parsed.has_value());
    const scene::AeroExperiment& e = parsed.value();
    CHECK(e.geometry.shape == scene::AeroShape::sphere);
    CHECK(e.geometry.dimensions.diameter_m == 0.2);
    CHECK(e.conditions.speed_m_s == 10.0);
    CHECK(e.conditions.density_kg_m3 == 1.225); // default
    CHECK(e.solver.solver == orrery::aero::SolverKind::newtonian);
    CHECK(e.solver.reference_area == orrery::geom::ReferenceAreaKind::frontal);
    const orrery::aero::FlowConditions flow = e.conditions.flow();
    CHECK(flow.speed == 10.0);
    CHECK(flow.angle_of_attack == 0.0);
}

TEST_CASE("aero experiment errors name the offending key", "[scene][experiment][aero]") {
    const auto error_of = [](const std::string& inputs, const std::string& conditions,
                             const std::string& solver, const std::string& tail = "") {
        const std::string json = R"({"version": 1, "domain": "aero", "inputs": )" + inputs +
                                 R"(, "conditions": )" + conditions + R"(, "solver": )" + solver +
                                 tail + "}";
        const auto parsed = scene::parse_aero_experiment(json);
        REQUIRE_FALSE(parsed.has_value());
        return parsed.error();
    };
    const std::string ok_inputs = R"({"geometry": "primitive", "shape": "sphere"})";
    const std::string ok_solver = R"({"name": "auto"})";
    CHECK(error_of(R"({"shape": "torus"})", "{}", ok_solver)
              .starts_with("inputs.shape: unknown shape 'torus'"));
    CHECK(error_of(R"({"shape": "sphere", "chord_m": 1})", "{}", ok_solver) ==
          "inputs.chord_m: unknown key (allowed: name, geometry, shape, diameter_m)");
    CHECK(error_of(R"({"shape": "naca_wing", "naca": "00x2"})", "{}", ok_solver) ==
          "inputs.naca: '00x2' is not a NACA 4-digit designation");
    CHECK(error_of(R"({"geometry": "file"})", "{}", ok_solver) ==
          "inputs.path: a file geometry needs a model path");
    CHECK(error_of(ok_inputs, R"({"speed_m_s": -3})", ok_solver) ==
          "conditions.speed_m_s: must be positive and finite");
    CHECK(error_of(ok_inputs, R"({"mach": 0.3})", ok_solver)
              .starts_with("conditions.mach: unknown key"));
    CHECK(error_of(ok_inputs, "{}", R"({"name": "rans"})") ==
          "solver.name: unknown solver 'rans' (allowed: auto, panel, newtonian, lbm)");
    CHECK(error_of(ok_inputs, "{}", R"({"name": "lbm", "settings": {"steps_per_frame": 2.5}})") ==
          "solver.settings.steps_per_frame: must be an integer in [1, 1000000]");
    CHECK(error_of(ok_inputs, "{}", R"({"name": "lbm", "settings": {"reference_area": "side"}})")
              .starts_with("solver.settings.reference_area: unknown reference area 'side'"));
    CHECK(error_of(ok_inputs, "{}", ok_solver, R"(, "result": {"cd": 1})") ==
          "result.solver: a result names the solver that produced it");
    CHECK(error_of(ok_inputs, "{}", ok_solver,
                   R"(, "result": {"solver": "lbm"}, "series": {"history_cd": [1, 2]})") ==
          "series: history_step, history_cd and history_cl must have the same length");
}
