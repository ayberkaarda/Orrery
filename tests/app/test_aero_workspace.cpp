#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "app_test_support.h"
#include "orrery/sim/aero_comparison.h"
#include "orrery/sim/aero_flow.h"
#include "orrery/sim/aero_geometry.h"
#include "orrery/sim/aero_import.h"
#include "orrery/sim/aero_runner.h"
#include "orrery/sim/aero_solve.h"
#include "orrery/sim/aero_view_data.h"
#include "orrery/sim/background_task.h"

// Phase 8b: the graphics-free side of the Aero workspace (geometry, flow
// summary, solvers, solver thread, imports, experiment files, comparison).

using namespace orrery;
using Catch::Approx;

namespace {

constexpr double kPi = std::numbers::pi;

scene::GeometryRef primitive(scene::AeroShape shape) {
    scene::GeometryRef ref;
    ref.shape = shape;
    return ref;
}

std::shared_ptr<const sim::PreparedGeometry> prepared(const scene::GeometryRef& ref) {
    auto built = sim::build_primitive(ref);
    REQUIRE(built.has_value());
    return std::make_shared<const sim::PreparedGeometry>(std::move(built).value());
}

std::filesystem::path scratch_dir(const char* name) {
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() / "orrery_test_aero_workspace" / name;
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}

scene::AeroExperiment naca_section_experiment(double aoa_deg) {
    scene::AeroExperiment e;
    e.name = "NACA 0012 at " + std::to_string(aoa_deg);
    e.geometry = primitive(scene::AeroShape::naca_section);
    e.conditions.speed_m_s = 30.0;
    e.conditions.angle_of_attack_deg = aoa_deg;
    e.solver.solver = aero::SolverKind::panel;
    return e;
}

// A tiny 2D lattice Boltzmann run: a few hundred lattice steps.
scene::AeroExperiment tiny_lbm_experiment() {
    scene::AeroExperiment e;
    e.geometry = primitive(scene::AeroShape::naca_section);
    e.geometry.dimensions.chord_m = 0.1;
    e.conditions.speed_m_s = 0.5;
    e.conditions.viscosity_pa_s = 1.0e-3; // Re about 60: a stable, short run
    e.conditions.angle_of_attack_deg = 0.0;
    e.solver.solver = aero::SolverKind::lattice_boltzmann;
    e.solver.lbm_cells_per_length = 12.0;
    e.solver.lbm_duration = 1.5;
    e.solver.steps_per_frame = 50;
    return e;
}

} // namespace

TEST_CASE("Primitives are watertight, outward and face the free stream", "[aero][workspace]") {
    const auto shape =
        GENERATE(scene::AeroShape::sphere, scene::AeroShape::cube, scene::AeroShape::cylinder,
                 scene::AeroShape::cone, scene::AeroShape::ellipsoid, scene::AeroShape::flat_plate,
                 scene::AeroShape::wedge, scene::AeroShape::capsule, scene::AeroShape::naca_wing);
    CAPTURE(scene::to_string(shape));
    const auto geometry = prepared(primitive(shape));
    CHECK_FALSE(geometry->two_dimensional);
    CHECK_FALSE(geometry->unvalidated);
    CHECK(geometry->report.watertight());
    CHECK(geometry->report.outward());
    CHECK(geometry->quantities.frontal_area > 0.0);
    CHECK(geometry->quantities.planform_area > 0.0);
    CHECK(geometry->quantities.wetted_area > 0.0);
    CHECK_FALSE(geometry->label.empty());
    // Centred on the origin.
    const geom::Bounds b = geom::bounds(geometry->mesh);
    CHECK(std::abs(b.centre().x) < 1e-9);
    CHECK(std::abs(b.centre().y) < 1e-9);
}

TEST_CASE("Primitive orientation and reference quantities", "[aero][workspace]") {
    SECTION("sphere: frontal area pi r^2") {
        scene::GeometryRef ref = primitive(scene::AeroShape::sphere);
        ref.dimensions.diameter_m = 0.4;
        const auto g = prepared(ref);
        CHECK(g->quantities.frontal_area == Approx(kPi * 0.04).epsilon(0.01));
        const aero::ReferenceValues r = sim::reference_values(*g, {});
        CHECK(r.area == Approx(kPi * 0.04).epsilon(0.01));
        CHECK(r.length == Approx(0.4).epsilon(1e-6));
    }
    SECTION("cylinder lies across the flow") {
        scene::GeometryRef ref = primitive(scene::AeroShape::cylinder);
        ref.dimensions.diameter_m = 0.5;
        ref.dimensions.length_m = 3.0;
        const geom::Vector3 extent = geom::bounds(prepared(ref)->mesh).extent();
        CHECK(extent.y == Approx(3.0));
        CHECK(extent.x == Approx(0.5).epsilon(0.01));
    }
    SECTION("cone apex points upstream") {
        scene::GeometryRef ref = primitive(scene::AeroShape::cone);
        const auto g = prepared(ref);
        const auto upstream = std::min_element(
            g->mesh.vertices.begin(), g->mesh.vertices.end(),
            [](const geom::Vector3& a, const geom::Vector3& b) { return a.x < b.x; });
        CHECK(std::abs(upstream->y) < 1e-9);
        CHECK(std::abs(upstream->z) < 1e-9);
    }
    SECTION("NACA wing: quarter-chord moment centre, trailing edge found") {
        scene::GeometryRef ref = primitive(scene::AeroShape::naca_wing);
        ref.dimensions.chord_m = 2.0;
        const auto g = prepared(ref);
        CHECK_FALSE(g->trailing_edge.empty());
        const geom::Bounds b = geom::bounds(g->mesh);
        CHECK(g->moment_centre.x == Approx(b.min.x + 0.5));
        scene::AeroSolverSettings planform;
        planform.reference_area = geom::ReferenceAreaKind::planform;
        CHECK(sim::reference_values(*g, planform).area == Approx(2.0 * 4.0).epsilon(0.01));
    }
    SECTION("NACA section is two-dimensional, per unit span") {
        scene::GeometryRef ref = primitive(scene::AeroShape::naca_section);
        ref.dimensions.chord_m = 0.5;
        const auto g = prepared(ref);
        CHECK(g->two_dimensional);
        CHECK(g->section.size() == 2U * static_cast<std::size_t>(sim::kNacaStations));
        CHECK_FALSE(g->mesh.triangles.empty()); // the preview
        const aero::AeroGeometry body = sim::to_aero_geometry(*g, {});
        CHECK(body.mesh.triangles.empty());
        CHECK(body.reference.area == 0.5);
        CHECK(body.reference.length == 0.5);
        CHECK(body.reference.moment_centre.x == Approx(0.125));
    }
    SECTION("an impossible primitive is an error, not a throw") {
        scene::GeometryRef ref = primitive(scene::AeroShape::naca_wing);
        ref.naca = "xx";
        const auto built = sim::build_primitive(ref);
        REQUIRE_FALSE(built.has_value());
        CHECK(built.error().title == "Cannot build the NACA wing");
    }
}

TEST_CASE("Flow summary derives Re and Mach and never blocks on the envelope",
          "[aero][workspace]") {
    scene::AeroConditions c;
    c.speed_m_s = 50.0;
    const sim::FlowSummary low = sim::summarize_flow(c, 1.0, std::nullopt, false);
    REQUIRE(low.runnable());
    CHECK(low.reynolds == Approx(1.225 * 50.0 / 1.789e-5));
    CHECK(low.mach == Approx(50.0 / std::sqrt(1.4 * 287.05 * 288.15)));
    CHECK(low.dynamic_pressure == Approx(0.5 * 1.225 * 2500.0));
    CHECK(low.solver == aero::SolverKind::panel);
    CHECK_FALSE(low.regime.overridden);

    // Mach 0.6: no solver is valid; the run still goes, with warnings.
    c.speed_m_s = 204.0;
    const sim::FlowSummary transonic = sim::summarize_flow(c, 1.0, std::nullopt, false);
    CHECK(transonic.runnable());
    CHECK_FALSE(transonic.warnings.empty());

    // An override is honoured, with its envelope warnings.
    c.speed_m_s = 50.0;
    const sim::FlowSummary forced = sim::summarize_flow(c, 1.0, aero::SolverKind::newtonian, false);
    CHECK(forced.solver == aero::SolverKind::newtonian);
    CHECK(forced.regime.overridden);
    CHECK(std::any_of(forced.warnings.begin(), forced.warnings.end(),
                      [](const aero::ValidityWarning& w) { return w.code == "newtonian.mach"; }));

    // Newtonian has no 2D form: the section runs the panel method and says so.
    const sim::FlowSummary section = sim::summarize_flow(c, 1.0, aero::SolverKind::newtonian, true);
    CHECK(section.solver == aero::SolverKind::panel);
    CHECK(section.warnings.front().code == "workspace.solver_unavailable");

    // Values no solver can use are input errors.
    c.density_kg_m3 = -1.0;
    const sim::FlowSummary bad = sim::summarize_flow(c, 1.0, std::nullopt, false);
    CHECK_FALSE(bad.runnable());
    CHECK(bad.input_errors.front() == "Density must be positive");

    CHECK(sim::sea_level_air(c).density_kg_m3 == 1.225);
    CHECK(sim::format_significant(1234567.0, 3) == "1.23e+06");
    CHECK(sim::format_significant(0.04125, 3) == "0.0413");
}

TEST_CASE("Direct solves fill a result record", "[aero][workspace]") {
    SECTION("2D panel method on NACA 0012 at 4 degrees: thin-airfoil lift") {
        sim::AeroRunSpec spec;
        spec.experiment = naca_section_experiment(4.0);
        spec.geometry = prepared(spec.experiment.geometry);
        const auto outcome = sim::solve(spec);
        REQUIRE(outcome.has_value());
        const sim::SolveOutcome& o = outcome.value();
        CHECK(o.finished);
        CHECK(o.solver == aero::SolverKind::panel);
        CHECK(o.result.cl == Approx(2.0 * kPi * 4.0 * kPi / 180.0).epsilon(0.15));
        const scene::AeroResultRecord r =
            sim::record_result(o.result, o.diagnostics, o.solver, o.flow, o.reference);
        CHECK(r.cl == o.result.cl);
        CHECK(r.reynolds == Approx(o.flow.reynolds));
        CHECK(r.reference_area_m2 == 1.0);
        CHECK(r.converged);
    }
    SECTION("Newtonian sphere at Mach 8") {
        sim::AeroRunSpec spec;
        spec.experiment.geometry = primitive(scene::AeroShape::sphere);
        spec.experiment.conditions.speed_m_s = 8.0 * 340.29;
        spec.geometry = prepared(spec.experiment.geometry);
        const auto outcome = sim::solve(spec);
        REQUIRE(outcome.has_value());
        CHECK(outcome.value().solver == aero::SolverKind::newtonian); // recommended
        CHECK(outcome.value().result.cd == Approx(0.9).margin(0.08));
        CHECK(outcome.value().result.stagnation_heat_flux.has_value());
    }
    SECTION("a lattice run stops at the step limit") {
        sim::AeroRunSpec spec;
        spec.experiment = tiny_lbm_experiment();
        spec.geometry = prepared(spec.experiment.geometry);
        const auto outcome = sim::solve(spec, 100);
        REQUIRE(outcome.has_value());
        CHECK_FALSE(outcome.value().finished);
        CHECK(outcome.value().diagnostics.lattice_steps == 100);
        CHECK(outcome.value().diagnostics.planned_steps > 100);
        CHECK(outcome.value().diagnostics.history_step.size() == 10);
        CHECK_FALSE(outcome.value().diagnostics.lattice.empty());
    }
}

TEST_CASE("The solver thread runs per frame, to convergence, pauses and stops",
          "[aero][workspace][thread]") {
    sim::AeroRunner runner;
    const auto latest = [&runner]() -> const sim::AeroProgress& {
        (void)runner.fetch();
        return runner.latest();
    };
    CHECK(latest().status.state == sim::AeroRunState::idle);

    SECTION("a direct solve finishes by itself") {
        sim::AeroRunSpec spec;
        spec.experiment = naca_section_experiment(2.0);
        spec.geometry = prepared(spec.experiment.geometry);
        runner.send(sim::StartAeroRun{spec, false});
        REQUIRE(app_test::wait_until(
            [&] { return latest().status.state == sim::AeroRunState::finished; }));
        REQUIRE(latest().result != nullptr);
        CHECK(latest().result->cl > 0.1);
        CHECK(latest().status.run == 1);
        CHECK(latest().spec->experiment.name == spec.experiment.name);
    }
    SECTION("a lattice run advances only when frames ask, until paused") {
        sim::AeroRunSpec spec;
        spec.experiment = tiny_lbm_experiment();
        spec.geometry = prepared(spec.experiment.geometry);
        runner.send(sim::StartAeroRun{spec, false});
        REQUIRE(app_test::wait_until(
            [&] { return latest().status.state == sim::AeroRunState::running; }));
        const std::uint64_t idle_calls = runner.step_calls();
        // No frames: no steps.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        CHECK(runner.step_calls() == idle_calls);
        runner.wake();
        REQUIRE(app_test::wait_until(
            [&] { return latest().diagnostics && latest().diagnostics->lattice_steps >= 50; }));
        CHECK(latest().diagnostics->lattice_steps % 50 == 0);
        runner.send(sim::SetAeroPaused{true});
        REQUIRE(app_test::wait_until(
            [&] { return latest().status.state == sim::AeroRunState::paused; }));
        const std::uint64_t paused_calls = runner.step_calls();
        runner.wake();
        runner.wake();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        CHECK(runner.step_calls() == paused_calls);
        // One frame on request, still paused.
        runner.send(sim::StepAeroFrame{});
        REQUIRE(app_test::wait_until([&] { return runner.step_calls() == paused_calls + 1; }));
        // Run to convergence: finishes without frames.
        runner.send(sim::SetAeroToConvergence{true});
        runner.send(sim::SetAeroPaused{false});
        REQUIRE(app_test::wait_until(
            [&] { return latest().status.state == sim::AeroRunState::finished; }));
        CHECK(latest().diagnostics->lattice_steps >= latest().diagnostics->planned_steps);
        runner.send(sim::StopAeroRun{});
        REQUIRE(
            app_test::wait_until([&] { return latest().status.state == sim::AeroRunState::idle; }));
        CHECK(latest().result != nullptr); // the last result stays readable
    }
    SECTION("a solver that cannot set up reports the failure") {
        sim::AeroRunSpec spec;
        spec.experiment = tiny_lbm_experiment();
        spec.experiment.solver.lbm_duration = 1.0e20; // beyond 2^53 lattice steps
        spec.geometry = prepared(spec.experiment.geometry);
        runner.send(sim::StartAeroRun{spec, true});
        REQUIRE(app_test::wait_until(
            [&] { return latest().status.state == sim::AeroRunState::failed; }));
        CHECK(latest().status.message.find("setup failed") != std::string::npos);
    }
}

TEST_CASE("Pause and Stop take effect between the solver calls of one frame",
          "[aero][workspace][thread]") {
    sim::AeroRunner runner;
    const auto latest = [&runner]() -> const sim::AeroProgress& {
        (void)runner.fetch();
        return runner.latest();
    };
    sim::AeroRunSpec spec;
    spec.experiment = tiny_lbm_experiment();
    spec.experiment.solver.lbm_duration = 5000.0;        // far more calls than one frame needs
    spec.experiment.solver.steps_per_frame = 100'000'000; // one frame: millions of calls
    spec.geometry = prepared(spec.experiment.geometry);
    runner.send(sim::StartAeroRun{spec, false});
    REQUIRE(
        app_test::wait_until([&] { return latest().status.state == sim::AeroRunState::running; }));
    // Waits for a frame's batch to make calls, then sends `command`; the
    // batch must end within a few calls, not run to its end.
    constexpr std::uint64_t kBound = 3;
    const auto interrupt = [&](sim::AeroCommand command, sim::AeroRunState expected) {
        const std::uint64_t before = runner.step_calls();
        runner.wake();
        REQUIRE(app_test::wait_until([&] { return runner.step_calls() >= before + 2; }));
        const std::uint64_t sent = runner.step_calls();
        runner.send(std::move(command));
        REQUIRE(app_test::wait_until([&] { return latest().status.state == expected; },
                                     std::chrono::seconds(10)));
        CHECK(runner.step_calls() - sent <= kBound);
    };
    interrupt(sim::SetAeroPaused{true}, sim::AeroRunState::paused);
    runner.send(sim::SetAeroPaused{false});
    REQUIRE(
        app_test::wait_until([&] { return latest().status.state == sim::AeroRunState::running; }));
    interrupt(sim::StopAeroRun{}, sim::AeroRunState::idle);
}

TEST_CASE("Comparison table: same geometry, different angles of attack",
          "[aero][workspace][experiment]") {
    std::vector<scene::AeroExperiment> runs;
    for (const double aoa : {4.0, 0.0, 2.0}) {
        scene::AeroExperiment e = naca_section_experiment(aoa);
        scene::AeroResultRecord r;
        r.cl = 0.1 * aoa;
        r.cd = 0.01 + 0.001 * aoa;
        r.cm = -0.01 * aoa;
        r.converged = true;
        e.result = r;
        runs.push_back(e);
    }
    scene::AeroExperiment other = naca_section_experiment(1.0);
    other.geometry.naca = "2412";
    other.name = "cambered, \"quoted\"";
    other.result = scene::AeroResultRecord{};
    runs.push_back(other);
    runs.push_back(naca_section_experiment(8.0)); // no result: left out

    const std::vector<sim::ComparisonTable> tables = sim::compare_by_geometry(runs);
    REQUIRE(tables.size() == 2);
    const sim::ComparisonTable& t = tables.front();
    CHECK(t.label == "NACA 0012 section, chord 1 m");
    REQUIRE(t.rows.size() == 3);
    CHECK(t.rows[0].angle_of_attack_deg == 0.0);
    CHECK(t.rows[2].angle_of_attack_deg == 4.0);
    CHECK(t.rows[2].delta_cl == Approx(0.4));
    CHECK(t.rows[2].delta_cd == Approx(0.004));
    CHECK(t.rows[1].lift_to_drag == Approx(0.2 / 0.012));
    REQUIRE(t.lift_slope_per_deg.has_value());
    CHECK(*t.lift_slope_per_deg == Approx(0.1));
    CHECK(*t.lift_slope_per_rad == Approx(0.1 * 180.0 / kPi));
    CHECK_FALSE(tables[1].lift_slope_per_deg.has_value()); // one row

    const std::string csv = sim::comparison_csv(t);
    CHECK(csv.starts_with("geometry,name,solver,aoa_deg"));
    CHECK(std::count(csv.begin(), csv.end(), '\n') == 4);
    CHECK(sim::comparison_csv(tables[1]).find("\"cambered, \"\"quoted\"\"\"") != std::string::npos);
}

TEST_CASE("Comparison CSV text cells cannot start a spreadsheet formula",
          "[aero][workspace][experiment]") {
    // Each name and the field the CSV must hold for it (between the label
    // and the solver columns).
    const std::vector<std::pair<std::string, std::string>> cases{
        {"=1+1", "\"'=1+1\""},
        {"+SUM(A1:A2)", "\"'+SUM(A1:A2)\""},
        {"-2+3", "\"'-2+3\""},
        {"@SUM(A1)", "\"'@SUM(A1)\""},
        {"\tindent", "\"'\tindent\""},
        {"\rreturn", "\"'\rreturn\""},
        {"=a,\"b\"", "\"'=a,\"\"b\"\"\""},
        {"plain name", "plain name"},
        {"a, b", "\"a, b\""},
        {"say \"hi\"", "\"say \"\"hi\"\"\""},
        {"x=1", "x=1"},
    };
    sim::ComparisonTable table;
    table.label = "=HYPERLINK(\"http://example.invalid\")";
    for (const auto& [name, field] : cases) {
        sim::ComparisonRow row;
        row.name = name;
        row.angle_of_attack_deg = -2.0; // numbers keep their sign
        table.rows = {row};
        const std::string csv = sim::comparison_csv(table);
        INFO(csv);
        const std::string label = "\"'=HYPERLINK(\"\"http://example.invalid\"\")\"";
        CHECK(csv.find("\r\n" + label + "," + field + ",panel,-2,") != std::string::npos);
    }
}

TEST_CASE("Experiment files save, list and load back", "[aero][workspace][experiment]") {
    const std::filesystem::path dir = scratch_dir("store");
    scene::AeroExperiment e = naca_section_experiment(4.0);
    e.name = "NACA 0012, 4 deg";
    const std::filesystem::path path = sim::new_experiment_path(dir, e.name);
    CHECK(path.filename() == "naca_0012_4_deg.json");
    REQUIRE(sim::save_aero_experiment_file(path, e).has_value());
    CHECK(sim::new_experiment_path(dir, e.name).filename() == "naca_0012_4_deg_2.json");
    CHECK(sim::new_experiment_path(dir, "  ").filename() == "experiment.json");

    // A space experiment is skipped, a broken aero file reported.
    std::filesystem::copy_file(app_test::experiment_dir() / "solar_system.json",
                               dir / "solar_system.json");
    {
        std::ofstream broken(dir / "broken.json");
        broken << R"({"version": 1, "domain": "aero", "conditions": {"speed_m_s": 0},
                     "solver": {"name": "auto"}})";
    }
    const sim::ExperimentListing listing = sim::list_aero_experiments(dir);
    REQUIRE(listing.experiments.size() == 1);
    CHECK(listing.experiments.front().experiment == e);
    REQUIRE(listing.problems.size() == 1);
    CHECK(listing.problems.front().starts_with("broken.json: conditions.speed_m_s"));

    const auto loaded = sim::load_aero_experiment_file(path);
    REQUIRE(loaded.has_value());
    CHECK(loaded.value() == e);
    CHECK_FALSE(sim::load_aero_experiment_file(dir / "missing.json").has_value());
}

TEST_CASE("User models: import errors are reported, units are confirmed",
          "[aero][workspace][import]") {
    const std::filesystem::path dir = scratch_dir("import");
    SECTION("missing file") {
        const auto missing = sim::load_model_file(dir / "nothing.stl", "");
        REQUIRE_FALSE(missing.has_value());
        CHECK(missing.error().title == "Model file not found");
        CHECK(missing.error().detail.starts_with("nothing.stl: "));
    }
    SECTION("a millimetre tetrahedron") {
        const std::filesystem::path stl = dir / "tetra.stl";
        {
            std::ofstream out(stl);
            out << "solid t\n";
            const double v[4][3] = {{0, 0, 0}, {1000, 0, 0}, {0, 1000, 0}, {0, 0, 1000}};
            const int faces[4][3] = {{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}};
            for (const auto& f : faces) {
                out << " facet normal 0 0 0\n  outer loop\n";
                for (const int i : f) {
                    out << "   vertex " << v[i][0] << ' ' << v[i][1] << ' ' << v[i][2] << '\n';
                }
                out << "  endloop\n endfacet\n";
            }
            out << "endsolid t\n";
        }
        sim::TaskProgress progress;
        const auto imported = sim::load_model_file(stl, "own drawing", &progress);
        REQUIRE(imported.has_value());
        const sim::ImportedModel& model = imported.value();
        CHECK(model.file_name == "tetra.stl");
        CHECK(model.imported.unit.unit == "mm");
        CHECK(model.ref.source == scene::GeometrySource::file);
        CHECK(model.ref.license_note == "own drawing");
        const auto ready = sim::prepare_user_model(model, 0.001, geom::UpAxis::z_up);
        REQUIRE(ready.has_value());
        const sim::PreparedGeometry& g = ready.value();
        CHECK(g.unvalidated);
        CHECK(g.report.watertight());
        CHECK(g.repair.has_value());
        CHECK(geom::bounds(g.mesh).extent().x == Approx(1.0));
        CHECK(g.ref.unit_scale == 0.001);
        CHECK_FALSE(sim::prepare_user_model(model, -1.0, geom::UpAxis::z_up).has_value());

        // The experiment path: the recorded scale and axis, relative path.
        scene::GeometryRef ref = g.ref;
        ref.path = "tetra.stl";
        const auto again = sim::load_experiment_geometry(ref, dir);
        REQUIRE(again.has_value());
        CHECK(again.value().ref.path == "tetra.stl");
        CHECK(geom::bounds(again.value().mesh).extent().x == Approx(1.0));
    }
    SECTION("a cancelled task stops before its next stage") {
        std::stop_source stop;
        sim::TaskProgress progress;
        progress.watch(stop.get_token());
        CHECK_FALSE(progress.stop_requested());
        stop.request_stop();
        CHECK(progress.stop_requested());
        const auto file = sim::load_model_file(dir / "nothing.stl", "", &progress);
        REQUIRE_FALSE(file.has_value());
        CHECK(file.error().title == "Cancelled");
        const auto url = sim::fetch_model_url("https://example.invalid/model.stl", {}, &progress);
        REQUIRE_FALSE(url.has_value());
        CHECK(url.error().title == "Cancelled");
        scene::GeometryRef ref;
        ref.source = scene::GeometrySource::file;
        ref.path = "nothing.stl";
        const auto geometry = sim::load_experiment_geometry(ref, dir, &progress);
        REQUIRE_FALSE(geometry.has_value());
        CHECK(geometry.error().title == "Cancelled");
    }
    SECTION("only https URLs are fetched") {
        const auto refused = sim::fetch_model_url("http://example.org/model.stl", {});
        REQUIRE_FALSE(refused.has_value());
        CHECK(refused.error().title == "URL refused");
        CHECK(refused.error().detail.starts_with("scheme_not_allowed"));
    }
}

TEST_CASE("View data places the lattice where the solver put it", "[aero][workspace][view]") {
    SECTION("2D: the section lies in the view's x-z plane") {
        sim::AeroRunSpec spec;
        spec.experiment = tiny_lbm_experiment();
        spec.geometry = prepared(spec.experiment.geometry);
        const sim::FlowSummary flow = spec.flow();
        const aero::ReferenceValues reference =
            sim::reference_values(*spec.geometry, spec.experiment.solver);
        auto solver = sim::make_solver(spec.plan(), spec.experiment.solver);
        solver->setup(sim::to_aero_geometry(*spec.geometry, spec.experiment.solver), flow.flow);
        (void)solver->step();
        const auto snapshot = sim::capture_flow(*solver, *spec.geometry, flow.flow, reference);
        REQUIRE(snapshot.has_value());
        REQUIRE(solver->result().velocity.has_value()); // zero angle of attack
        const aero::VelocityField& field = *solver->result().velocity;
        CHECK(snapshot.value().origin.x == Approx(field.origin.x).margin(1e-12));
        CHECK(snapshot.value().origin.y == 0.0);
        CHECK(snapshot.value().origin.z == Approx(field.origin.z).margin(1e-12));
        CHECK(snapshot.value().spacing == Approx(field.spacing));
        CHECK(snapshot.value().step == 50);
        const auto cp = sim::panel_cp(spec.geometry->section, solver->result().surface);
        CHECK(cp.size() == spec.geometry->section.size());
    }
    SECTION("3D: the wind frame of the lattice") {
        sim::AeroRunSpec spec;
        spec.experiment.geometry = primitive(scene::AeroShape::sphere);
        spec.experiment.geometry.dimensions.diameter_m = 0.1;
        spec.experiment.conditions.speed_m_s = 0.5;
        spec.experiment.conditions.viscosity_pa_s = 1.0e-3;
        spec.experiment.solver.solver = aero::SolverKind::lattice_boltzmann;
        spec.experiment.solver.lbm_cells_per_length = 6.0;
        spec.geometry = prepared(spec.experiment.geometry);
        const sim::FlowSummary flow = spec.flow();
        const aero::ReferenceValues reference =
            sim::reference_values(*spec.geometry, spec.experiment.solver);
        auto solver = sim::make_solver(spec.plan(), spec.experiment.solver);
        solver->setup(sim::to_aero_geometry(*spec.geometry, spec.experiment.solver), flow.flow);
        (void)solver->step();
        const auto snapshot = sim::capture_flow(*solver, *spec.geometry, flow.flow, reference);
        REQUIRE(snapshot.has_value());
        REQUIRE(solver->result().velocity.has_value());
        const aero::VelocityField& field = *solver->result().velocity;
        CHECK(snapshot.value().origin.x == Approx(field.origin.x).margin(1e-12));
        CHECK(snapshot.value().origin.y == Approx(field.origin.y).margin(1e-12));
        CHECK(snapshot.value().origin.z == Approx(field.origin.z).margin(1e-12));
        const auto cp = sim::triangle_cp(spec.geometry->mesh, solver->result().surface);
        CHECK(cp.size() == spec.geometry->mesh.triangles.size());
    }
    SECTION("frames are proper rotations") {
        scene::AeroConditions c;
        c.angle_of_attack_deg = 12.0;
        c.sideslip_deg = 3.0;
        for (const bool two_d : {false, true}) {
            const sim::ViewFrame frame = sim::view_frame(c.flow(), two_d);
            const geom::Vector3 x = sim::to_view({1, 0, 0}, frame);
            const geom::Vector3 y = sim::to_view({0, 1, 0}, frame);
            const geom::Vector3 z = sim::to_view({0, 0, 1}, frame);
            CHECK(geom::dot(geom::cross(x, y), z) == Approx(1.0));
            // The free stream runs along +x of the view.
            CHECK(sim::to_view(frame.drag, frame).x == Approx(1.0));
        }
    }
}

TEST_CASE("Background tasks hand over results and exceptions", "[aero][workspace][thread]") {
    sim::BackgroundTask<int> idle;
    CHECK_FALSE(idle.started());
    CHECK_FALSE(idle.done());

    sim::BackgroundTask<int> ok(
        [](sim::TaskProgress& progress) {
            progress.set_stage("working");
            return 42;
        },
        [](const std::string&) { return -1; });
    REQUIRE(app_test::wait_until([&] { return ok.done(); }));
    CHECK(ok.take() == 42);
    CHECK_FALSE(ok.started());

    sim::BackgroundTask<std::string> thrown(
        [](sim::TaskProgress&) -> std::string { throw std::runtime_error("disk on fire"); },
        [](const std::string& message) { return "failed: " + message; });
    REQUIRE(app_test::wait_until([&] { return thrown.done(); }));
    CHECK(thrown.take() == "failed: disk on fire");
}

TEST_CASE("Destroying a running background task cancels it without waiting for its work",
          "[aero][workspace][thread]") {
    using Clock = std::chrono::steady_clock;
    std::atomic<bool> entered{false};
    std::atomic<bool> saw_stop{false};
    const Clock::time_point start = Clock::now();
    {
        sim::BackgroundTask<int> blocked(
            [&entered, &saw_stop](sim::TaskProgress& progress) {
                // Stands for a long import or download that polls for
                // cancellation; without it the task would run for a minute.
                entered.store(true);
                const Clock::time_point deadline = Clock::now() + std::chrono::seconds(60);
                while (!progress.stop_requested() && Clock::now() < deadline) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
                saw_stop.store(progress.stop_requested());
                return 0;
            },
            [](const std::string&) { return -1; });
        REQUIRE(app_test::wait_until([&] { return entered.load(); }));
        CHECK_FALSE(blocked.done());
    } // shutdown: the destructor requests the stop and joins
    CHECK(saw_stop.load());
    CHECK(Clock::now() - start < std::chrono::seconds(20));
}
