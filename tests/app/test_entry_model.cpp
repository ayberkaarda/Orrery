#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "app_test_support.h"
#include "orrery/atmo/planet.h"
#include "orrery/scene/experiment.h"
#include "orrery/sim/entry_backend.h"
#include "orrery/sim/entry_comparison.h"
#include "orrery/sim/entry_experiments.h"
#include "orrery/sim/entry_model.h"
#include "orrery/sim/entry_runner.h"

// Phase 9b: the graphics-free side of the Entry workspace: input checks, the
// periapsis form of the initial state, summaries and interpolation, the
// solver thread (pause and stop between slices, failures reported) and the
// comparison table with its CSV export.

using namespace orrery;
using Catch::Approx;

TEST_CASE("Entry setup checks name every value no run can use", "[app][entry]") {
    sim::EntrySetup setup;
    CHECK(sim::setup_errors(setup).empty());
    setup.mass_kg = 0.0;
    setup.reference_area_m2 = std::numeric_limits<double>::quiet_NaN();
    setup.cd_table = {{2.0, 1.2}, {1.0, 1.1}};
    const std::vector<std::string> errors = sim::setup_errors(setup);
    REQUIRE(errors.size() == 3);
    CHECK(errors[0].find("Mass") != std::string::npos);
    CHECK(errors[1].find("Reference area") != std::string::npos);
    CHECK(errors[2].find("increasing Mach") != std::string::npos);

    sim::EntrySetup empty_table;
    empty_table.cd_table.clear();
    CHECK_FALSE(sim::setup_errors(empty_table).empty());
    sim::EntrySetup steep;
    steep.flight_path_angle_deg = -90.0;
    CHECK_FALSE(sim::setup_errors(steep).empty());
}

TEST_CASE("The periapsis form gives the flight-path angle of the same orbit", "[app][entry]") {
    const double altitude = 121'920.0;
    const double speed = 7'900.0;
    const auto gamma =
        sim::flight_path_angle_for_periapsis(sim::EntryPlanet::earth, altitude, speed, 30'000.0);
    REQUIRE(gamma.has_value());
    CHECK(gamma.value_or(0.0) < 0.0);
    const auto periapsis =
        sim::periapsis_altitude(sim::EntryPlanet::earth, altitude, speed, gamma.value_or(0.0));
    REQUIRE(periapsis.has_value());
    CHECK(periapsis.value_or(0.0) == Approx(30'000.0).margin(1.0));
    // A periapsis above the start altitude has no descending solution.
    CHECK_FALSE(
        sim::flight_path_angle_for_periapsis(sim::EntryPlanet::earth, altitude, speed, 200'000.0)
            .has_value());
    sim::EntrySetup setup;
    setup.start = sim::EntryStart::periapsis;
    setup.periapsis_altitude_m = 500'000.0;
    CHECK_FALSE(sim::setup_errors(setup).empty());
}

TEST_CASE("Entry summaries find peaks, the heat load and interpolate", "[app][entry]") {
    sim::EntrySeries series;
    for (int i = 0; i <= 10; ++i) {
        sim::EntrySample s;
        s.time_s = i;
        s.g_load = i <= 5 ? i : 10 - i;
        s.heat_flux_w_m2 = 100.0;
        s.downrange_m = 1000.0 * i;
        series.push(s);
    }
    REQUIRE(series.consistent());
    const sim::EntrySummary summary = sim::summarize(series, sim::EntryOutcome::landed);
    CHECK(summary.peak_g == 5.0);
    CHECK(summary.peak_g_time_s == 5.0);
    CHECK(summary.heat_load_j_m2 == Approx(1000.0));
    CHECK(summary.duration_s == 10.0);
    CHECK(summary.downrange_m == 10'000.0);
    CHECK(sim::value_at(series.time_s, series.g_load, 4.5).value_or(-1.0) == Approx(4.5));
    CHECK(sim::value_at(series.time_s, series.g_load, -3.0).value_or(-1.0) == 0.0);
    CHECK(sim::nearest_index(series.time_s, 7.4).value_or(99) == 7);
    CHECK_FALSE(sim::value_at(series.time_s, series.mach, std::nan("")).has_value());
    series.mach.pop_back();
    CHECK_FALSE(series.consistent());
    CHECK(sim::summarize(series, sim::EntryOutcome::landed).peak_g == 0.0);
}

TEST_CASE("A steep Earth entry lands and a grazing one skips out", "[app][entry]") {
    sim::EntrySetup steep;
    steep.flight_path_angle_deg = -6.0;
    auto landed = sim::run_entry(steep);
    REQUIRE(landed.has_value());
    const sim::EntrySummary& l = landed.value().summary;
    CHECK(l.outcome == sim::EntryOutcome::landed);
    CHECK(l.peak_g > 2.0);
    CHECK(l.peak_heat_flux_w_m2 > 1.0e5);
    CHECK(landed.value().series.altitude_m.back() == Approx(0.0).margin(1.0));

    sim::EntrySetup grazing;
    grazing.speed_m_s = 10'500.0;
    grazing.flight_path_angle_deg = -1.0;
    auto skipped = sim::run_entry(grazing);
    REQUIRE(skipped.has_value());
    CHECK(skipped.value().summary.outcome == sim::EntryOutcome::skip_out);
}

namespace {

// A stepper that takes `wall` per call and never ends, to test pacing.
class SlowStepper final : public sim::EntryStepper {
public:
    explicit SlowStepper(std::chrono::milliseconds wall, bool fail_on_third = false)
        : wall_(wall), fail_(fail_on_third) {}
    bool advance(double seconds, sim::EntrySeries& out) override {
        std::this_thread::sleep_for(wall_);
        if (fail_ && ++calls_ == 3) {
            throw std::runtime_error("density table exhausted");
        }
        t_ += seconds;
        sim::EntrySample s;
        s.time_s = t_;
        s.altitude_m = 1.0e5;
        out.push(s);
        return false;
    }
    [[nodiscard]] sim::EntryOutcome outcome() const noexcept override {
        return sim::EntryOutcome::none;
    }
    [[nodiscard]] double time_s() const noexcept override { return t_; }

private:
    std::chrono::milliseconds wall_;
    bool fail_ = false;
    int calls_ = 0;
    double t_ = 0.0;
};

sim::EntryStepperFactory slow_factory(std::chrono::milliseconds wall, bool fail = false) {
    return [wall, fail](const sim::EntrySetup&)
               -> core::Result<std::unique_ptr<sim::EntryStepper>, std::string> {
        std::unique_ptr<sim::EntryStepper> stepper = std::make_unique<SlowStepper>(wall, fail);
        return stepper;
    };
}

bool wait_for(sim::EntryRunner& runner, sim::EntryRunState state) {
    return app_test::wait_until([&] {
        runner.wake();
        (void)runner.fetch();
        return runner.latest().status.state == state;
    });
}

} // namespace

TEST_CASE("The entry runner runs to the end on its own thread", "[app][entry]") {
    sim::EntryRunner runner;
    sim::EntrySetup setup;
    setup.flight_path_angle_deg = -6.0;
    runner.send(sim::StartEntryRun{setup});
    REQUIRE(wait_for(runner, sim::EntryRunState::finished));
    const sim::EntryProgress& p = runner.latest();
    REQUIRE(p.trace != nullptr);
    CHECK(p.trace->summary.outcome == sim::EntryOutcome::landed);
    CHECK(p.trace->series.size() > 100);
    REQUIRE(p.setup != nullptr);
    CHECK(*p.setup == setup);
}

TEST_CASE("Pause and stop act between slices of the entry runner", "[app][entry]") {
    sim::EntryRunner runner(slow_factory(std::chrono::milliseconds(20)));
    runner.send(sim::StartEntryRun{sim::EntrySetup{}});
    REQUIRE(wait_for(runner, sim::EntryRunState::running));
    REQUIRE(app_test::wait_until([&] { return runner.slices() >= 2; }));
    runner.send(sim::SetEntryPaused{true});
    REQUIRE(wait_for(runner, sim::EntryRunState::paused));
    const std::uint64_t paused_at = runner.slices();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    CHECK(runner.slices() <= paused_at + 1);
    runner.send(sim::SetEntryPaused{false});
    REQUIRE(app_test::wait_until([&] { return runner.slices() > paused_at + 2; }));
    const auto sent = std::chrono::steady_clock::now();
    runner.send(sim::StopEntryRun{});
    REQUIRE(wait_for(runner, sim::EntryRunState::stopped));
    // Within a slice or two (20 ms each), not the whole 12 ms batch of many.
    CHECK(std::chrono::steady_clock::now() - sent < std::chrono::milliseconds(500));
    const std::uint64_t stopped_at = runner.slices();
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    CHECK(runner.slices() == stopped_at);
    // The trace up to the stop stays readable.
    REQUIRE(runner.latest().trace != nullptr);
    CHECK_FALSE(runner.latest().trace->series.empty());
}

TEST_CASE("Stop and shutdown stay prompt with a tiny integration step", "[app][entry]") {
    // 1e-9 s steps: one 2 s slice would be 2e9 steps if a stepper call were
    // not bounded.
    sim::EntrySetup setup;
    setup.integration.time_step_s = 1.0e-9;
    auto runner = std::make_unique<sim::EntryRunner>();
    const auto within = [&](const std::function<bool()>& condition) {
        const bool reached = app_test::wait_until(condition, std::chrono::seconds(5));
        if (!reached) {
            // The solver thread is stuck inside one stepper call: leave it
            // behind instead of blocking this test in the destructor's join.
            (void)runner.release();
        }
        return reached;
    };
    const auto prompt = [&](sim::EntryRunState state) {
        return within([&] {
            runner->wake();
            (void)runner->fetch();
            return runner->latest().status.state == state;
        });
    };
    runner->send(sim::StartEntryRun{setup});
    REQUIRE(prompt(sim::EntryRunState::running));
    REQUIRE(within([&] { return runner->slices() >= 1; }));
    const auto sent = std::chrono::steady_clock::now();
    runner->send(sim::StopEntryRun{});
    REQUIRE(prompt(sim::EntryRunState::stopped));
    CHECK(std::chrono::steady_clock::now() - sent < std::chrono::seconds(1));
    // Shutdown in the middle of a run joins promptly too.
    runner->send(sim::StartEntryRun{setup});
    REQUIRE(prompt(sim::EntryRunState::running));
    const auto destroyed = std::chrono::steady_clock::now();
    runner.reset();
    CHECK(std::chrono::steady_clock::now() - destroyed < std::chrono::seconds(1));
}

TEST_CASE("Entry runner failures are published with their reason", "[app][entry]") {
    sim::EntryRunner runner(slow_factory(std::chrono::milliseconds(1), true));
    runner.send(sim::StartEntryRun{sim::EntrySetup{}});
    REQUIRE(wait_for(runner, sim::EntryRunState::failed));
    CHECK(runner.latest().status.message.find("density table exhausted") != std::string::npos);
    REQUIRE(runner.latest().trace != nullptr); // what was integrated stays visible

    sim::EntryRunner real;
    sim::EntrySetup invalid;
    invalid.mass_kg = -1.0;
    real.send(sim::StartEntryRun{invalid});
    REQUIRE(wait_for(real, sim::EntryRunState::failed));
    CHECK(real.latest().status.message.find("Mass") != std::string::npos);
    CHECK(real.latest().trace == nullptr); // no stale trace of an earlier run
}

TEST_CASE("Entry comparison sorts by flight-path angle and escapes CSV cells", "[app][entry]") {
    std::vector<sim::EntryComparisonInput> inputs(3);
    inputs[0].name = "shallow";
    inputs[0].setup.flight_path_angle_deg = -1.5;
    inputs[0].summary.outcome = sim::EntryOutcome::skip_out;
    inputs[1].name = "=HYPERLINK(\"x\")";
    inputs[1].setup.flight_path_angle_deg = -6.0;
    inputs[1].summary.peak_g = 7.25;
    inputs[2].name = "-mid, \"quoted\"";
    inputs[2].setup.flight_path_angle_deg = -3.0;
    const std::vector<sim::EntryComparisonRow> rows = sim::compare_entries(inputs);
    REQUIRE(rows.size() == 3);
    CHECK(rows[0].flight_path_angle_deg == -6.0);
    CHECK(rows[2].name == "shallow");
    const std::string csv = sim::entry_comparison_csv(rows);
    CHECK(csv.rfind("name,planet,mass_kg,", 0) == 0);
    CHECK(csv.find(R"csv("'=HYPERLINK(""x"")",earth)csv") != std::string::npos);
    CHECK(csv.find(R"csv("'-mid, ""quoted""",earth)csv") != std::string::npos);
    CHECK(csv.find(",7.25,") != std::string::npos);
    CHECK(csv.find("shallow,earth,") != std::string::npos);
    CHECK(csv.find(",skip_out,") != std::string::npos);
    for (const char lead : {'+', '@', '\t', '\r'}) {
        const std::string cell = sim::csv_text_cell(std::string(1, lead) + "x");
        CHECK(cell.rfind("\"'", 0) == 0);
    }
    CHECK(sim::csv_text_cell("plain") == "plain");
}

TEST_CASE("Entry records round-trip through the experiment JSON", "[app][entry]") {
    sim::EntrySetup setup = sim::entry_presets().back().setup; // Mars, its own Cd table
    setup.atmosphere = sim::EntryAtmosphere::exponential;
    setup.exponential_density_kg_m3 = 0.0158;
    setup.exponential_scale_height_m = 11'100.0;
    setup.start = sim::EntryStart::periapsis;
    setup.periapsis_altitude_m = 21'500.0;
    setup.cd_source = sim::CdSource::aero_solver;
    setup.co_rotating_atmosphere = true;
    setup.integration.integrator = sim::EntryIntegrator::yoshida4;
    setup.integration.time_step_s = 0.25;
    setup.integration.interface_altitude_m = 130'000.0;
    auto run = sim::run_entry(setup);
    INFO((run ? std::string() : run.error()));
    REQUIRE(run.has_value());
    const sim::EntryTrace& trace = run.value();
    REQUIRE(trace.record != nullptr);

    const scene::Experiment record = sim::entry_to_experiment(setup, &trace);
    CHECK(record.domain == scene::Domain::entry);
    const std::string json = scene::save_experiment(record);
    auto parsed = scene::parse_experiment(json);
    REQUIRE(parsed.has_value());
    CHECK(parsed.value() == record);
    auto back = sim::entry_from_experiment(parsed.value());
    INFO((back ? std::string() : back.error()));
    REQUIRE(back.has_value());
    const sim::StoredEntry& stored = back.value();
    // The periapsis form comes back as the flight-path angle the run used.
    sim::EntrySetup resolved = setup;
    resolved.start = sim::EntryStart::state;
    resolved.flight_path_angle_deg = sim::effective_flight_path_angle(setup).value_or(0.0);
    // ... with the periapsis altitude derived from it again.
    CHECK(stored.setup.periapsis_altitude_m == Approx(setup.periapsis_altitude_m).margin(1.0e-3));
    resolved.periapsis_altitude_m = stored.setup.periapsis_altitude_m;
    CHECK(stored.setup == resolved);
    REQUIRE(stored.result != nullptr);
    CHECK(stored.result->series.time_s == trace.series.time_s);
    CHECK(stored.result->series.heat_flux_w_m2 == trace.series.heat_flux_w_m2);
    CHECK(stored.result->series.downrange_m == trace.series.downrange_m);
    CHECK(stored.result->summary == trace.summary);
    REQUIRE(stored.result->record != nullptr);
    CHECK(*stored.result->record == *trace.record);
    // The record runs again to the same result.
    auto again = sim::run_entry(stored.setup);
    REQUIRE(again.has_value());
    CHECK(again.value().summary == trace.summary);

    // Inputs only: no result section, nothing recorded.
    auto inputs_only = sim::entry_from_experiment(sim::entry_to_experiment(setup, nullptr));
    REQUIRE(inputs_only.has_value());
    CHECK(inputs_only.value().result == nullptr);

    // Problems name the key.
    scene::Experiment broken = record;
    broken.inputs["planet"] = std::string("venus");
    auto rejected = sim::entry_from_experiment(broken);
    REQUIRE_FALSE(rejected.has_value());
    CHECK(rejected.error().find("inputs.planet") != std::string::npos);
    broken = record;
    broken.series["g_load"].pop_back();
    CHECK_FALSE(sim::entry_from_experiment(broken).has_value());
    broken = record;
    broken.series["cd_table_cd"].clear();
    CHECK_FALSE(sim::entry_from_experiment(broken).has_value());
    broken = record;
    broken.domain = scene::Domain::aero;
    CHECK_FALSE(sim::entry_from_experiment(broken).has_value());
}

TEST_CASE("Entry planet constants are the physics library's presets", "[app][entry]") {
    for (const sim::EntryPlanet planet : {sim::EntryPlanet::earth, sim::EntryPlanet::mars}) {
        const atmo::Planet preset =
            planet == sim::EntryPlanet::mars ? atmo::Planet::mars() : atmo::Planet::earth();
        const sim::PlanetConstants c = sim::planet_constants(planet);
        CHECK(c.radius_m == preset.radius.value());
        CHECK(c.mu_m3_s2 == preset.gm.value());
        CHECK(c.atmosphere_top_m == preset.entry_interface_altitude.value());
    }
}

TEST_CASE("Cd(Mach) is sampled from the aero solver on a capsule", "[app][entry]") {
    sim::CdSampling sampling;
    sampling.reference_area_m2 = 12.02;
    auto samples = sim::sample_cd_table(sampling);
    INFO((samples ? std::string() : samples.error()));
    REQUIRE(samples.has_value());
    const std::vector<sim::MachCd>& table = samples.value().table;
    REQUIRE(table.size() == 5);
    for (std::size_t i = 0; i < table.size(); ++i) {
        CHECK(table[i].cd > 1.0);
        CHECK(table[i].cd < 1.8);
        CHECK((i == 0 || table[i].mach > table[i - 1].mach));
    }
    CHECK(samples.value().label.find("capsule") != std::string::npos);
    sampling.reference_area_m2 = 0.0;
    CHECK_FALSE(sim::sample_cd_table(sampling).has_value());
}