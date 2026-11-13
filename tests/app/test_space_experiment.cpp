#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <new>
#include <string>
#include <variant>
#include <vector>

#include "../core/allocation_failure_injection.h"
#include "app_test_support.h"
#include "orrery/core/state_hash.h"
#include "orrery/sim/space_experiment.h"

using namespace orrery;
using namespace orrery::sim;

namespace {

std::filesystem::path fixture(const char* name) {
    return app_test::test_source_dir() / "fixtures" / name;
}

bool contains(const std::string& text, const char* part) {
    return text.find(part) != std::string::npos;
}

} // namespace

TEST_CASE("Every bundled experiment loads", "[app][sim][experiment]") {
    for (const char* name :
         {"solar_system", "sun_earth_moon", "figure_eight", "sgr_a_s2", "black_hole"}) {
        INFO(name);
        const std::array<std::filesystem::path, 1> dirs{app_test::experiment_dir()};
        const auto path = resolve_experiment(name, dirs);
        REQUIRE(path.has_value());
        const auto experiment = load_space_experiment(path.value());
        REQUIRE(experiment.has_value());
        CHECK_FALSE(experiment.value().scene.state.bodies().empty());
        CHECK(std::filesystem::exists(experiment.value().scenario_path));
        CHECK_FALSE(validate(experiment.value().config).has_value());
    }
    const auto solar = load_space_experiment(app_test::experiment_dir() / "solar_system.json");
    REQUIRE(solar.has_value());
    CHECK(solar.value().config.integrator == SpaceIntegrator::leapfrog_kdk);
    CHECK(solar.value().config.dt_days == 0.05);
    CHECK(solar.value().config.time_warp == 1'000'000);
    CHECK(solar.value().scenario_path.filename() == "solar_system_j2000.json");
}

TEST_CASE("Experiment settings are read from conditions and solver", "[app][sim][experiment]") {
    const auto loaded = load_space_experiment(fixture("custom_space.json"));
    REQUIRE(loaded.has_value());
    const SimulationConfig& config = loaded.value().config;
    CHECK(config.integrator == SpaceIntegrator::rk4);
    CHECK(config.force == ForceChoice::newtonian);
    CHECK(config.dt_days == 0.25);
    CHECK(config.softening_au == 0.001);
    CHECK(config.time_warp == 50'000);
    CHECK(config.max_steps_per_advance == 64);
    CHECK(config.paused);
    CHECK(loaded.value().scene.state.bodies().size() == 3);
}

TEST_CASE("Invalid experiments fail with a message naming the problem", "[app][sim][experiment]") {
    const auto expect_error = [](const char* name, const char* part) {
        INFO(name);
        const auto loaded = load_space_experiment(fixture(name));
        REQUIRE_FALSE(loaded.has_value());
        INFO(loaded.error());
        CHECK(contains(loaded.error(), name));
        CHECK(contains(loaded.error(), part));
    };
    expect_error("aero_domain.json", "domain 'aero'");
    expect_error("missing_scenario.json", "no_such_scenario.json");
    expect_error("unknown_integrator.json", "unknown integrator 'euler'");
    expect_error("unknown_setting.json", "solver.settings.dt: unknown key");
    expect_error("bad_warp.json", "conditions.time_warp");
    expect_error("no_scenario.json", "inputs.scenario");
    expect_error("syntax_error.json", "syntax");
    const auto missing = load_space_experiment(fixture("not_there.json"));
    REQUIRE_FALSE(missing.has_value());
    CHECK(contains(missing.error(), "not_there.json"));
}

TEST_CASE("Experiment names resolve through the search directories", "[app][sim][experiment]") {
    const std::vector<std::filesystem::path> dirs{app_test::asset_dir() / "nowhere",
                                                  app_test::experiment_dir()};
    const auto by_name = resolve_experiment("figure_eight", dirs);
    REQUIRE(by_name.has_value());
    CHECK(by_name.value().filename() == "figure_eight.json");
    const auto with_extension = resolve_experiment("figure_eight.json", dirs);
    REQUIRE(with_extension.has_value());
    CHECK(with_extension.value() == by_name.value());
    const auto by_path = resolve_experiment(fixture("custom_space.json").string(), dirs);
    REQUIRE(by_path.has_value());
    CHECK(by_path.value() == fixture("custom_space.json"));

    const auto unknown = resolve_experiment("andromeda", dirs);
    REQUIRE_FALSE(unknown.has_value());
    CHECK(contains(unknown.error(), "andromeda"));
    CHECK(contains(unknown.error(), "nowhere"));
    CHECK_FALSE(resolve_experiment("", dirs).has_value());
}

TEST_CASE("Scenario listing is sorted and limited to JSON files", "[app][sim][experiment]") {
    const std::vector<ScenarioEntry> scenarios = list_scenarios(app_test::scenario_dir());
    REQUIRE(scenarios.size() >= 5);
    for (std::size_t i = 1; i < scenarios.size(); ++i) {
        CHECK(scenarios[i - 1].name < scenarios[i].name);
    }
    CHECK(scenarios.front().path.extension() == ".json");
    CHECK(list_scenarios(app_test::asset_dir() / "nowhere").empty());
}

namespace {

// A fresh, empty directory under the temporary directory.
std::filesystem::path scratch(const char* name) {
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() / "orrery_tests" / name;
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}

} // namespace

TEST_CASE("A running space simulation saves as an experiment that loads with its settings",
          "[app][sim][experiment]") {
    const std::array<std::filesystem::path, 1> dirs{app_test::experiment_dir()};
    const auto source = resolve_experiment("figure_eight", dirs);
    REQUIRE(source.has_value());
    auto experiment = load_space_experiment(source.value());
    REQUIRE(experiment.has_value());
    auto created = Simulation::create(experiment.value().scene, experiment.value().config);
    REQUIRE(created.has_value());
    Simulation simulation = std::move(created).value();
    simulation.apply(SetTimeWarp{500'000});
    for (int i = 0; i < 20; ++i) {
        REQUIRE_FALSE(simulation.advance(16'666'667).failed);
    }
    const SimStatus& status = simulation.status();
    REQUIRE(status.steps > 0);

    const std::filesystem::path dir = scratch("space_save");
    const std::filesystem::path file = dir / "figure_8_test.json";
    const scene::Experiment record = space_record(experiment.value().scenario_path, status,
                                                  simulation.state().time().value(), dir);
    REQUIRE(save_space_experiment_file(file, record).has_value());
    CHECK_FALSE(std::filesystem::exists(dir / "figure_8_test.json.tmp"));
    // The scenario is referenced relative to the experiment when it can be.
    if (std::filesystem::absolute(dir).root_name() ==
        std::filesystem::absolute(experiment.value().scenario_path).root_name()) {
        const auto* reference = std::get_if<std::string>(&record.inputs.at("scenario"));
        REQUIRE(reference != nullptr);
        CHECK(std::filesystem::path(*reference).is_relative());
    }

    auto reloaded = load_space_experiment(file);
    REQUIRE(reloaded.has_value());
    const SimulationConfig& config = reloaded.value().config;
    CHECK(config.integrator == SpaceIntegrator::yoshida4);
    CHECK(config.dt_days == experiment.value().config.dt_days);
    CHECK(config.time_warp == 500'000);
    CHECK(config.force == experiment.value().config.force);
    CHECK(std::filesystem::equivalent(reloaded.value().scenario_path,
                                      experiment.value().scenario_path));
    CHECK(std::get<double>(record.result.at("steps")) == static_cast<double>(status.steps));

    // Listed with a summary; a broken space file is named, other domains
    // are skipped.
    {
        std::ofstream broken(dir / "broken.json");
        broken << R"({"version":1,"domain":"space","inputs":{},"conditions":{},)"
               << R"("solver":{"name":"leapfrog_kdk","version":"1","settings":{}},"result":{}})";
    }
    std::filesystem::copy_file(app_test::experiment_dir() / "entry_apollo_like.json",
                               dir / "entry.json");
    const SpaceListing listing = list_space_experiments(dir);
    REQUIRE(listing.entries.size() == 1);
    CHECK(listing.entries.front().name == "figure_8_test");
    CHECK(contains(listing.entries.front().summary, "figure_eight, yoshida4"));
    REQUIRE(listing.problems.size() == 1);
    CHECK(contains(listing.problems.front(), "broken.json"));
    CHECK(list_space_experiments(dir / "missing").entries.empty());
}

TEST_CASE("LoadExperiment replaces the system and its settings, or nothing",
          "[app][sim][experiment]") {
    SimulationConfig config;
    config.step_time_budget_s = 0.25;
    config.warp_budget = WarpBudget{};
    Simulation simulation = app_test::make_simulation("solar_system_j2000.json", config);
    const std::uint64_t generation = simulation.status().generation;

    simulation.apply(LoadExperiment{app_test::experiment_dir() / "figure_eight.json"});
    const SimStatus& loaded = simulation.status();
    CHECK(loaded.message.empty());
    CHECK(loaded.generation == generation + 1);
    CHECK(loaded.settings.integrator == SpaceIntegrator::yoshida4);
    CHECK(loaded.dt_days == 0.5);
    CHECK(loaded.time_warp == 2'000'000);
    CHECK(loaded.governed_warp == 2'000'000);
    CHECK(loaded.load_limit == 0.6); // the interactive budget stays
    CHECK(simulation.config().step_time_budget_s == 0.25);
    CHECK(loaded.experiment_file.filename() == "figure_eight.json");
    CHECK(loaded.scenario_file.filename() == "figure_eight.json");
    CHECK(simulation.state().bodies().size() == 3);

    simulation.apply(LoadExperiment{fixture("unknown_integrator.json")});
    CHECK(contains(simulation.status().message, "Cannot open experiment unknown_integrator.json"));
    CHECK(simulation.status().generation == generation + 1);
    CHECK(simulation.status().settings.integrator == SpaceIntegrator::yoshida4);
}

namespace {

constexpr std::int64_t kFrameNs = 16'666'667;

// The interactive setup: a warp budget and a clock that never advances
// (every step is free, so the governor is deterministic).
SimulationConfig governed_config() {
    SimulationConfig config;
    config.dt_days = 0.05;
    config.time_warp = 1'000'000;
    config.warp_budget = WarpBudget{};
    config.clock = [] { return std::int64_t{0}; };
    return config;
}

struct Observed {
    std::uint64_t hash = 0;
    SimStatus status;
    SpaceIntegrator integrator = SpaceIntegrator::leapfrog_kdk;
    ForceChoice force = ForceChoice::automatic;
    double dt_days = 0.0;
    double softening_au = 0.0;
    std::int64_t time_warp = 0;
    bool paused = false;
    std::int64_t max_steps = 0;
};

Observed observe(Simulation& simulation) {
    const SimulationConfig& config = simulation.config();
    return {core::state_hash(simulation.state()),
            simulation.status(),
            config.integrator,
            config.force,
            config.dt_days,
            config.softening_au,
            config.time_warp,
            config.paused,
            config.max_steps_per_advance};
}

void check_same(const Observed& a, const Observed& b) {
    CHECK(a.hash == b.hash);
    CHECK(a.status == b.status);
    CHECK(a.integrator == b.integrator);
    CHECK(a.force == b.force);
    CHECK(a.dt_days == b.dt_days);
    CHECK(a.softening_au == b.softening_au);
    CHECK(a.time_warp == b.time_warp);
    CHECK(a.paused == b.paused);
    CHECK(a.max_steps == b.max_steps);
}

} // namespace

TEST_CASE("A LoadExperiment that fails on an allocation changes nothing",
          "[app][sim][experiment][alloc]") {
    const auto started = [] {
        Simulation simulation =
            app_test::make_simulation("sun_earth_moon.json", governed_config());
        for (int i = 0; i < 3; ++i) {
            REQUIRE(simulation.advance(kFrameNs).steps > 0);
        }
        return simulation;
    };
    // What an untouched simulation does next.
    Simulation reference = started();
    const Observed before = observe(reference);
    for (int i = 0; i < 4; ++i) {
        REQUIRE(reference.advance(kFrameNs).steps > 0);
    }
    const Observed after = observe(reference);

    // Every allocation of the load fails once in turn (custom_space.json
    // changes the integrator, force, step, warp, softening, budget and pause).
    int failures = 0;
    for (long long allowed = 0;; ++allowed) {
        Simulation simulation = started();
        bool fired = false;
        try {
            const test::AllocationFailureScope scope(allowed);
            simulation.apply(LoadExperiment{fixture("custom_space.json")});
            fired = scope.fired();
        } catch (const std::bad_alloc&) {
            fired = true;
        }
        if (!fired) {
            // The whole load succeeded without an injected failure.
            CHECK(simulation.config().integrator == SpaceIntegrator::rk4);
            CHECK(simulation.status().dt_days == 0.25);
            break;
        }
        ++failures;
        INFO("allocation " << allowed << " failed");
        Observed now = observe(simulation);
        // A failure reported as a message is the only allowed difference.
        now.status.message = before.status.message;
        check_same(now, before);
        for (int i = 0; i < 4; ++i) {
            REQUIRE(simulation.advance(kFrameNs).steps > 0);
        }
        Observed later = observe(simulation);
        later.status.message = after.status.message;
        check_same(later, after);
    }
    CHECK(failures > 10);
}
