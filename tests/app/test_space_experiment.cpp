#include <catch2/catch_test_macros.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <new>
#include <string>
#include <system_error>
#include <thread>
#include <variant>
#include <vector>

#if !defined(_WIN32)
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#include "../core/allocation_failure_injection.h"
#include "app_test_support.h"
#include "orrery/core/state_hash.h"
#include "orrery/sim/physics_thread.h"
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
    // The S2 scenario is the Newtonian point-mass model its title and its
    // period test use; "auto" would pick Paczynski-Wiita for the black hole.
    const auto s2 = load_space_experiment(app_test::experiment_dir() / "sgr_a_s2.json");
    REQUIRE(s2.has_value());
    CHECK(s2.value().config.force == ForceChoice::newtonian);
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

// A directory that exists but cannot be listed, or empty when this system
// offers none (POSIX: mode 000, unless the tests run as root; Windows: the
// System Volume Information of the drive holding the temporary directory).
std::filesystem::path unlistable_directory() {
#if defined(_WIN32)
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path().root_path() / "System Volume Information";
#else
    const std::filesystem::path dir = scratch("unlistable") / "locked";
    std::filesystem::create_directories(dir);
    std::filesystem::permissions(dir, std::filesystem::perms::none);
#endif
    std::error_code error;
    const std::filesystem::directory_iterator probe(dir, error);
    return error ? dir : std::filesystem::path();
}

} // namespace

TEST_CASE("Experiment folder errors are problems, a missing folder is not",
          "[app][sim][experiment][listing]") {
    const std::filesystem::path dir = scratch("listing_errors");
    const SpaceListing missing = list_space_experiments(dir / "missing");
    CHECK(missing.entries.empty());
    CHECK(missing.problems.empty());

    {
        std::ofstream(dir / "not_a_folder") << "a file";
    }
    const SpaceListing file = list_space_experiments(dir / "not_a_folder");
    CHECK(file.entries.empty());
    REQUIRE(file.problems.size() == 1);
    CHECK(contains(file.problems.front(), "not_a_folder"));
    CHECK(contains(file.problems.front(), "not a directory"));

    const std::filesystem::path locked = unlistable_directory();
    if (locked.empty()) {
        SKIP("no unlistable directory on this system");
    }
    const SpaceListing denied = list_space_experiments(locked);
#if !defined(_WIN32)
    std::filesystem::permissions(locked, std::filesystem::perms::owner_all);
#endif
    CHECK(denied.entries.empty());
    REQUIRE(denied.problems.size() == 1);
    CHECK(contains(denied.problems.front(), locked.filename().string().c_str()));
    CHECK(contains(denied.problems.front(), "cannot be listed"));
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

namespace {

// An experiment in `dir` whose scenario is `scenario` (relative to dir).
std::filesystem::path experiment_using(const std::filesystem::path& dir, const char* name,
                                       const char* scenario) {
    const std::filesystem::path file = dir / name;
    std::ofstream out(file, std::ios::binary);
    out << R"({"version":1,"domain":"space","inputs":{"scenario":")" << scenario
        << R"("},"conditions":{},"solver":{"name":"leapfrog_kdk","version":"1",)"
        << R"("settings":{}},"result":{}})";
    return file;
}

} // namespace

TEST_CASE("Files that are not regular files are refused before they are opened",
          "[app][sim][experiment][special]") {
    const std::filesystem::path dir = scratch("special_files");
    std::filesystem::create_directories(dir / "folder.json");
    const std::filesystem::path experiment =
        experiment_using(dir, "uses_folder.json", "folder.json");

    const auto loaded = load_space_experiment(experiment);
    REQUIRE_FALSE(loaded.has_value());
    CHECK(contains(loaded.error(), "folder.json"));
    CHECK(contains(loaded.error(), "not a regular file"));
    const auto direct = load_space_experiment(dir / "folder.json");
    REQUIRE_FALSE(direct.has_value());
    CHECK(contains(direct.error(), "not a regular file"));

    const SpaceListing listing = list_space_experiments(dir);
    CHECK(listing.entries.empty());
    REQUIRE(listing.problems.size() == 1);
    CHECK(contains(listing.problems.front(), "not a regular file"));

    SimulationConfig config;
    Simulation simulation = app_test::make_simulation("sun_earth.json", config);
    const std::uint64_t generation = simulation.status().generation;
    simulation.apply(LoadScenario{dir / "folder.json"});
    CHECK(contains(simulation.status().message, "not a regular file"));
    simulation.apply(LoadExperiment{experiment});
    CHECK(contains(simulation.status().message, "not a regular file"));
    CHECK(simulation.status().generation == generation);
}

#if !defined(_WIN32)
namespace {

// Runs `call` on a thread of its own. When it has not returned within 10 s
// (blocked opening the FIFO for reading), writers are opened on the FIFO
// until it returns, to release every blocked reader; false then.
bool returns_in_time(const std::filesystem::path& fifo, const std::function<void()>& call) {
    std::promise<void> done;
    std::future<void> finished = done.get_future();
    std::thread worker([&] {
        call();
        done.set_value();
    });
    const bool in_time = finished.wait_for(std::chrono::seconds(10)) == std::future_status::ready;
    while (finished.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
        const int writer = ::open(fifo.c_str(), O_WRONLY | O_NONBLOCK);
        if (writer >= 0) {
            (void)::close(writer);
        }
    }
    worker.join();
    return in_time;
}

} // namespace

TEST_CASE("A FIFO named as a scenario neither blocks listing nor loading",
          "[app][sim][experiment][special]") {
    const std::filesystem::path dir = scratch("fifo");
    const std::filesystem::path fifo = dir / "pipe.json";
    REQUIRE(::mkfifo(fifo.c_str(), 0600) == 0);
    const std::filesystem::path experiment = experiment_using(dir, "uses_pipe.json", "pipe.json");

    std::string load_error;
    CHECK(returns_in_time(fifo, [&] {
        const auto loaded = load_space_experiment(experiment);
        load_error = loaded.has_value() ? std::string("loaded") : loaded.error();
    }));
    CHECK(contains(load_error, "not a regular file"));

    SpaceListing listing;
    CHECK(returns_in_time(fifo, [&] { listing = list_space_experiments(dir); }));
    CHECK(listing.entries.empty());
    REQUIRE(listing.problems.size() == 1);
    CHECK(contains(listing.problems.front(), "not a regular file"));

    // On the physics thread: reported, and the thread still shuts down.
    std::string message;
    CHECK(returns_in_time(fifo, [&] {
        sim::PhysicsThread physics(app_test::make_simulation("sun_earth.json", SimulationConfig{}));
        physics.send(LoadExperiment{experiment});
        physics.send(LoadScenario{fifo});
        (void)app_test::wait_until(
            [&] {
                physics.wake();
                if (physics.fetch()) {
                    message = physics.latest().status.message;
                }
                return contains(message, "pipe.json") && contains(message, "Cannot load scenario");
            },
            std::chrono::seconds(8));
    }));
    CHECK(contains(message, "not a regular file"));
}
#endif

TEST_CASE("A LoadExperiment that fails on an allocation changes nothing",
          "[app][sim][experiment][alloc]") {
    const auto started = [] {
        Simulation simulation = app_test::make_simulation("sun_earth_moon.json", governed_config());
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
