#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "app_test_support.h"
#include "orrery/core/event.h"
#include "orrery/core/event_step.h"
#include "orrery/core/force_model.h"
#include "orrery/core/integrator.h"
#include "orrery/core/state_hash.h"
#include "orrery/sim/simulation.h"

using namespace orrery;
using namespace orrery::sim;
using Catch::Approx;

namespace {

constexpr std::int64_t kFrameNs = 16'666'667;

SimulationConfig solar_config() {
    SimulationConfig config;
    config.dt_days = 0.05;
    config.time_warp = 1'000'000;
    return config;
}

// Deterministic pseudo-random frame lengths in [1 ms, 50 ms] summing to
// exactly `total_ns`.
std::vector<std::int64_t> irregular_frames(std::int64_t total_ns, std::uint64_t seed) {
    std::vector<std::int64_t> frames;
    std::uint64_t state = seed;
    std::int64_t remaining = total_ns;
    while (remaining > 0) {
        state = state * 6364136223846793005ULL + 1442695040888963407ULL;
        const auto length = static_cast<std::int64_t>(1'000'000 + (state >> 33) % 49'000'000);
        frames.push_back(std::min(length, remaining));
        remaining -= frames.back();
    }
    return frames;
}

struct RunResult {
    std::uint64_t hash = 0;
    std::uint64_t steps = 0;
    double time = 0.0;
};

RunResult run(const std::vector<std::int64_t>& frames, const SimulationConfig& config) {
    Simulation simulation = app_test::make_simulation("solar_system_j2000.json", config);
    for (const std::int64_t frame : frames) {
        const AdvanceResult result = simulation.advance(frame);
        REQUIRE_FALSE(result.failed);
        REQUIRE_FALSE(result.warp_limited);
    }
    return {core::state_hash(simulation.state()), simulation.status().steps,
            simulation.state().time().value()};
}

} // namespace

TEST_CASE("Accumulator: the final state does not depend on frame pacing",
          "[app][sim][accumulator]") {
    const SimulationConfig config = solar_config();
    constexpr std::int64_t kFrames = 600;
    const std::vector<std::int64_t> regular(kFrames, kFrameNs);
    const std::int64_t total = kFrames * kFrameNs;

    // 144 Hz-like pacing, irregular pacing, and 250 ms chunks (the clamp).
    std::vector<std::int64_t> fast;
    for (std::int64_t left = total; left > 0; left -= 6'944'444) {
        fast.push_back(std::min<std::int64_t>(6'944'444, left));
    }
    std::vector<std::int64_t> chunks;
    for (std::int64_t left = total; left > 0; left -= kMaxAdvanceWallNs) {
        chunks.push_back(std::min(kMaxAdvanceWallNs, left));
    }

    const RunResult a = run(regular, config);
    const RunResult b = run(irregular_frames(total, 1), config);
    const RunResult c = run(irregular_frames(total, 42), config);
    const RunResult d = run(fast, config);
    const RunResult e = run(chunks, config);

    // floor(total * warp / step) with step = 0.05 d = 4.32e12 ns.
    const std::uint64_t expected =
        static_cast<std::uint64_t>(total) * 1'000'000ULL / 4'320'000'000'000ULL;
    CHECK(a.steps == expected);
    CHECK(a.time == Approx(static_cast<double>(expected) * 0.05));
    for (const RunResult& other : {b, c, d, e}) {
        CHECK(other.steps == a.steps);
        CHECK(other.hash == a.hash);
        CHECK(other.time == a.time);
    }
}

TEST_CASE("Simulation steps are the integrator's steps, bit for bit", "[app][sim][accumulator]") {
    SimulationConfig config = solar_config();
    config.integrator = SpaceIntegrator::yoshida4;
    Simulation simulation = app_test::make_simulation("solar_system_j2000.json", config);
    for (int frame = 0; frame < 120; ++frame) {
        REQUIRE_FALSE(simulation.advance(kFrameNs).failed);
    }
    const std::uint64_t steps = simulation.status().steps;
    REQUIRE(steps > 0);

    // The same steps taken directly with the core types.
    core::SimState state = app_test::load_scene("solar_system_j2000.json").state;
    const auto integrator = core::make_integrator(core::IntegratorKind::yoshida4);
    const core::NewtonianGravity gravity;
    core::EventStepper stepper;
    core::EventLog log;
    for (std::uint64_t i = 0; i < steps; ++i) {
        REQUIRE(
            stepper.step(*integrator, state, core::space::Time(0.05), gravity, log).has_value());
    }
    CHECK(core::state_hash(state) == core::state_hash(simulation.state()));
    CHECK(simulation.status().integrator == "yoshida4");
    CHECK(simulation.status().force_model == "newtonian");
}

TEST_CASE("Step budget limits the warp instead of accumulating debt", "[app][sim][accumulator]") {
    SimulationConfig config = solar_config();
    config.time_warp = 10'000'000;
    config.max_steps_per_advance = 10;
    Simulation simulation = app_test::make_simulation("solar_system_j2000.json", config);

    // One 60 Hz frame at 1e7 owes 1.67e14 ns = 38.6 steps of 0.05 d.
    const AdvanceResult first = simulation.advance(kFrameNs);
    CHECK(first.steps == 10);
    CHECK(first.warp_limited);
    CHECK(simulation.status().warp_limited);
    const double achieved = 10.0 * 4.32e12 / static_cast<double>(kFrameNs);
    CHECK(simulation.status().effective_warp == Approx(achieved).epsilon(1e-9));
    CHECK(simulation.status().effective_warp < 1.0e7);

    // The dropped time is not carried over: the next frame is the same.
    const AdvanceResult second = simulation.advance(kFrameNs);
    CHECK(second.steps == 10);
    CHECK(second.warp_limited);
    CHECK(simulation.status().steps == 20);

    // A sustainable warp clears the flag and reports the requested rate.
    simulation.apply(SetTimeWarp{100'000});
    const AdvanceResult third = simulation.advance(kFrameNs);
    CHECK_FALSE(third.warp_limited);
    CHECK_FALSE(simulation.status().warp_limited);
    CHECK(simulation.status().effective_warp == Approx(1.0e5));
}

TEST_CASE("Simulation commands: pause, single step, warp, reset", "[app][sim][commands]") {
    Simulation simulation = app_test::make_simulation("sun_earth_moon.json", solar_config());
    const std::uint64_t initial_hash = core::state_hash(simulation.state());
    const std::uint64_t generation = simulation.status().generation;

    simulation.apply(SetPaused{true});
    CHECK(simulation.advance(kFrameNs * 100).steps == 0);
    CHECK(simulation.status().paused);
    CHECK(simulation.status().effective_warp == 0.0);
    CHECK(core::state_hash(simulation.state()) == initial_hash);

    simulation.apply(SingleStep{});
    CHECK(simulation.status().steps == 1);
    CHECK(simulation.status().paused);
    CHECK(simulation.state().time().value() == Approx(0.05));
    CHECK(simulation.status().step_ms.has_value());

    // Resuming does not catch up the time spent paused.
    simulation.apply(SetPaused{false});
    CHECK(simulation.advance(1).steps == 0);

    simulation.apply(SetTimeWarp{0});
    CHECK(simulation.status().time_warp == kMinTimeWarp);
    simulation.apply(SetTimeWarp{1'000'000'000});
    CHECK(simulation.status().time_warp == kMaxTimeWarp);

    simulation.apply(SetTimeWarp{1'000'000});
    for (int i = 0; i < 30; ++i) {
        (void)simulation.advance(kFrameNs);
    }
    CHECK(simulation.status().steps > 1);
    REQUIRE(simulation.status().energy_drift.has_value());
    CHECK(*simulation.status().energy_drift < 1.0e-6);

    simulation.apply(Reset{});
    CHECK(simulation.status().generation == generation + 1);
    CHECK(simulation.status().steps == 0);
    CHECK(simulation.state().time().value() == 0.0);
    CHECK(core::state_hash(simulation.state()) == initial_hash);
    CHECK(simulation.status().time_warp == 1'000'000); // kept across reset
}

TEST_CASE("Simulation loads scenarios and reports load failures", "[app][sim][commands]") {
    Simulation simulation = app_test::make_simulation("sun_earth.json", solar_config());
    CHECK(simulation.state().bodies().size() == 2);
    const std::uint64_t generation = simulation.status().generation;

    simulation.apply(LoadScenario{app_test::scenario_dir() / "does_not_exist.json"});
    CHECK(simulation.status().generation == generation);
    CHECK(simulation.status().message.find("does_not_exist.json") != std::string::npos);
    CHECK(simulation.state().bodies().size() == 2);

    simulation.apply(LoadScenario{app_test::scenario_dir() / "sgr_a_s2.json"});
    CHECK(simulation.status().generation == generation + 1);
    CHECK(simulation.status().message.empty());
    CHECK(simulation.status().force_model == "paczynski_wiita");
    CHECK(simulation.status().scenario.find("S2") != std::string::npos);

    // Reset returns to the scenario loaded last.
    (void)simulation.advance(kFrameNs);
    simulation.apply(Reset{});
    CHECK(simulation.status().force_model == "paczynski_wiita");
    CHECK(simulation.state().time().value() == 0.0);
}

TEST_CASE("Invalid simulation settings are rejected", "[app][sim]") {
    const scene::Scene scene = app_test::load_scene("sun_earth.json");
    SimulationConfig config;
    config.dt_days = 0.0;
    CHECK_FALSE(Simulation::create(scene, config).has_value());
    config = SimulationConfig{};
    config.time_warp = 0;
    CHECK_FALSE(Simulation::create(scene, config).has_value());
    config.time_warp = kMaxTimeWarp + 1;
    CHECK_FALSE(Simulation::create(scene, config).has_value());
    config = SimulationConfig{};
    config.max_steps_per_advance = 0;
    CHECK_FALSE(Simulation::create(scene, config).has_value());
    config = SimulationConfig{};
    config.softening_au = -1.0;
    CHECK_FALSE(Simulation::create(scene, config).has_value());
    CHECK(Simulation::create(scene, SimulationConfig{}).has_value());

    CHECK(integrator_from_string("ias15") == SpaceIntegrator::ias15);
    CHECK_FALSE(integrator_from_string("euler").has_value());
    CHECK(force_from_string("auto") == ForceChoice::automatic);
    CHECK(to_string(ForceChoice::barnes_hut) == "barnes_hut");
}

TEST_CASE("A failing integration step pauses the simulation with a message", "[app][sim]") {
    // Two unsoftened stars at the same point: the first acceleration is
    // non-finite and the integrator refuses the step.
    scene::Scenario scenario;
    scenario.name = "coincident";
    scene::BodySpec a;
    a.name = "A";
    a.kind = core::BodyKindTag::star;
    a.mass = core::space::Mass(1.0);
    scene::BodySpec b = a;
    b.name = "B";
    scenario.bodies = {a, b};
    auto built = scene::build_scene(scenario);
    REQUIRE(built.has_value());
    SimulationConfig config;
    config.force = ForceChoice::newtonian;
    auto created = Simulation::create(std::move(built).value(), config);
    REQUIRE(created.has_value());
    Simulation simulation = std::move(created).value();
    const AdvanceResult result = simulation.advance(kFrameNs);
    CHECK(result.failed);
    CHECK(result.steps == 0);
    CHECK(simulation.status().paused);
    CHECK(simulation.status().message.find("Integration failed") != std::string::npos);
    CHECK(simulation.state().time().value() == 0.0);
}
