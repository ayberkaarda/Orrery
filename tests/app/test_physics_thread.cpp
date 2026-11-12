#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>

#include "app_test_support.h"
#include "orrery/sim/physics_thread.h"

using namespace orrery;
using namespace orrery::sim;
using app_test::wait_until;

namespace {

PhysicsThreadOptions fast_ticks() {
    PhysicsThreadOptions options;
    options.idle_tick = std::chrono::microseconds(2000);
    return options;
}

// Fetches until a published frame satisfies `condition`.
template <typename Condition> bool wait_for_frame(PhysicsThread& physics, Condition condition) {
    return wait_until([&] {
        physics.wake();
        (void)physics.fetch();
        return condition(physics.latest());
    });
}

} // namespace

TEST_CASE("Physics thread publishes the initial state at once", "[app][sim][physics_thread]") {
    SimulationConfig config;
    config.paused = true;
    PhysicsThread physics(app_test::make_simulation("sun_earth.json", config), fast_ticks());
    const PublishedFrame& frame = physics.latest();
    REQUIRE(frame.snapshot != nullptr);
    CHECK(frame.snapshot->size() == 2);
    CHECK(frame.snapshot->time().value() == 0.0);
    CHECK(frame.status.paused);
    CHECK(frame.status.generation == 1);
}

TEST_CASE("Physics thread runs, steps and pauses on command", "[app][sim][physics_thread]") {
    SimulationConfig config;
    config.paused = true;
    config.time_warp = 10'000'000;
    PhysicsThread physics(app_test::make_simulation("sun_earth_moon.json", config), fast_ticks());

    // Paused: ticks happen, the state does not move.
    const std::uint64_t ticks = physics.ticks();
    REQUIRE(wait_until([&] { return physics.ticks() >= ticks + 5; }));
    (void)physics.fetch();
    CHECK(physics.latest().snapshot->time().value() == 0.0);

    // Single step: exactly one step.
    physics.send(SingleStep{});
    REQUIRE(wait_for_frame(physics, [](const PublishedFrame& f) { return f.status.steps == 1; }));
    CHECK(physics.latest().snapshot->sequence() == 1);
    CHECK(physics.latest().status.paused);

    // Run: the published time advances and the snapshot sequence follows the
    // step count.
    physics.send(SetPaused{false});
    REQUIRE(wait_for_frame(physics, [](const PublishedFrame& f) {
        return !f.status.paused && f.snapshot->time().value() > 1.0;
    }));
    CHECK(physics.latest().snapshot->sequence() == physics.latest().status.steps);

    // Pause again: the state freezes.
    physics.send(SetPaused{true});
    REQUIRE(wait_for_frame(physics, [](const PublishedFrame& f) { return f.status.paused; }));
    const double frozen = physics.latest().snapshot->time().value();
    const std::uint64_t later = physics.ticks();
    REQUIRE(wait_until([&] { return physics.ticks() >= later + 5; }));
    (void)physics.fetch();
    CHECK(physics.latest().snapshot->time().value() == frozen);
}

TEST_CASE("Physics thread applies warp, reset and scenario loads", "[app][sim][physics_thread]") {
    SimulationConfig config;
    config.time_warp = 1'000;
    PhysicsThread physics(app_test::make_simulation("sun_earth.json", config), fast_ticks());

    physics.send(SetTimeWarp{5'000'000});
    REQUIRE(wait_for_frame(physics, [](const PublishedFrame& f) {
        return f.status.time_warp == 5'000'000 && f.snapshot->time().value() > 0.5;
    }));

    physics.send(Reset{});
    REQUIRE(
        wait_for_frame(physics, [](const PublishedFrame& f) { return f.status.generation == 2; }));
    CHECK(physics.latest().status.time_warp == 5'000'000);

    physics.send(LoadScenario{app_test::scenario_dir() / "figure_eight.json"});
    REQUIRE(wait_for_frame(physics, [](const PublishedFrame& f) {
        return f.status.generation == 3 && f.snapshot->size() == 3;
    }));
    CHECK(physics.latest().status.message.empty());

    physics.send(LoadScenario{app_test::scenario_dir() / "missing.json"});
    REQUIRE(
        wait_for_frame(physics, [](const PublishedFrame& f) { return !f.status.message.empty(); }));
    CHECK(physics.latest().status.generation == 3);
    CHECK(physics.latest().snapshot->size() == 3);
}

TEST_CASE("Snapshots outlive the physics thread and stay immutable", "[app][sim][physics_thread]") {
    std::shared_ptr<const core::SimSnapshot> kept;
    double kept_time = 0.0;
    {
        SimulationConfig config;
        config.time_warp = 10'000'000;
        PhysicsThread physics(app_test::make_simulation("sun_earth.json", config), fast_ticks());
        REQUIRE(
            wait_for_frame(physics, [](const PublishedFrame& f) { return f.status.steps >= 3; }));
        kept = physics.latest().snapshot;
        kept_time = kept->time().value();
        // Let the thread overwrite every slot several times.
        REQUIRE(wait_for_frame(physics, [&](const PublishedFrame& f) {
            return f.status.steps >= kept->sequence() + 50;
        }));
    }
    REQUIRE(kept != nullptr);
    CHECK(kept->time().value() == kept_time);
    CHECK(kept->size() == 2);
}

// Review fix: time that elapsed under one state (paused, an old warp) must
// not be credited under the state a command switches to. With a long idle
// tick the thread sleeps between ticks; the command wakes it, and the first
// frame published after the command may only show the simulated time owed
// since the command was sent.
TEST_CASE("Commands do not apply retroactively to the time before them",
          "[app][sim][physics_thread]") {
    SimulationConfig config;
    config.paused = true;
    config.time_warp = kMaxTimeWarp;
    PhysicsThreadOptions slow;
    slow.idle_tick = std::chrono::milliseconds(300);
    PhysicsThread physics(app_test::make_simulation("sun_earth.json", config), slow);
    const auto observe = [&](auto condition) {
        // No wake(): the thread runs on its idle tick and on commands only.
        return wait_until([&] {
            (void)physics.fetch();
            return condition(physics.latest());
        });
    };
    using Clock = std::chrono::steady_clock;
    // Allowed simulated days for `wall` of real time at `warp`, plus one step.
    const auto allowed = [](Clock::duration wall, double warp) {
        return std::chrono::duration<double>(wall).count() * warp / 86400.0 + 0.05 + 1.0e-9;
    };
    for (int attempt = 0; attempt < 3; ++attempt) {
        INFO("attempt " << attempt);
        // Resume after a paused interval longer than the idle tick.
        physics.send(SetPaused{true});
        REQUIRE(observe([](const PublishedFrame& f) { return f.status.paused; }));
        std::this_thread::sleep_for(std::chrono::milliseconds(450));
        (void)physics.fetch();
        const double paused_at = physics.latest().snapshot->time().value();
        const Clock::time_point resumed = Clock::now();
        physics.send(SetPaused{false});
        REQUIRE(observe([](const PublishedFrame& f) { return !f.status.paused; }));
        const double after_resume = physics.latest().snapshot->time().value() - paused_at;
        CHECK(after_resume <= allowed(Clock::now() - resumed, static_cast<double>(kMaxTimeWarp)));

        // Raise the warp after running slowly: the time before the command
        // runs at the old warp.
        physics.send(SetTimeWarp{1});
        REQUIRE(observe([](const PublishedFrame& f) { return f.status.time_warp == 1; }));
        std::this_thread::sleep_for(std::chrono::milliseconds(450));
        (void)physics.fetch();
        const double slow_at = physics.latest().snapshot->time().value();
        const Clock::time_point raised = Clock::now();
        physics.send(SetTimeWarp{kMaxTimeWarp});
        REQUIRE(
            observe([](const PublishedFrame& f) { return f.status.time_warp == kMaxTimeWarp; }));
        const double after_raise = physics.latest().snapshot->time().value() - slow_at;
        // 450 ms at warp 1 is 5e-6 days.
        CHECK(after_raise <=
              allowed(Clock::now() - raised, static_cast<double>(kMaxTimeWarp)) + 1.0e-5);
    }
}

// Review fix: queued commands are bounded work per tick. A backlog of single
// steps is spread over several ticks (frames keep being published) and the
// thread stops between commands instead of draining the whole backlog.
TEST_CASE("A backlog of single steps is bounded per tick and does not delay shutdown",
          "[app][sim][physics_thread]") {
    SimulationConfig config;
    config.paused = true;
    config.integrator = SpaceIntegrator::ias15;
    {
        PhysicsThread physics(app_test::make_simulation("solar_system_j2000.json", config),
                              fast_ticks());
        constexpr std::uint64_t kSteps = 600;
        for (std::uint64_t i = 0; i < kSteps; ++i) {
            physics.send(SingleStep{});
        }
        // Spin on fetch (a sleeping poll can miss the short-lived frames).
        bool partial = false;
        bool done = false;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
        while (!done && std::chrono::steady_clock::now() < deadline) {
            (void)physics.fetch();
            const std::uint64_t steps = physics.latest().status.steps;
            partial = partial || (steps > 0 && steps < kSteps);
            done = steps >= kSteps;
            std::this_thread::yield();
        }
        CHECK(done);
        CHECK(partial);
        CHECK(physics.latest().status.steps == kSteps);
    }
    auto physics = std::make_unique<PhysicsThread>(
        app_test::make_simulation("solar_system_j2000.json", config), fast_ticks());
    for (int i = 0; i < 200'000; ++i) {
        physics->send(SingleStep{});
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const auto begin = std::chrono::steady_clock::now();
    physics.reset();
    const std::chrono::duration<double> shutdown = std::chrono::steady_clock::now() - begin;
    CHECK(shutdown.count() < 1.0);
}