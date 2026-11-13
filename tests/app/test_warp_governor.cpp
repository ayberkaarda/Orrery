#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <vector>

#include "app_test_support.h"
#include "orrery/core/state_hash.h"
#include "orrery/sim/simulation.h"
#include "orrery/sim/warp_governor.h"

// Time-warp budget (docs/app.md, "Time-warp budget"): the governor on its own
// with synthetic ticks, then inside the Simulation with a fake clock. No test
// here reads the real clock, so every outcome is exact.

using namespace orrery;
using namespace orrery::sim;

namespace {

constexpr std::int64_t kFrameNs = 16'666'667;         // a 60 Hz tick
constexpr std::int64_t kStepNs = 4'320'000'000'000;   // 0.05 d
constexpr double kCostFor2e5 = 0.6 * 4.32e12 / 2.0e5; // 12.96 ms: fits 2e5 exactly
constexpr double kCheapCost = 0.6 * 4.32e12 / 1.0e9;  // fits far more than 1e7

// Drives a governor like the accumulator does: owed simulated time grows by
// wall * warp, whole steps are taken, each costing `cost_ns` of wall time.
struct Driver {
    WarpGovernor governor;
    std::int64_t owed = 0;
    std::vector<std::int64_t> history; // effective warp after every tick

    explicit Driver(std::int64_t requested) : governor(WarpBudget{}, requested, kStepNs) {}

    void run(double seconds, double cost_ns) {
        const auto ticks = static_cast<int>(std::lround(seconds * 60.0));
        for (int i = 0; i < ticks; ++i) {
            owed += kFrameNs * governor.effective();
            const std::int64_t steps = owed / kStepNs;
            owed %= kStepNs;
            const auto busy = static_cast<std::int64_t>(static_cast<double>(steps) * cost_ns);
            history.push_back(governor.update(WarpSample{kFrameNs, busy, steps}));
        }
    }

    // Number of times the effective warp changed in the recorded history.
    [[nodiscard]] int changes() const {
        int count = 0;
        for (std::size_t i = 1; i < history.size(); ++i) {
            count += history[i] != history[i - 1] ? 1 : 0;
        }
        return count;
    }
};

// A fake monotonic clock shared with the Simulation: every reading advances
// it by `cost` (the Simulation reads it once before stepping and once after
// every step, so each step costs exactly `cost`).
struct FakeClock {
    std::shared_ptr<std::int64_t> now = std::make_shared<std::int64_t>(0);
    std::shared_ptr<std::int64_t> cost = std::make_shared<std::int64_t>(0);

    [[nodiscard]] std::function<std::int64_t()> function() const {
        return [clock_now = now, step_cost = cost] {
            const std::int64_t reading = *clock_now;
            *clock_now += *step_cost;
            return reading;
        };
    }
};

} // namespace

TEST_CASE("Warp levels follow the 1-2-5 sequence", "[app][sim][warp]") {
    CHECK(warp_level_at_or_below(0.3) == 1);
    CHECK(warp_level_at_or_below(std::numeric_limits<double>::quiet_NaN()) == 1);
    CHECK(warp_level_at_or_below(1.0) == 1);
    CHECK(warp_level_at_or_below(7.0) == 5);
    CHECK(warp_level_at_or_below(199'999.0) == 100'000);
    CHECK(warp_level_at_or_below(200'000.0) == 200'000);
    CHECK(warp_level_at_or_below(1.0e7) == 10'000'000);
    CHECK(warp_level_at_or_below(1.0e300) == 5'000'000'000'000'000'000);
    CHECK(warp_level_above(1) == 2);
    CHECK(warp_level_above(2) == 5);
    CHECK(warp_level_above(5) == 10);
    CHECK(warp_level_above(300) == 500);
    CHECK(warp_level_above(10'000'000) == 20'000'000);
}

TEST_CASE("Warp budgets are validated", "[app][sim][warp]") {
    CHECK_FALSE(validate(WarpBudget{}).has_value());
    WarpBudget budget;
    budget.load_limit = 1.5;
    CHECK(validate(budget).has_value());
    budget = WarpBudget{};
    budget.load_limit = std::numeric_limits<double>::quiet_NaN();
    CHECK(validate(budget).has_value());
    budget = WarpBudget{};
    budget.raise_margin = 1.0;
    CHECK(validate(budget).has_value());
    budget = WarpBudget{};
    budget.smoothing_ns = 0;
    CHECK(validate(budget).has_value());
    budget = WarpBudget{};
    budget.lower_after_ns = -1;
    CHECK(validate(budget).has_value());
}

TEST_CASE("An unaffordable warp is lowered once, held, and restored when the load drops",
          "[app][sim][warp]") {
    Driver driver(10'000'000);
    // Expensive steps: 1e7 would need 50 times the budget.
    driver.run(10.0, kCostFor2e5 * 1.01);
    CHECK(driver.governor.reduced());
    CHECK(driver.governor.effective() == 100'000); // 2e5 needs 101 % of the limit
    CHECK(driver.changes() == 1);                  // one drop, no flicker
    // The drop came after the hold (0.25 s), not on the first tick.
    CHECK(driver.history.front() == 10'000'000);
    REQUIRE(driver.governor.load().has_value());
    CHECK(*driver.governor.load() <= 0.6);

    // The load drops: the warp comes back to the request, only upwards, at
    // most one move per raise hold (1.5 s) while the cost estimate settles.
    driver.history.clear();
    driver.run(5.0, kCheapCost);
    CHECK(driver.governor.effective() == 10'000'000);
    CHECK_FALSE(driver.governor.reduced());
    CHECK(driver.changes() >= 1);
    CHECK(driver.changes() <= 3);
    CHECK(std::is_sorted(driver.history.begin(), driver.history.end()));
    // Not before the hold: still reduced one second in.
    CHECK(driver.history.at(60) == 100'000);
}

TEST_CASE("A load near a level boundary does not make the warp flicker", "[app][sim][warp]") {
    Driver driver(10'000'000);
    // 2e5 fits with 10 % to spare on average, the cost jitters by +-8 %
    // from tick to tick; the next level (5e5) never fits with the margin.
    const double base = kCostFor2e5 / 1.1;
    for (int second = 0; second < 30; ++second) {
        driver.run(0.5, base * 1.08);
        driver.run(0.5, base * 0.92);
    }
    CHECK(driver.governor.effective() == 200'000);
    CHECK(driver.changes() == 1);
}

TEST_CASE("The governor is a pure function of its samples", "[app][sim][warp]") {
    Driver a(5'000'000);
    Driver b(5'000'000);
    for (const double cost : {kCostFor2e5 * 3.0, kCheapCost, kCostFor2e5 * 0.5, kCostFor2e5}) {
        a.run(2.0, cost);
        b.run(2.0, cost);
    }
    CHECK(a.history == b.history);
    CHECK(a.changes() >= 2);
}

TEST_CASE("Requests apply at once below the effective warp and up to the measured capacity",
          "[app][sim][warp]") {
    Driver driver(10'000'000);
    driver.run(2.0, kCostFor2e5);
    CHECK(driver.governor.effective() == 200'000);
    driver.governor.request(1'000);
    CHECK(driver.governor.effective() == 1'000);
    // Asking for more again: granted only as far as the known cost allows.
    driver.governor.request(5'000'000);
    CHECK(driver.governor.effective() == 200'000);
    // A new system forgets the cost: the request applies until measured.
    driver.governor.restart(kStepNs);
    CHECK(driver.governor.effective() == 5'000'000);
    CHECK_FALSE(driver.governor.step_cost_ns().has_value());
    // Ticks without wall time change nothing.
    CHECK(driver.governor.update(WarpSample{0, 1'000'000, 3}) == 5'000'000);
}

TEST_CASE("The Simulation credits the governed warp and reports it", "[app][sim][warp]") {
    FakeClock clock;
    SimulationConfig config;
    config.dt_days = 0.05;
    config.time_warp = 10'000'000;
    config.warp_budget = WarpBudget{};
    config.clock = clock.function();
    Simulation simulation = app_test::make_simulation("sun_earth_moon.json", config);
    CHECK(simulation.status().load_limit == 0.6);
    CHECK(simulation.status().governed_warp == 10'000'000);

    *clock.cost = static_cast<std::int64_t>(kCostFor2e5 * 1.01);
    for (int i = 0; i < 600; ++i) {
        REQUIRE_FALSE(simulation.advance(kFrameNs).failed);
    }
    const SimStatus reduced = simulation.status();
    CHECK(reduced.warp_reduced);
    CHECK(reduced.warp_limited);
    CHECK(reduced.governed_warp == 100'000);
    CHECK(reduced.effective_warp == 100'000.0);
    CHECK(reduced.time_warp == 10'000'000); // the request is kept
    REQUIRE(reduced.physics_load.has_value());
    CHECK(*reduced.physics_load <= 0.6);
    REQUIRE(reduced.step_ms.has_value());
    CHECK(std::abs(*reduced.step_ms - kCostFor2e5 * 1.01e-6) < 1.0e-3);

    *clock.cost = 1'000;
    for (int i = 0; i < 300; ++i) {
        REQUIRE_FALSE(simulation.advance(kFrameNs).failed);
    }
    CHECK_FALSE(simulation.status().warp_reduced);
    CHECK_FALSE(simulation.status().warp_limited);
    CHECK(simulation.status().governed_warp == 10'000'000);

    // A lower request applies at once and is never exceeded.
    simulation.apply(SetTimeWarp{50'000});
    CHECK(simulation.status().governed_warp == 50'000);
}

TEST_CASE("A governed run reaches the same state as a plain run of the same steps",
          "[app][sim][warp][determinism]") {
    FakeClock clock;
    SimulationConfig config;
    config.dt_days = 0.05;
    config.time_warp = 10'000'000;
    config.warp_budget = WarpBudget{};
    config.clock = clock.function();
    Simulation governed = app_test::make_simulation("sun_earth_moon.json", config);
    // The cost changes while it runs, so the warp drops and recovers.
    for (const std::int64_t cost :
         {std::int64_t{40'000'000}, std::int64_t{1'000}, std::int64_t{13'000'000}}) {
        *clock.cost = cost;
        for (int i = 0; i < 240; ++i) {
            REQUIRE_FALSE(governed.advance(kFrameNs).failed);
        }
    }
    const std::uint64_t steps = governed.status().steps;
    REQUIRE(steps > 100);

    SimulationConfig plain_config;
    plain_config.dt_days = 0.05;
    plain_config.time_warp = 10'000'000;
    Simulation plain = app_test::make_simulation("sun_earth_moon.json", plain_config);
    for (std::uint64_t i = 0; i < steps; ++i) {
        plain.apply(SingleStep{});
    }
    CHECK(plain.status().steps == steps);
    CHECK(plain.state().time().value() == governed.state().time().value());
    CHECK(core::state_hash(plain.state()) == core::state_hash(governed.state()));
}

TEST_CASE("An invalid warp budget is refused", "[app][sim][warp]") {
    SimulationConfig config;
    WarpBudget budget;
    budget.load_limit = 0.0;
    config.warp_budget = budget;
    CHECK_FALSE(
        Simulation::create(app_test::load_scene("sun_earth_moon.json"), config).has_value());
}
