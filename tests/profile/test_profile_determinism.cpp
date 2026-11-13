#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "orrery/core/barnes_hut.h"
#include "orrery/core/body.h"
#include "orrery/core/body_store.h"
#include "orrery/core/event.h"
#include "orrery/core/event_step.h"
#include "orrery/core/integrator.h"
#include "orrery/core/sim_state.h"
#include "orrery/core/state_hash.h"
#include "orrery/core/trace_capture.h"
#include "orrery/core/units.h"

using namespace orrery::core;
namespace profile = orrery::profile;

namespace {

constexpr std::size_t kBodies = 300;
constexpr int kSteps = 20;

// Deterministic pseudo-random coordinate in [-1, 1) AU (64-bit LCG).
double next_coordinate(std::uint64_t& seed) {
    seed = seed * 6364136223846793005ULL + 1442695040888963407ULL;
    return static_cast<double>(seed >> 11U) * 0x1.0p-52 - 1.0;
}

SimState cluster() {
    BodyStore store;
    std::uint64_t seed = 12345;
    for (std::size_t i = 0; i < kBodies; ++i) {
        const double x = next_coordinate(seed);
        const double y = next_coordinate(seed);
        const double z = next_coordinate(seed);
        BodyInit init{"b" + std::to_string(i), Asteroid{}, space::Mass(1.0e-6),
                      Vec3<space::Length>{space::Length(x), space::Length(y), space::Length(z)},
                      Vec3<space::Velocity>{}};
        if (!store.add(std::move(init))) {
            throw std::runtime_error("BodyStore::add failed");
        }
    }
    return SimState(space::Time(0.0), std::move(store));
}

// The state hash after kSteps Barnes-Hut steps through the event stepper
// (the physics_step and tree_build zones).
std::uint64_t run_hash() {
    BarnesHutOptions options;
    options.softening = space::Length(1.0e-3);
    const auto gravity = BarnesHutGravity::create(options);
    if (!gravity) {
        throw std::runtime_error("BarnesHutGravity::create failed");
    }
    SimState state = cluster();
    LeapfrogKdk integrator;
    EventStepper stepper(EncounterOptions{false, false, false});
    EventLog log;
    for (int s = 0; s < kSteps; ++s) {
        if (!stepper.step(integrator, state, space::Time(0.01), gravity.value(), log)) {
            throw std::runtime_error("step failed");
        }
    }
    return state_hash(state);
}

const profile::ZoneTotal* total_named(const profile::Capture& capture, std::string_view name) {
    for (const profile::ZoneTotal& total : capture.totals) {
        if (total.name == name) {
            return &total;
        }
    }
    return nullptr;
}

} // namespace

TEST_CASE("capturing zones does not change the simulated state", "[profile][determinism]") {
    profile::stop_capture();
    const std::uint64_t idle = run_hash();
    profile::start_capture();
    const std::uint64_t captured = run_hash();
    profile::stop_capture();
    CHECK(captured == idle);
    CHECK(run_hash() == idle);

    const profile::Capture capture = profile::capture_snapshot();
    if (profile::zones_compiled()) {
        // One physics_step per step; at least one tree build per step.
        const profile::ZoneTotal* steps = total_named(capture, "physics_step");
        const profile::ZoneTotal* trees = total_named(capture, "tree_build");
        REQUIRE(steps != nullptr);
        REQUIRE(trees != nullptr);
        CHECK(steps->count == static_cast<std::uint64_t>(kSteps));
        CHECK(trees->count >= static_cast<std::uint64_t>(kSteps));
    } else {
        CHECK(capture.events.empty());
        CHECK(capture.totals.empty());
    }
}
