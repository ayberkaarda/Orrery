#pragma once

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <filesystem>
#include <functional>
#include <thread>
#include <utility>

#include "orrery/scene/scenario.h"
#include "orrery/sim/simulation.h"

// Shared helpers of the Phase 4c application tests. Bundled assets are
// located from ORRERY_SCENARIO_DIR (<source>/assets/scenarios), so the
// experiments are its sibling directory.
namespace orrery::app_test {

inline std::filesystem::path scenario_dir() {
    return std::filesystem::path(ORRERY_SCENARIO_DIR);
}

inline std::filesystem::path asset_dir() {
    return scenario_dir().parent_path();
}

inline std::filesystem::path experiment_dir() {
    return asset_dir() / "experiments";
}

// tests/app, next to this header in the source tree.
inline std::filesystem::path test_source_dir() {
    return asset_dir().parent_path() / "tests" / "app";
}

inline scene::Scene load_scene(const char* file) {
    auto scene = scene::load_scene_file(scenario_dir() / file);
    REQUIRE(scene.has_value());
    return std::move(scene).value();
}

inline sim::Simulation make_simulation(const char* file, const sim::SimulationConfig& config) {
    auto simulation = sim::Simulation::create(load_scene(file), config);
    REQUIRE(simulation.has_value());
    return std::move(simulation).value();
}

// Polls `condition` until it holds or `timeout` passes (no fixed sleeps:
// the physics thread runs at its own pace).
inline bool wait_until(const std::function<bool()>& condition,
                       std::chrono::milliseconds timeout = std::chrono::seconds(20)) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (condition()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return condition();
}

} // namespace orrery::app_test
