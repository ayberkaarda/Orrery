#pragma once

#include <cstdint>
#include <filesystem>
#include <variant>

// Commands from the user interface to the simulation: the only direction in
// which the render side may influence physics (architecture rule; the other
// direction is the published SimSnapshot). Plain values, applied by the
// physics thread in the order they were sent.
namespace orrery::sim {

// Lowest and highest time warp: simulated seconds per wall-clock second.
inline constexpr std::int64_t kMinTimeWarp = 1;
inline constexpr std::int64_t kMaxTimeWarp = 10'000'000;

// Pauses or resumes the simulation. Resuming never catches up the paused
// wall-clock time.
struct SetPaused {
    bool paused = true;
};

// Sets the time warp; values outside [kMinTimeWarp, kMaxTimeWarp] are
// clamped.
struct SetTimeWarp {
    std::int64_t warp = kMinTimeWarp;
};

// Replaces the simulated system by a scenario file (orrery/scene/scenario.h).
// The file is read on the physics thread; on failure the current system is
// kept and the status reports the error.
struct LoadScenario {
    std::filesystem::path path;
};

// Restores the initial state of the current scenario.
struct Reset {};

// Pauses and advances exactly one physics step.
struct SingleStep {};

// Replaces the simulated system and its settings (integrator, force model,
// step, warp, paused) by a space experiment file (space_experiment.h). Read
// on the physics thread; on failure everything stays as it was and the
// status reports the error. The interactive budgets are kept.
struct LoadExperiment {
    std::filesystem::path path;
};

using SimCommand =
    std::variant<SetPaused, SetTimeWarp, LoadScenario, Reset, SingleStep, LoadExperiment>;

} // namespace orrery::sim
