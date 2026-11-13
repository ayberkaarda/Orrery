#pragma once

#include <cstdint>
#include <filesystem>
#include <iosfwd>
#include <optional>
#include <vector>

#include "orrery/app/config.h"
#include "orrery/scene/aero_experiment.h"
#include "orrery/sim/entry_experiments.h"
#include "orrery/sim/space_experiment.h"

// Interactive window of orrery_app (docs/app.md, "Window"). Not part of
// headless builds (ORRERY_HEADLESS): CMake compiles this unit only for
// builds with a window system.
namespace orrery::app {

struct WindowRequest {
    sim::SpaceExperiment experiment;
    // Scenarios offered by the scenario selector.
    std::vector<sim::ScenarioEntry> scenarios;
    int width = 1600;
    int height = 900;
    // > 0: render this many frames without vsync, print the timing, exit.
    std::int64_t benchmark_frames = 0;
    // Phase 8b: user configuration, the workspace to open, and an aero
    // experiment to open in the Aero workspace (its relative geometry paths
    // resolve against aero_base_dir).
    AppConfig config;
    Workspace workspace = Workspace::space;
    std::optional<scene::AeroExperiment> aero_experiment;
    std::filesystem::path aero_base_dir;
    // Phase 9b: an entry experiment to open in the Entry workspace.
    std::optional<sim::StoredEntry> entry_experiment;
    // Phase 10: the bundled experiments folder (the Space Experiment panel
    // lists its space experiments) and the configuration file shown by the
    // shortcut help.
    std::filesystem::path bundled_experiment_dir;
    std::filesystem::path config_path;
};

// Runs the window until it is closed. Returns an exit code of cli.h; errors
// are written to `err`, the benchmark result to `out`.
[[nodiscard]] int run_window(WindowRequest request, std::ostream& out, std::ostream& err);

} // namespace orrery::app
