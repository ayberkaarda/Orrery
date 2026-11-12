#pragma once

#include <cstdint>
#include <iosfwd>
#include <vector>

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
};

// Runs the window until it is closed. Returns an exit code of cli.h; errors
// are written to `err`, the benchmark result to `out`.
[[nodiscard]] int run_window(WindowRequest request, std::ostream& out, std::ostream& err);

} // namespace orrery::app
