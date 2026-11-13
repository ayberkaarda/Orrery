#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

#include "orrery/app/headless_error.h"
#include "orrery/core/result.h"
#include "orrery/render/image.h"
#include "orrery/scene/aero_experiment.h"
#include "orrery/sim/aero_solve.h"

// Headless rendering of an aero experiment (spec 7.7 for the Aero workspace;
// docs/aero-workspace.md, "Headless"): the geometry is prepared as in the
// workspace (relative paths against `base_dir`), the solver runs on the
// calling thread - a time-marching solver for exactly `frames` frames of the
// experiment's steps_per_frame lattice steps (capped at its duration; the
// solver calls then advance a divisor of that count), or to
// completion when frames is 0; a direct solver always to completion - and
// the Aero view draws the result with the workspace's default layers and
// framing. The lattice Boltzmann solvers run on the CPU, so the image is
// deterministic for a given experiment, frame count and size.
namespace orrery::app {

struct AeroHeadlessRequest {
    int width = 1280;
    int height = 720;
    std::int64_t frames = 0;
    int msaa_samples = 4;
};

struct AeroHeadlessFrame {
    render::ImageRgba8 image;
    sim::SolveOutcome outcome;
    std::string label; // the geometry
};

[[nodiscard]] core::Result<AeroHeadlessFrame, HeadlessError>
render_aero_headless(const scene::AeroExperiment& experiment, const std::filesystem::path& base_dir,
                     const AeroHeadlessRequest& request);

} // namespace orrery::app
