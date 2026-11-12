#pragma once

#include <cstdint>
#include <filesystem>
#include <iosfwd>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "orrery/app/cli.h"
#include "orrery/core/result.h"
#include "orrery/core/sim_snapshot.h"
#include "orrery/render/image.h"
#include "orrery/render/render_settings.h"
#include "orrery/sim/simulation.h"
#include "orrery/sim/space_experiment.h"

// Headless rendering of an experiment (spec 7.7, docs/app.md "Headless"):
// the experiment's scenario is advanced `frames` frames of kFrameWallNs
// wall-clock time each by the same Simulation (accumulator, integrator,
// force model, step budget) that the physics thread runs, trails are recorded
// from every new snapshot exactly as in the window, and the last state is
// drawn by SpaceRenderer through the offscreen HDR pipeline with the window's
// framing camera. The result is deterministic for a given experiment, frame
// count and image size (no wall-clock budget is applied).
namespace orrery::app {

struct HeadlessRequest {
    int width = 1280;
    int height = 720;
    std::int64_t frames = 0;
    render::RenderSettings settings{};
    int msaa_samples = 4;
};

struct HeadlessFrame {
    render::ImageRgba8 image;
    std::shared_ptr<const core::SimSnapshot> snapshot;
    sim::SimStatus status;
};

enum class HeadlessErrorKind : std::uint8_t {
    no_context, // no OpenGL 4.6 core context on this machine
    failure,    // anything else (simulation, renderer setup, drawing)
};

struct HeadlessError {
    HeadlessErrorKind kind = HeadlessErrorKind::failure;
    std::string message;
};

// Advances the experiment without rendering (the simulation part of
// render_headless, usable without OpenGL).
[[nodiscard]] core::Result<sim::Simulation, std::string>
advance_headless(const sim::SpaceExperiment& experiment, std::int64_t frames);

[[nodiscard]] core::Result<HeadlessFrame, HeadlessError>
render_headless(const sim::SpaceExperiment& experiment, const HeadlessRequest& request);

// Directories searched for experiment names: <asset dir>/experiments for each
// of asset_directories(executable).
[[nodiscard]] std::vector<std::filesystem::path>
experiment_search_dirs(const std::filesystem::path& executable);

// The --headless command: resolves and loads the experiment, renders it and
// writes the PNG. Progress goes to `out`, every error to `err` as one line
// prefixed "orrery_app: ". Returns an exit code of cli.h (kExitOk,
// kExitFailure, kExitNoGlContext).
[[nodiscard]] int run_headless_command(const CliOptions& options,
                                       std::span<const std::filesystem::path> search_dirs,
                                       std::ostream& out, std::ostream& err);

} // namespace orrery::app
