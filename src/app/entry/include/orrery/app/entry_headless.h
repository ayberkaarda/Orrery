#pragma once

#include <filesystem>
#include <ostream>
#include <string>

#include "orrery/app/cli.h"
#include "orrery/app/headless_error.h"
#include "orrery/core/result.h"
#include "orrery/render/image.h"
#include "orrery/scene/experiment.h"
#include "orrery/sim/entry_backend.h"
#include "orrery/sim/entry_experiments.h"

// Headless rendering of the Entry workspace (spec 7.7; docs/entry-workspace.md,
// "Headless"): the entry runs to its end on the calling thread, then the whole
// workspace - main bar, panels with the profile plots and the ground track,
// and the 3D view with its legend - is drawn offscreen with a fixed frame
// time and no pointer, so the image is deterministic for a given experiment
// and size. Used by `orrery_app --headless` for entry experiments and by the
// golden test.
namespace orrery::app {

struct EntryHeadlessRequest {
    int width = 1600;
    int height = 900;
    int msaa_samples = 4;
};

struct EntryHeadlessFrame {
    render::ImageRgba8 image;
    sim::EntryRunResult run;
};

[[nodiscard]] core::Result<EntryHeadlessFrame, HeadlessError>
render_entry_headless(const sim::EntrySetup& setup, const EntryHeadlessRequest& request);

// The command-line path for an experiment of domain "entry" (`generic` read
// from `path`): renders, writes options.out and prints the summary line.
// Returns the exit code.
int run_entry_headless_command(const CliOptions& options, const std::filesystem::path& path,
                               const scene::Experiment& generic, std::ostream& out,
                               std::ostream& err);

} // namespace orrery::app
