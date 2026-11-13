#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

#include "orrery/render/ui_panels.h"
#include "orrery/sim/simulation.h"
#include "orrery/sim/space_experiment.h"

// The Experiment panel of the Space workspace (spec 8 "Scenario/Experiment";
// docs/app.md, "Experiment panel"): what runs now, saving it as a space
// experiment, and opening a saved or bundled one. Files are read and
// written on the calling (UI) thread: experiment and scenario files are
// small JSON documents. Drawn with Dear ImGui between UiContext::begin_frame
// and end_frame; save() and the listings work without a frame.
namespace orrery::app {

inline constexpr std::string_view kSpaceExperimentWindow = render::kSpaceExperimentWindow;

// What the panel shows about the running simulation.
struct SpaceRunView {
    std::string scenario;   // scenario name
    std::string experiment; // file name of the experiment opened last, or empty
    std::string integrator;
    std::string force_model;
    double dt_days = 0.0;
    std::int64_t time_warp = 1;
    double sim_time_days = 0.0;
};

// What the panel shows for `status` (status.source decides, never an empty
// file name): the startup experiment's file name while the system the
// window started with runs, the opened experiment's after a LoadExperiment,
// and none after a standalone LoadScenario.
[[nodiscard]] SpaceRunView space_run_view(const sim::SimStatus& status,
                                          const std::filesystem::path& startup_experiment,
                                          double sim_time_days);

// The scenario file the running system came from (what Save records):
// `startup_scenario` for the startup system, else the loaded one.
[[nodiscard]] std::filesystem::path running_scenario(const sim::SimStatus& status,
                                                     const std::filesystem::path& startup_scenario);

struct SpaceExperimentIntent {
    bool save = false;                         // Save pressed (the caller calls save())
    std::optional<std::filesystem::path> open; // an experiment to open
};

class SpaceExperimentPanel {
public:
    // Saved experiments live in `experiment_dir`; bundled ones are listed
    // from `bundled_dir` (empty: none). Both are read once here.
    SpaceExperimentPanel(std::filesystem::path experiment_dir, std::filesystem::path bundled_dir);

    // Re-reads both folders.
    void refresh();
    // Writes the running simulation as <name>.json into the experiments
    // folder (sim::space_record; `scenario` is the file the system came
    // from). The notice says what happened; the saved list is re-read.
    std::optional<std::filesystem::path> save(const std::filesystem::path& scenario,
                                              const sim::SimStatus& status, double sim_time_days);

    SpaceExperimentIntent draw(const SpaceRunView& run);

    [[nodiscard]] const sim::SpaceListing& saved() const noexcept { return saved_; }
    [[nodiscard]] const sim::SpaceListing& bundled() const noexcept { return bundled_; }
    [[nodiscard]] const std::string& notice() const noexcept { return notice_; }
    void set_notice(std::string notice) { notice_ = std::move(notice); }
    // The name the next save uses (the panel's text field).
    [[nodiscard]] const std::string& name() const noexcept { return name_; }
    void set_name(std::string name) { name_ = std::move(name); }

private:
    void draw_run(const SpaceRunView& run, SpaceExperimentIntent& intent);
    void draw_open(SpaceExperimentIntent& intent);

    std::filesystem::path experiment_dir_;
    std::filesystem::path bundled_dir_;
    sim::SpaceListing saved_;
    sim::SpaceListing bundled_;
    std::string name_ = "space_run";
    std::string notice_;
    bool notice_is_error_ = false;
    std::optional<std::filesystem::path> selected_;
};

} // namespace orrery::app
