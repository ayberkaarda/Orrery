#pragma once

// Private state of the Entry workspace, shared by its translation units
// (entry_workspace.cpp: state, bar and layout; entry_panels.cpp: Entry
// Setup, Experiments and View; entry_profile.cpp: Entry Profile, Ground
// Track and the scene overlay). Private to orrery_app_core.

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "orrery/app/entry_workspace.h"
#include "orrery/core/result.h"
#include "orrery/render/aero_colormap.h"
#include "orrery/sim/background_task.h"
#include "orrery/sim/entry_backend.h"
#include "orrery/sim/entry_comparison.h"

namespace orrery::app {

// Equal inputs apart from the name (which does not change a run).
[[nodiscard]] bool same_entry_inputs(const sim::EntrySetup& a, const sim::EntrySetup& b);

// Quantity shown on the ribbon, the map and the legend: label, unit and
// the value of a sample in that unit.
struct EntryQuantity {
    const char* title; // "g-load", "Stagnation heat flux"
    const char* unit;  // "g", "kW/m2"
    double scale;      // series value x scale = shown value
};
[[nodiscard]] EntryQuantity entry_quantity(EntryColoring coloring) noexcept;
[[nodiscard]] const std::vector<double>& coloring_values(const sim::EntrySeries& series,
                                                         EntryColoring coloring) noexcept;
// Colour range of the shown quantity: from 0 to a round number above its
// peak.
[[nodiscard]] render::ScalarRange coloring_range(const sim::EntrySeries& series,
                                                 EntryColoring coloring);

using CdTaskResult = core::Result<sim::CdSamples, std::string>;

struct EntryWorkspace::Impl {
    explicit Impl(EntryWorkspaceOptions o);

    EntryWorkspaceOptions options;
    sim::EntrySetup setup;
    sim::EntryRunner runner;

    // Inputs of the last run started; the record loaded from a file and the
    // inputs it was recorded with.
    std::optional<sim::EntrySetup> run_setup;
    std::uint64_t run_number = 0; // the runner's number of our latest run
    std::uint64_t starts_sent = 0;
    std::shared_ptr<const sim::EntryTrace> loaded_trace;
    std::optional<sim::EntrySetup> loaded_setup;
    bool loaded_is_record = true; // false for present()

    // Cd sampling from the aero solver.
    sim::BackgroundTask<CdTaskResult> cd_task;
    sim::CdBody cd_body = sim::CdBody::apollo_capsule;
    std::string cd_label;
    std::vector<std::string> cd_warnings;

    // Experiments.
    sim::EntryListing listing;
    std::vector<char> compared; // per listed experiment
    bool listing_dirty = true;
    std::string notice;
    bool notice_is_error = false;

    // Problem modal: title and detail.
    std::optional<std::pair<std::string, std::string>> modal;
    bool modal_requested = false;

    // Shared cursor: hovered in this frame (plots, map), else pinned by a
    // click, else the peak g-load.
    std::optional<double> hover_time;
    std::optional<double> pinned_time;

    // View.
    EntryLayers layers;
    render::Camera camera;
    render::UiRect camera_scene;
    render::CameraInput camera_input;
    int camera_width = 0;
    int camera_height = 0;
    std::uint64_t framed_key = 0;
    bool reframe_pending = true;
    std::string view_problem;

    double pace = 100.0;
    bool layout_built = false;
    int layout_frames = 0;
    std::array<char, 160> name_buffer{};

    // --- State -------------------------------------------------------------
    [[nodiscard]] bool active() const; // running or paused
    [[nodiscard]] std::shared_ptr<const sim::EntryTrace> shown() const;
    [[nodiscard]] bool shows_record() const;
    [[nodiscard]] std::optional<double> cursor() const;
    // Key of what the camera frames (trace revision, planet, exaggeration).
    [[nodiscard]] std::uint64_t view_key() const;
    bool start_run();
    void open(sim::StoredEntry entry);
    std::optional<std::filesystem::path> save();
    void refresh_listing();
    std::optional<std::filesystem::path> export_comparison();
    [[nodiscard]] std::vector<sim::EntryComparisonInput> comparison_inputs() const;
    void set_notice(std::string message, bool is_error);
    void show_problem(std::string title, std::string detail);
    void poll_cd_task();
    void start_cd_sampling();
    void reframe();
    [[nodiscard]] EntryViewState view_state() const;

    // --- Layout and panels --------------------------------------------------
    void draw_bar(double fps);
    render::UiRect dockspace();
    void draw_setup_panel();
    void draw_planet_inputs();
    void draw_probe_inputs();
    void draw_cd_inputs();
    void draw_cd_table();
    void draw_initial_state();
    void draw_integration_inputs();
    void draw_run_controls();
    void draw_profile_panel();
    void draw_readouts(const sim::EntryTrace* trace);
    void draw_cursor_line(const sim::EntryTrace& trace);
    void draw_plots(const sim::EntryTrace* trace);
    void draw_ground_track_panel();
    void draw_experiments_panel();
    void draw_saved_table();
    void draw_comparison_table();
    void draw_view_panel();
    void draw_scene_overlay(const render::UiRect& scene);
    void draw_modal();
};

} // namespace orrery::app
