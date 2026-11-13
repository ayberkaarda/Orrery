#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "orrery/render/camera.h"
#include "orrery/render/entry_view.h"
#include "orrery/render/ui_app_bar.h"
#include "orrery/render/ui_context.h"
#include "orrery/sim/entry_experiments.h"
#include "orrery/sim/entry_runner.h"

// The Entry workspace of the window (docs/entry-workspace.md): its main bar,
// the Entry Setup, Entry Profile, Ground Track, Experiments and View panels
// in their own dock layout, the solver thread, and the turntable camera of
// the scene. Drawing the 3D scene is the job of EntryViewport
// (entry_viewport.h), which reads view_state(); the workspace has no OpenGL
// code and runs under a headless UiContext.
//
// The entry integrates on the EntryRunner thread; Cd sampling from the aero
// solver runs as a background task. Every failure is shown: setup problems
// next to the inputs, solver failures in the profile panel and the bar,
// file problems in the problem modal.
namespace orrery::app {

inline constexpr std::string_view kEntryWorkspace = "Entry";
inline constexpr std::string_view kEntrySetupWindow = "Entry Setup";
inline constexpr std::string_view kEntryProfileWindow = "Entry Profile";
inline constexpr std::string_view kGroundTrackWindow = "Ground Track";
// Displayed as "Experiments" and "View"; the suffix keeps them apart from
// the Aero workspace's windows of the same title.
inline constexpr std::string_view kEntryExperimentsWindow = "Experiments##entry";
inline constexpr std::string_view kEntryViewWindow = "View##entry";
inline constexpr std::string_view kEntryErrorModal = "Problem##entry";

struct EntryWorkspaceOptions {
    // Saved experiments are listed from and written to this directory.
    std::filesystem::path experiment_dir;
    // Simulated seconds per wall-clock second (0: as fast as possible).
    double pace = 100.0;
};

// What the ribbon is coloured by.
enum class EntryColoring : std::uint8_t { g_load, heat_flux };

struct EntryLayers {
    EntryColoring coloring = EntryColoring::g_load;
    double altitude_scale = 5.0;
    bool atmosphere = true;
    bool graticule = true;
    bool ground_track = true;
    bool curtain = true;
};

// What the scene renderer draws this frame.
struct EntryViewState {
    sim::EntryPlanet planet = sim::EntryPlanet::earth;
    std::shared_ptr<const sim::EntryTrace> trace; // the shown run (null before any)
    std::uint64_t revision = 0;                   // changes with trace, planet, scale or coloring
    render::EntryGlobe globe;
    render::EntryViewOptions options; // range and markers filled in
    std::vector<render::EntryTrackPoint> track;
};

class EntryWorkspace {
public:
    explicit EntryWorkspace(EntryWorkspaceOptions options);
    EntryWorkspace(const EntryWorkspace&) = delete;
    EntryWorkspace& operator=(const EntryWorkspace&) = delete;
    EntryWorkspace(EntryWorkspace&&) = delete;
    EntryWorkspace& operator=(EntryWorkspace&&) = delete;
    ~EntryWorkspace();

    // Replaces the inputs with a stored experiment; its recorded run is shown
    // until a new run replaces it (the current run is stopped).
    void open(sim::StoredEntry entry);
    // Like open(), for a run made outside the workspace (the headless path):
    // shown as a run of these inputs, not as a recorded one.
    void present(sim::StoredEntry entry);

    // Once per frame before the UI: takes the solver's progress and finished
    // background work, and wakes the solver thread.
    void update(double dt);
    // The main bar of the workspace (call before the dock space).
    void draw_bar(double fps);
    // Dock space, panels and the scene overlay (legend, outcome banner);
    // returns the scene area left free by the panels.
    render::UiRect draw_panels();
    // Orbit, zoom and keys aimed at the scene (render::read_scene_input).
    void handle_scene_input(const render::SceneInput& input, double dt);

    // Camera for a window of width x height pixels whose scene area is
    // `scene`: the track framed in that area. Reframes on a new track,
    // planet or exaggeration, a changed area, or on request (F or Home).
    [[nodiscard]] const render::Camera& camera(const render::UiRect& scene, int width, int height);
    [[nodiscard]] EntryViewState view_state() const;

    // For tests, the headless path and the status line.
    [[nodiscard]] const sim::EntryProgress& progress() const noexcept;
    [[nodiscard]] const sim::EntrySetup& setup() const noexcept;
    void set_setup(const sim::EntrySetup& setup);
    // The run shown in the profile and the view: the workspace's run of the
    // current inputs, else the loaded record; null when there is neither.
    [[nodiscard]] std::shared_ptr<const sim::EntryTrace> shown_trace() const;
    [[nodiscard]] bool shows_record() const;
    // The cursor shared by the profile plots, the map and the view [s].
    [[nodiscard]] std::optional<double> cursor_time() const;
    void set_cursor_time(std::optional<double> t);
    // Starts a run of the current inputs; false (reason in the panel) when an
    // input is invalid.
    bool start_run();
    void pause(bool paused);
    void stop();
    void set_pace(double pace);
    // Writes the inputs, with the shown run when it belongs to exactly these
    // inputs, to the experiments folder; the file or nullopt (reason in the
    // panel).
    std::optional<std::filesystem::path> save();
    // Re-reads the experiments folder.
    void refresh_experiments();
    [[nodiscard]] const sim::EntryListing& experiments() const noexcept;
    // Marks a listed experiment for the comparison and the profile overlay.
    void set_compared(std::size_t index, bool compared);
    // Writes the comparison CSV next to the experiments; the file or nullopt.
    std::optional<std::filesystem::path> export_comparison();
    [[nodiscard]] EntryLayers& layers() noexcept;
    // What the scene could not show (EntryViewport::problem()).
    void set_view_problem(std::string problem);
    // The last notice of the Experiments panel (save, load, export).
    [[nodiscard]] const std::string& notice() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace orrery::app
