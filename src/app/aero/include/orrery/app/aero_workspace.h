#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "orrery/app/aero_layers.h"
#include "orrery/render/camera.h"
#include "orrery/render/ui_app_bar.h"
#include "orrery/render/ui_context.h"
#include "orrery/scene/aero_experiment.h"
#include "orrery/sim/aero_runner.h"

// The Aero workspace of the window (docs/aero-workspace.md): its main bar,
// the Geometry, Flow Conditions, Aero Results, Experiments and View panels
// in their own dock layout, the solver thread and the background imports,
// and the turntable camera of the scene. Drawing the scene itself is the
// job of AeroViewport (aero_viewport.h), which reads view_state(); the
// workspace has no OpenGL code and runs under a headless UiContext.
//
// Long work never runs on the UI thread: geometry builds and imports are
// BackgroundTasks, solves run on the AeroRunner thread. Every failure is
// shown: import and geometry errors in a modal with the full detail, solver
// failures in the results panel and the bar.
namespace orrery::app {

inline constexpr std::string_view kAeroWorkspace = "Aero";
inline constexpr std::string_view kGeometryWindow = "Geometry";
inline constexpr std::string_view kFlowWindow = "Flow Conditions";
inline constexpr std::string_view kResultsWindow = "Aero Results";
inline constexpr std::string_view kExperimentsWindow = "Experiments";
inline constexpr std::string_view kViewWindow = "View";
inline constexpr std::string_view kErrorModal = "Problem";

struct AeroWorkspaceOptions {
    // Saved experiments are listed from and written to this directory.
    std::filesystem::path experiment_dir;
    // Downloaded models (empty: the platform cache).
    std::filesystem::path model_cache_dir;
    std::int64_t steps_per_frame = 100;
};

// What the scene renderer draws this frame.
struct AeroViewState {
    std::shared_ptr<const sim::PreparedGeometry> geometry;
    std::uint64_t geometry_revision = 0;
    // Of the shown run (null before any).
    std::shared_ptr<const aero::AeroResult> result;
    std::shared_ptr<const std::vector<double>> surface_cp;
    std::shared_ptr<const sim::FlowSnapshot> flow;
    std::uint64_t view_revision = 0;
    // Conditions the view frame is built from (the run's, else the inputs).
    aero::FlowConditions conditions;
    AeroLayers layers;
    bool running = false; // particles advance only while the solver runs
};

class AeroWorkspace {
public:
    explicit AeroWorkspace(AeroWorkspaceOptions options);
    AeroWorkspace(const AeroWorkspace&) = delete;
    AeroWorkspace& operator=(const AeroWorkspace&) = delete;
    AeroWorkspace(AeroWorkspace&&) = delete;
    AeroWorkspace& operator=(AeroWorkspace&&) = delete;
    ~AeroWorkspace();

    // Replaces the working experiment (its geometry is rebuilt in the
    // background; a recorded result is shown until a new run replaces it).
    // Relative geometry paths resolve against `base_dir`.
    void open(scene::AeroExperiment experiment, std::filesystem::path base_dir);

    // Once per frame before the UI: takes finished background work and the
    // solver's progress, and wakes the solver thread for this frame.
    void update(double dt);

    // The main bar of the workspace (call before the dock space, like
    // render::draw_app_bar).
    void draw_bar(double fps);
    // Dock space and panels; returns the scene area left free by them.
    render::UiRect draw_panels();
    // Orbit, zoom and keys aimed at the scene (render::read_scene_input).
    void handle_scene_input(const render::SceneInput& input, double dt);

    // Camera for a window of width x height pixels whose scene area left
    // free by the panels is `scene`: the body is framed in that area (and
    // centred on it). Reframes on a new geometry, a changed area, or on
    // request (F or Home, the View panel's button).
    [[nodiscard]] const render::Camera& camera(const render::UiRect& scene, int width, int height);
    [[nodiscard]] AeroViewState view_state() const;

    // For tests and the status line.
    [[nodiscard]] const sim::AeroProgress& progress() const noexcept;
    [[nodiscard]] const scene::AeroExperiment& experiment() const noexcept;
    [[nodiscard]] bool geometry_ready() const noexcept;
    [[nodiscard]] bool busy() const noexcept; // a background task is running
    // Starts a run of the current inputs; false (with the reason in the
    // status line) when the geometry is not ready or an input is invalid.
    bool start_run();
    // Writes the inputs to the experiments folder, like the Save button,
    // with the result of a finished or loaded run of exactly these inputs;
    // the file written, or nullopt (the reason in the status line).
    std::optional<std::filesystem::path> save();
    // Replace the flow conditions or the solver settings as their panels do.
    void set_conditions(const scene::AeroConditions& conditions);
    void set_solver_settings(const scene::AeroSolverSettings& settings);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace orrery::app
