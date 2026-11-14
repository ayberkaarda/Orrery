#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iosfwd>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "orrery/app/config.h"
#include "orrery/app/workspace.h"
#include "orrery/core/sim_snapshot.h"
#include "orrery/render/gl_context.h"
#include "orrery/scene/aero_experiment.h"
#include "orrery/sim/entry_experiments.h"
#include "orrery/sim/simulation.h"
#include "orrery/sim/space_experiment.h"

namespace orrery::render {
class UiContext;
} // namespace orrery::render

// Interactive window of orrery_app (docs/app.md, "Window"). Not part of
// headless builds (ORRERY_HEADLESS): CMake compiles this unit only for
// builds with a window system.
namespace orrery::app {

class AeroWorkspace;
class EntryWorkspace;
class SpaceExperimentPanel;

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

// Scripted window sessions (docs/app.md, "Scripted window sessions"): the
// real window loop with a hidden window, driven frame by frame. The driver
// queues input on the UI's Dear ImGui context with the calls the GLFW
// platform backend's callbacks make (io.AddKeyEvent, AddInputCharacter,
// AddMousePosEvent, AddMouseButtonEvent), so everything from input ingestion
// onwards is the window's own code; only the operating system's delivery of
// events to GLFW is left out.

// The window after one frame, as a scripted session sees it. The pointers
// are valid during WindowDriver::after_frame only.
struct WindowFrame {
    std::int64_t index = 0; // 0 for the first frame
    // False when the framebuffer was empty (minimised): nothing was drawn.
    bool drawn = false;
    int width = 0; // framebuffer [px]
    int height = 0;
    render::ContentScale content_scale;
    double frame_seconds = 0.0; // measured wall time since the previous frame [s]
    Workspace workspace = Workspace::space;
    bool help_open = false;
    // The main bar's warp-limited chip (LIMITED) is shown.
    bool warp_limited_shown = false;
    // The Space simulation's latest status and snapshot, and the scenario
    // selector's entry.
    const sim::SimStatus* status = nullptr;
    std::shared_ptr<const core::SimSnapshot> snapshot;
    std::optional<std::size_t> scenario;
    AeroWorkspace* aero = nullptr;
    EntryWorkspace* entry = nullptr;
    SpaceExperimentPanel* space_panel = nullptr;
    const render::UiContext* ui = nullptr;
    // Current; its back buffer holds this frame (read_window_pixels).
    const render::GlContext* gl = nullptr;
    // OpenGL errors (glGetError, and KHR_debug errors with gl_debug) since
    // the window opened.
    std::size_t gl_errors = 0;
};

// What a scripted session asks of the window before a frame.
struct WindowControl {
    // Resize the window's client area (each > 0).
    std::optional<std::pair<int, int>> window_size;
    // This frame sees an empty framebuffer, as a minimised window does
    // (GLFW cannot size a window to zero).
    bool empty_framebuffer = false;
    // End the loop after this frame's check (exit code 0, like closing).
    bool close = false;
};

class WindowDriver {
public:
    WindowDriver() = default;
    WindowDriver(const WindowDriver&) = delete;
    WindowDriver& operator=(const WindowDriver&) = delete;
    WindowDriver(WindowDriver&&) = delete;
    WindowDriver& operator=(WindowDriver&&) = delete;
    virtual ~WindowDriver() = default;
    // Before frame `index`, after GLFW's events were polled and with the
    // UI's Dear ImGui context current: queue input, set `control`.
    virtual void before_frame(std::int64_t index, WindowControl& control) = 0;
    // After the frame was drawn into the back buffer, before the swap.
    virtual void after_frame(const WindowFrame& frame) = 0;
};

struct WindowSession {
    WindowDriver* driver = nullptr; // null: an interactive window
    // Hidden window (never shown, never focused) instead of a visible one.
    bool hidden = false;
    // Debug context; KHR_debug errors count in WindowFrame::gl_errors.
    bool gl_debug = false;
};

// Runs the window until it is closed. Returns an exit code of cli.h; errors
// are written to `err`, the benchmark result to `out`.
[[nodiscard]] int run_window(WindowRequest request, std::ostream& out, std::ostream& err,
                             const WindowSession& session = {});

// --capture (docs/screenshots.md): runs the window hidden for `frames`
// frames at 60 per second, waits for the shown workspace's run (an aero
// solve started on the experiment, or an entry run) to finish, then writes
// the window's framebuffer, panels included, to `out_png`.
[[nodiscard]] int capture_window(WindowRequest request, std::int64_t frames,
                                 const std::filesystem::path& out_png, std::ostream& out,
                                 std::ostream& err);

} // namespace orrery::app
