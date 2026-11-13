#include "orrery/app/window_app.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>
#include <ostream>
#include <span>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include "orrery/app/aero_viewport.h"
#include "orrery/app/aero_workspace.h"
#include "orrery/app/cli.h"
#include "orrery/app/entry_viewport.h"
#include "orrery/app/entry_workspace.h"
#include "orrery/app/frame_stats.h"
#include "orrery/app/shortcut_dispatch.h"
#include "orrery/app/shortcut_input.h"
#include "orrery/app/shortcuts.h"
#include "orrery/app/space_experiment_panel.h"
#include "orrery/app/space_view.h"
#include "orrery/app/workspace_strip.h"
#include "orrery/core/sim_snapshot.h"
#include "orrery/render/body_visuals.h"
#include "orrery/render/camera.h"
#include "orrery/render/gl_context.h"
#include "orrery/render/hdr_pipeline.h"
#include "orrery/render/render_settings.h"
#include "orrery/render/space_renderer.h"
#include "orrery/render/trails.h"
#include "orrery/render/ui_app_bar.h"
#include "orrery/render/ui_context.h"
#include "orrery/render/ui_panels.h"
#include "orrery/sim/physics_thread.h"

namespace orrery::app {

namespace {

using Clock = std::chrono::steady_clock;

// Orbit camera controls: radians per dragged pixel, e-folds of distance per
// wheel notch.
constexpr double kDragRadiansPerPixel = 0.005;
constexpr double kZoomPerNotch = 0.15;
// Interactive wall-clock budget of one physics tick [s]: beyond it the warp
// is reported as limited instead of letting the tick (and the snapshot
// latency) grow.
constexpr double kInteractiveStepBudget = 0.02;
// The warp-limited chip stays visible this long after the last limited tick,
// so that a budget hit every few ticks does not make it flicker [s].
constexpr double kWarpLimitHold = 0.75;
constexpr std::size_t kMaxSeriesPoints = 4096;
constexpr std::int64_t kBenchmarkWarmupFrames = 30;

std::optional<std::size_t> scenario_index(const std::vector<sim::ScenarioEntry>& scenarios,
                                          const std::filesystem::path& path) {
    for (std::size_t i = 0; i < scenarios.size(); ++i) {
        std::error_code error;
        if (std::filesystem::equivalent(scenarios[i].path, path, error) && !error) {
            return i;
        }
    }
    return std::nullopt;
}

// Drift series of the System Monitor: decimated by two when full, so that it
// always spans the whole run with at most kMaxSeriesPoints points.
struct Series {
    std::vector<double> time_days;
    std::vector<double> values;

    void clear() noexcept {
        time_days.clear();
        values.clear();
    }

    void add(double t, double value) {
        if (!time_days.empty() && t <= time_days.back()) {
            return;
        }
        if (time_days.size() >= kMaxSeriesPoints) {
            std::size_t kept = 0;
            for (std::size_t i = 0; i < time_days.size(); i += 2) {
                time_days[kept] = time_days[i];
                values[kept] = values[i];
                ++kept;
            }
            time_days.resize(kept);
            values.resize(kept);
        }
        time_days.push_back(t);
        values.push_back(value);
    }
};

// Where the centre of the scene area sits in the window, in half window
// heights right and up of the window's centre (render::OrbitMode::view_shift).
glm::dvec2 scene_view_shift(const render::UiRect& scene, int window_width, int window_height) {
    const double height = static_cast<double>(window_height);
    return {2.0 * (scene.x + 0.5 * scene.width - 0.5 * static_cast<double>(window_width)) / height,
            2.0 * (0.5 * height - (scene.y + 0.5 * scene.height)) / height};
}

bool same_area(const render::UiRect& a, const render::UiRect& b) {
    return std::abs(a.x - b.x) <= 0.5 && std::abs(a.y - b.y) <= 0.5 &&
           std::abs(a.width - b.width) <= 0.5 && std::abs(a.height - b.height) <= 0.5;
}

double focus_distance(const core::SimSnapshot& snapshot, std::size_t index) {
    const render::BodyAppearance appearance = render::appearance_for(
        snapshot.names()[index], snapshot.kinds()[index], snapshot.masses()[index]);
    return std::max(12.0 * render::physical_radius(snapshot.radii()[index], appearance), 1.0e6);
}

class WindowApp {
public:
    WindowApp(WindowRequest request, render::GlContext& gl, render::UiContext& ui,
              render::SpaceRenderer& space, sim::PhysicsThread& physics, std::ostream& out,
              WindowDriver* driver)
        : request_(std::move(request)), gl_(gl), ui_(ui), space_(space), physics_(physics),
          out_(out), driver_(driver), trails_(trail_settings(settings_)),
          workspace_(request_.workspace), aero_(aero_options(request_.config)),
          entry_(entry_options(request_.config)),
          space_panel_(request_.config.experiment_dir, request_.bundled_experiment_dir) {
        for (const sim::ScenarioEntry& entry : request_.scenarios) {
            scenario_names_.push_back(entry.name);
        }
        current_scenario_ = scenario_index(request_.scenarios, request_.experiment.scenario_path);
        if (request_.aero_experiment) {
            aero_.open(*request_.aero_experiment, request_.aero_base_dir);
        }
        if (request_.entry_experiment) {
            entry_.open(*request_.entry_experiment);
        }
        take(physics_.latest());
    }

    // Runs until the window closes (or the benchmark ends); returns an exit
    // code.
    int run(std::ostream& err);

private:
    static AeroWorkspaceOptions aero_options(const AppConfig& config) {
        AeroWorkspaceOptions options;
        options.experiment_dir = config.experiment_dir;
        options.model_cache_dir = config.model_cache_dir;
        options.steps_per_frame = config.steps_per_frame;
        return options;
    }

    static EntryWorkspaceOptions entry_options(const AppConfig& config) {
        EntryWorkspaceOptions options;
        options.experiment_dir = config.experiment_dir;
        return options;
    }

    void take(const sim::PublishedFrame& frame);
    // What the shortcuts act on this frame (shortcut_dispatch.h).
    ShortcutTargets shortcut_targets();
    void draw_space_experiment();
    void reframe(const render::UiRect& scene, int window_width, int window_height);
    // A new scene area (window resized, panels docked anew): framed again
    // while the user has not moved the camera since the last framing,
    // otherwise only centred on the new area.
    void follow_scene_area(const render::UiRect& scene, int window_width, int window_height);
    void build_ui(double dt, int width, int height);
    void draw_space_bar(double fps);
    void build_space_ui(const render::SceneInput& input, double dt, int width, int height);
    void handle_scene_input(const render::SceneInput& input, double dt);
    void draw_aero_scene(const render::FrameInfo& frame, double dt, std::ostream& err);
    void draw_entry_scene(const render::FrameInfo& frame, std::ostream& err);
    // One drawn frame into `pipeline` (recreated for a new size); false when
    // the pipeline cannot be created.
    bool draw_frame(int width, int height, double dt, std::ostream& err);
    void draw_space_scene(const render::FrameInfo& frame, double dt);
    // Scripted sessions: the driver's requests before a frame, and its view
    // of the frame afterwards.
    WindowControl drive_before_frame();
    void report_frame(bool drawn, int width, int height, double frame_seconds);

    WindowRequest request_;
    render::GlContext& gl_;
    render::UiContext& ui_;
    render::SpaceRenderer& space_;
    sim::PhysicsThread& physics_;
    std::ostream& out_;
    WindowDriver* driver_ = nullptr;
    std::int64_t frames_ = 0;
    std::size_t gl_errors_ = 0;
    std::optional<render::HdrPipeline> pipeline_;

    render::RenderSettings settings_;
    render::TrailHistory trails_;
    render::Camera camera_;
    std::optional<core::BodyId> selected_;
    std::shared_ptr<const core::SimSnapshot> snapshot_;
    sim::SimStatus status_;
    std::optional<std::uint64_t> generation_;
    Series drift_;
    std::vector<std::string> scenario_names_;
    std::optional<std::size_t> current_scenario_;
    std::optional<std::size_t> requested_scenario_;
    double frame_seconds_ = 1.0 / 60.0; // smoothed frame time
    double warp_limited_hold_ = 0.0;
    // The view is framed again (scenario loaded or reset, F/Home) once the
    // scene area of the dock space is known in the next UI frame.
    bool reframe_pending_ = true;
    // The scene area the view was last framed or centred for, and whether
    // the user moved the camera since the last framing.
    std::optional<render::UiRect> framed_scene_;
    bool camera_moved_ = false;
    // Phase 10: the help overlay of the keyboard shortcuts is shown.
    bool help_open_ = false;
    render::CameraInput camera_input_;

    // Phase 8b: workspaces. The Aero workspace's view is created in the
    // window's context the first time it is shown.
    Workspace workspace_ = Workspace::space;
    AeroWorkspace aero_;
    std::optional<AeroViewport> aero_view_;
    bool aero_view_failed_ = false;
    // The aero view's last reported problem (printed once per change).
    std::string aero_view_problem_;
    render::UiRect aero_scene_;

    // Phase 9b: the Entry workspace, its view created like the aero view's.
    EntryWorkspace entry_;
    std::optional<EntryViewport> entry_view_;
    bool entry_view_failed_ = false;
    std::string entry_view_problem_;
    render::UiRect entry_scene_;

    // Phase 10: the Space workspace's Experiment panel.
    SpaceExperimentPanel space_panel_;
};

void WindowApp::take(const sim::PublishedFrame& frame) {
    const bool new_generation = !generation_ || *generation_ != frame.status.generation;
    const bool new_snapshot = frame.snapshot != snapshot_;
    snapshot_ = frame.snapshot;
    status_ = frame.status;
    if (new_generation) {
        generation_ = frame.status.generation;
        trails_.clear();
        drift_.clear();
        if (requested_scenario_ && status_.message.empty()) {
            current_scenario_ = requested_scenario_;
        }
        if (!status_.scenario_file.empty()) {
            // Loaded by a command (scenario selector or an opened experiment).
            current_scenario_ = scenario_index(request_.scenarios, status_.scenario_file);
        }
        requested_scenario_.reset();
        if (selected_ && snapshot_ && !snapshot_->index_of(*selected_)) {
            selected_.reset();
        }
        reframe_pending_ = true;
    }
    if (new_snapshot && snapshot_) {
        trails_.record(*snapshot_);
        if (status_.energy_drift) {
            drift_.add(snapshot_->time().value(), *status_.energy_drift);
        }
    }
    if (status_.warp_limited) {
        warp_limited_hold_ = kWarpLimitHold;
    }
}

// Frames the system in the scene area left free by the panels: the framing
// is computed for that area's aspect, moved back so that the area's height
// (not the window's) spans the field of view, and shifted so that the system
// sits at the area's centre instead of the window's.
void WindowApp::reframe(const render::UiRect& scene, int window_width, int window_height) {
    if (!snapshot_) {
        return;
    }
    render::CameraLens lens;
    lens.vertical_fov = kSpaceFovDegrees * 3.141592653589793 / 180.0;
    (void)camera_.set_lens(lens);
    const bool usable = scene.width > 1.0 && scene.height > 1.0 && window_height > 0;
    render::OrbitMode orbit = framing_orbit(*snapshot_, usable ? scene.width / scene.height : 1.0);
    if (usable) {
        orbit.distance *= std::max(1.0, static_cast<double>(window_height) / scene.height);
        orbit.view_shift = scene_view_shift(scene, window_width, window_height);
    }
    camera_.set_mode(orbit);
    camera_.update(0.0, render::CameraInput{}, body_frames(snapshot_));
    framed_scene_ = scene;
    camera_moved_ = false;
}

void WindowApp::follow_scene_area(const render::UiRect& scene, int window_width,
                                  int window_height) {
    if (!framed_scene_ || same_area(scene, *framed_scene_) || reframe_pending_) {
        return;
    }
    const auto* orbit = std::get_if<render::OrbitMode>(&camera_.goal());
    if (!camera_moved_ || orbit == nullptr) {
        reframe_pending_ = true;
        return;
    }
    if (scene.width > 1.0 && scene.height > 1.0 && window_height > 0) {
        render::OrbitMode centred = *orbit;
        centred.view_shift = scene_view_shift(scene, window_width, window_height);
        camera_.set_mode(centred);
    }
    framed_scene_ = scene;
}

void WindowApp::handle_scene_input(const render::SceneInput& input, double dt) {
    ShortcutTargets targets = shortcut_targets();
    if (apply_space_scene_input(input, targets)) {
        reframe_pending_ = true;
    }
    camera_input_ = render::CameraInput{};
    camera_moved_ =
        camera_moved_ || input.drag_x != 0.0 || input.drag_y != 0.0 || input.wheel != 0.0;
    if (dt > 0.0) {
        camera_input_.yaw = -input.drag_x * kDragRadiansPerPixel / dt;
        camera_input_.pitch = input.drag_y * kDragRadiansPerPixel / dt;
        camera_input_.zoom = input.wheel * kZoomPerNotch / dt;
    }
}

void WindowApp::build_ui(double dt, int width, int height) {
    render::UiFrameInput input;
    input.width = width;
    input.height = height;
    input.delta_time = dt;
    ui_.begin_frame(input);

    const double fps = frame_seconds_ > 0.0 ? 1.0 / frame_seconds_ : 0.0;
    if (workspace_ == Workspace::aero) {
        aero_.draw_bar(fps);
    } else if (workspace_ == Workspace::entry) {
        entry_.draw_bar(fps);
    } else {
        draw_space_bar(fps);
    }
    if (const auto picked = draw_workspace_strip(workspace_)) {
        workspace_ = *picked;
    }
    // One table drives every key (docs/app.md, "Keyboard shortcuts"), read
    // and carried out by the functions the end-to-end tests drive as well.
    const std::vector<ShortcutAction> actions =
        frame_shortcuts(request_.config.shortcuts, workspace_);
    ShortcutTargets targets = shortcut_targets();
    apply_shortcuts(actions, targets);
    const render::SceneInput scene_input = scene_input_from(render::read_scene_input(), actions);
    if (workspace_ == Workspace::aero) {
        aero_scene_ = aero_.draw_panels();
        aero_.handle_scene_input(scene_input, dt);
    } else if (workspace_ == Workspace::entry) {
        entry_scene_ = entry_.draw_panels();
        entry_.handle_scene_input(scene_input, dt);
    } else {
        build_space_ui(scene_input, dt, width, height);
    }
    apply_late_shortcuts(actions, targets); // open: after the panels exist this frame
    draw_shortcut_help(request_.config.shortcuts, workspace_, help_open_,
                       request_.config_path.string());
}

ShortcutTargets WindowApp::shortcut_targets() {
    return ShortcutTargets{workspace_,
                           help_open_,
                           selected_,
                           aero_,
                           entry_,
                           space_panel_,
                           physics_,
                           status_,
                           running_scenario(status_, request_.experiment.scenario_path),
                           snapshot_ ? snapshot_->time().value() : 0.0};
}

void WindowApp::draw_space_experiment() {
    const SpaceRunView run = space_run_view(status_, request_.experiment.experiment_path,
                                            snapshot_ ? snapshot_->time().value() : 0.0);
    const SpaceExperimentIntent intent = space_panel_.draw(run);
    if (intent.save) {
        ShortcutTargets targets = shortcut_targets();
        save_workspace_experiment(targets);
    }
    if (intent.open) {
        physics_.send(sim::LoadExperiment{*intent.open});
        space_panel_.set_notice("Opening " + intent.open->filename().string());
    }
}

void WindowApp::draw_space_bar(double fps) {
    render::AppBarData bar;
    bar.paused = status_.paused;
    bar.time_warp = status_.time_warp;
    bar.effective_warp = status_.effective_warp;
    bar.warp_limited = warp_limited_hold_ > 0.0;
    bar.warp_reduced = status_.warp_reduced;
    bar.step_ms = status_.step_ms;
    bar.physics_load = status_.physics_load;
    bar.load_limit = status_.load_limit;
    bar.sim_time_days = snapshot_ ? snapshot_->time().value() : 0.0;
    bar.scenarios = scenario_names_;
    bar.scenario = current_scenario_;
    bar.scenario_label = status_.scenario;
    bar.fps = fps;
    bar.message = status_.message;
    const render::AppBarIntent intent = render::draw_app_bar(bar);
    if (intent.reset) {
        physics_.send(sim::Reset{});
    }
    if (intent.set_paused) {
        physics_.send(sim::SetPaused{*intent.set_paused});
    }
    if (intent.single_step) {
        physics_.send(sim::SingleStep{});
    }
    if (intent.set_time_warp) {
        physics_.send(sim::SetTimeWarp{*intent.set_time_warp});
    }
    if (intent.load_scenario && *intent.load_scenario < request_.scenarios.size()) {
        requested_scenario_ = intent.load_scenario;
        physics_.send(sim::LoadScenario{request_.scenarios[*intent.load_scenario].path});
    }
}

void WindowApp::build_space_ui(const render::SceneInput& input, double dt, int width, int height) {
    const render::UiRect scene = ui_.dockspace();
    follow_scene_area(scene, width, height);
    if (reframe_pending_) {
        reframe_pending_ = false;
        reframe(scene, width, height);
    }
    handle_scene_input(input, dt);

    if (snapshot_) {
        const render::BodyInspectorIntent inspector =
            render::draw_body_inspector(render::make_body_inspector_data(*snapshot_, selected_));
        if (inspector.select) {
            selected_ = inspector.select;
        }
        if (inspector.focus_camera && selected_) {
            if (const auto index = snapshot_->index_of(*selected_)) {
                render::OrbitMode orbit;
                if (const auto* current = std::get_if<render::OrbitMode>(&camera_.goal())) {
                    orbit = *current;
                }
                orbit.target = *selected_;
                orbit.distance = focus_distance(*snapshot_, *index);
                camera_.set_mode(orbit);
                camera_moved_ = true;
            }
        }
    }

    render::SystemMonitorData monitor;
    monitor.sim_time_days = snapshot_ ? snapshot_->time().value() : 0.0;
    monitor.body_count = snapshot_ ? snapshot_->size() : 0;
    monitor.frame_time_ms = frame_seconds_ * 1000.0;
    monitor.physics_step_ms = status_.step_ms;
    monitor.integrator = status_.integrator;
    render::WarpReadout warp;
    warp.requested = status_.time_warp;
    warp.achieved = status_.effective_warp;
    warp.limited = status_.warp_reduced && !status_.paused;
    warp.load = status_.physics_load;
    warp.load_limit = status_.load_limit;
    monitor.warp = warp;
    monitor.series_time_days = drift_.time_days;
    monitor.series_values = drift_.values;
    if (render::draw_system_monitor(monitor).clear_series) {
        drift_.clear();
    }
    draw_space_experiment();

    render::RenderSettingsContext panel_context;
    if (snapshot_) {
        for (std::size_t i = 0; i < snapshot_->size(); ++i) {
            panel_context.black_hole |= snapshot_->kind_tag(i) == core::BodyKindTag::black_hole;
        }
    }
    if (const auto changed = render::draw_render_settings(settings_, panel_context).changed) {
        const bool trails_changed = changed->trail_length != settings_.trail_length;
        settings_ = *changed;
        if (trails_changed) {
            trails_.set_settings(trail_settings(settings_));
        }
    }
}

void WindowApp::draw_aero_scene(const render::FrameInfo& frame, double dt, std::ostream& err) {
    if (!aero_view_ && !aero_view_failed_) {
        gl_.make_current();
        auto created = AeroViewport::create();
        if (created) {
            aero_view_.emplace(std::move(created).value());
        } else {
            // Reported once; the panels keep working without the 3D view.
            aero_view_failed_ = true;
            err << "orrery_app: aero view: " << created.error() << '\n';
        }
    }
    if (!aero_view_) {
        return;
    }
    const render::Camera& camera = aero_.camera(aero_scene_, frame.width, frame.height);
    if (auto drawn = aero_view_->draw(aero_.view_state(), camera, frame, dt); !drawn) {
        if (!aero_view_failed_) {
            err << "orrery_app: aero view: " << drawn.error() << '\n';
        }
        aero_view_failed_ = true;
    }
    std::string problem = aero_view_->problem();
    if (problem != aero_view_problem_ && !problem.empty()) {
        err << "orrery_app: aero view: " << problem << '\n';
    }
    aero_view_problem_ = problem;
    aero_.set_view_problem(std::move(problem));
}

void WindowApp::draw_entry_scene(const render::FrameInfo& frame, std::ostream& err) {
    if (!entry_view_ && !entry_view_failed_) {
        gl_.make_current();
        auto created = EntryViewport::create();
        if (created) {
            entry_view_.emplace(std::move(created).value());
        } else {
            // Reported once; the panels keep working without the 3D view.
            entry_view_failed_ = true;
            err << "orrery_app: entry view: " << created.error() << '\n';
        }
    }
    if (!entry_view_) {
        return;
    }
    const render::Camera& camera = entry_.camera(entry_scene_, frame.width, frame.height);
    if (auto drawn = entry_view_->draw(entry_.view_state(), camera, frame); !drawn) {
        if (!entry_view_failed_) {
            err << "orrery_app: entry view: " << drawn.error() << '\n';
        }
        entry_view_failed_ = true;
    }
    std::string problem = entry_view_->problem();
    if (problem != entry_view_problem_ && !problem.empty()) {
        err << "orrery_app: entry view: " << problem << '\n';
    }
    entry_view_problem_ = problem;
    entry_.set_view_problem(std::move(problem));
}

void WindowApp::draw_space_scene(const render::FrameInfo& frame, double dt) {
    camera_.update(dt, camera_input_, body_frames(snapshot_));
    if (snapshot_) {
        const render::PreparedBodies prepared =
            render::prepare_bodies(*snapshot_, camera_, frame, settings_);
        const std::vector<render::SceneLabel> labels =
            render::make_scene_labels(*snapshot_, prepared.screen, selected_);
        render::draw_scene_labels(labels);
    }
    pipeline_->set_exposure(settings_.exposure);
    pipeline_->begin_scene();
    if (snapshot_) {
        const render::SpaceView view{snapshot_.get(), &trails_, &camera_, settings_};
        (void)space_.draw(view, frame);
    }
    pipeline_->resolve();
    pipeline_->begin_overlay();
}

bool WindowApp::draw_frame(int width, int height, double dt, std::ostream& err) {
    if (!pipeline_ || pipeline_->frame().width != width || pipeline_->frame().height != height) {
        gl_.make_current();
        pipeline_.reset();
        render::HdrSettings hdr;
        hdr.samples = 4;
        auto created = render::HdrPipeline::create(width, height, hdr);
        if (!created) {
            err << "orrery_app: frame pipeline " << width << 'x' << height << ": "
                << created.error() << '\n';
            return false;
        }
        pipeline_.emplace(std::move(created).value());
    }
    if (workspace_ == Workspace::aero) {
        aero_.update(dt);
    } else if (workspace_ == Workspace::entry) {
        entry_.update(dt);
    }
    build_ui(dt, width, height);
    const render::FrameInfo frame{width, height};
    if (workspace_ == Workspace::aero) {
        pipeline_->set_exposure(1.0); // the aero view's colours assume exposure 1
        pipeline_->begin_scene();
        draw_aero_scene(frame, dt, err);
        pipeline_->resolve();
        pipeline_->begin_overlay();
        if (aero_view_) {
            aero_view_->draw_overlay(frame);
        }
    } else if (workspace_ == Workspace::entry) {
        pipeline_->set_exposure(1.0); // the entry view's colours assume exposure 1
        pipeline_->begin_scene();
        draw_entry_scene(frame, err);
        pipeline_->resolve();
        pipeline_->begin_overlay();
    } else {
        draw_space_scene(frame, dt);
    }
    ui_.end_frame();
    pipeline_->present(width, height);
    return true;
}

WindowControl WindowApp::drive_before_frame() {
    WindowControl control;
    if (driver_ == nullptr) {
        return control;
    }
    ui_.make_current();
    driver_->before_frame(frames_, control);
    if (control.window_size) {
        gl_.set_window_size(control.window_size->first, control.window_size->second);
    }
    return control;
}

void WindowApp::report_frame(bool drawn, int width, int height, double frame_seconds) {
    if (driver_ == nullptr) {
        return;
    }
    gl_.make_current();
    gl_errors_ += gl_.drain_gl_errors();
    WindowFrame frame;
    frame.index = frames_;
    frame.drawn = drawn;
    frame.width = width;
    frame.height = height;
    frame.content_scale = gl_.content_scale();
    frame.frame_seconds = frame_seconds;
    frame.workspace = workspace_;
    frame.help_open = help_open_;
    frame.warp_limited_shown = warp_limited_hold_ > 0.0;
    frame.status = &status_;
    frame.snapshot = snapshot_;
    frame.scenario = current_scenario_;
    frame.aero = &aero_;
    frame.entry = &entry_;
    frame.space_panel = &space_panel_;
    frame.ui = &ui_;
    frame.gl = &gl_;
    frame.gl_errors = gl_errors_ + gl_.debug_error_count();
    driver_->after_frame(frame);
}

int WindowApp::run(std::ostream& err) {
    Clock::time_point last = Clock::now();
    BenchmarkStats benchmark(kBenchmarkWarmupFrames);

    while (!gl_.should_close()) {
        ORRERY_FRAME_ZONE();
        gl_.poll_events();
        const WindowControl control = drive_before_frame();
        const int width = control.empty_framebuffer ? 0 : gl_.framebuffer_width();
        const int height = control.empty_framebuffer ? 0 : gl_.framebuffer_height();
        physics_.wake();
        const Clock::time_point now = Clock::now();
        // Interactive updates use the clamped delta, statistics the measured one.
        const FrameDelta delta = frame_delta(std::chrono::duration<double>(now - last).count());
        if (width <= 0 || height <= 0) {
            // Minimised: keep the simulation running, render nothing.
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
            if (physics_.fetch()) {
                take(physics_.latest());
            }
            last = Clock::now();
            report_frame(false, width, height, delta.raw_seconds);
            ++frames_;
            if (control.close) {
                break;
            }
            continue;
        }
        const double dt = delta.interactive_seconds;
        last = now;
        frame_seconds_ += (dt - frame_seconds_) * 0.05;
        warp_limited_hold_ = std::max(0.0, warp_limited_hold_ - dt);
        if (physics_.fetch()) {
            take(physics_.latest());
        }
        if (!draw_frame(width, height, dt, err)) {
            return kExitFailure;
        }
        report_frame(true, width, height, delta.raw_seconds);
        gl_.swap_buffers();
        ++frames_;
        if (control.close) {
            break;
        }
        if (request_.benchmark_frames > 0) {
            benchmark.add(delta);
            if (frames_ >= request_.benchmark_frames + kBenchmarkWarmupFrames) {
                out_ << "benchmark: " << benchmark.frames() << " frames at " << width << 'x'
                     << height << " (" << status_.scenario << ", " << snapshot_->size()
                     << " bodies): mean " << benchmark.mean_ms() << " ms, " << benchmark.fps()
                     << " fps\n";
                return kExitOk;
            }
        }
    }
    return kExitOk;
}

} // namespace

int run_window(WindowRequest request, std::ostream& out, std::ostream& err,
               const WindowSession& session) {
    render::ContextOptions options;
    options.width = request.width;
    options.height = request.height;
    options.title = "Orrery";
    options.visible = !session.hidden;
    options.debug = session.gl_debug;
    // A hidden window is never on screen: its swaps wait for nothing.
    options.swap_interval = request.benchmark_frames > 0 || session.hidden ? 0 : 1;
    auto context = render::GlContext::create(options);
    if (!context) {
        err << "orrery_app: cannot open an OpenGL 4.6 window: " << context.error() << '\n';
        return kExitNoGlContext;
    }
    render::GlContext gl = std::move(context).value();
    auto ui_created = render::UiContext::create(gl, render::UiOptions{}, true);
    if (!ui_created) {
        err << "orrery_app: " << ui_created.error() << '\n';
        return kExitFailure;
    }
    render::UiContext ui = std::move(ui_created).value();
    gl.make_current();
    auto space_created = render::SpaceRenderer::create();
    if (!space_created) {
        err << "orrery_app: space renderer: " << space_created.error() << '\n';
        return kExitFailure;
    }
    render::SpaceRenderer space = std::move(space_created).value();

    sim::SimulationConfig config = request.experiment.config;
    config.step_time_budget_s = kInteractiveStepBudget;
    // The time-warp budget (docs/app.md, "Time-warp budget"): the warp is
    // lowered while stepping would take more than this share of the time.
    sim::WarpBudget warp_budget;
    warp_budget.load_limit = request.config.physics_budget;
    config.warp_budget = warp_budget;
    auto simulation = sim::Simulation::create(request.experiment.scene, config);
    if (!simulation) {
        err << "orrery_app: " << simulation.error() << '\n';
        return kExitFailure;
    }
    sim::PhysicsThread physics(std::move(simulation).value());
    WindowApp app(std::move(request), gl, ui, space, physics, out, session.driver);
    const int code = app.run(err);
    gl.make_current();
    return code;
}

} // namespace orrery::app
