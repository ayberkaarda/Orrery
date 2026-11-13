#include "window_app.h"

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

double focus_distance(const core::SimSnapshot& snapshot, std::size_t index) {
    const render::BodyAppearance appearance = render::appearance_for(
        snapshot.names()[index], snapshot.kinds()[index], snapshot.masses()[index]);
    return std::max(12.0 * render::physical_radius(snapshot.radii()[index], appearance), 1.0e6);
}

class WindowApp {
public:
    WindowApp(WindowRequest request, render::GlContext& gl, render::UiContext& ui,
              render::SpaceRenderer& space, sim::PhysicsThread& physics, std::ostream& out)
        : request_(std::move(request)), gl_(gl), ui_(ui), space_(space), physics_(physics),
          out_(out), trails_(trail_settings(settings_)), workspace_(request_.workspace),
          aero_(aero_options(request_.config)), entry_(entry_options(request_.config)),
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
    void handle_shortcuts(std::span<const ShortcutAction> actions);
    void save_experiment();
    void open_experiments();
    void draw_space_experiment();
    void reframe(const render::UiRect& scene, int window_height);
    void build_ui(double dt, int width, int height);
    void draw_space_bar(double fps);
    void build_space_ui(const render::SceneInput& input, double dt, int height);
    void handle_scene_input(const render::SceneInput& input, double dt);
    void draw_aero_scene(const render::FrameInfo& frame, double dt, std::ostream& err);
    void draw_entry_scene(const render::FrameInfo& frame, std::ostream& err);

    WindowRequest request_;
    render::GlContext& gl_;
    render::UiContext& ui_;
    render::SpaceRenderer& space_;
    sim::PhysicsThread& physics_;
    std::ostream& out_;

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

    // Phase 10: the help overlay of the keyboard shortcuts and the Space
    // workspace's Experiment panel.
    bool help_open_ = false;
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
// is computed for that area's aspect and moved back so that the area's
// height (not the window's) spans the field of view.
void WindowApp::reframe(const render::UiRect& scene, int window_height) {
    if (!snapshot_) {
        return;
    }
    render::CameraLens lens;
    lens.vertical_fov = kSpaceFovDegrees * 3.141592653589793 / 180.0;
    (void)camera_.set_lens(lens);
    const bool usable = scene.width > 1.0 && scene.height > 1.0;
    render::OrbitMode orbit = framing_orbit(*snapshot_, usable ? scene.width / scene.height : 1.0);
    if (usable) {
        orbit.distance *= std::max(1.0, static_cast<double>(window_height) / scene.height);
    }
    camera_.set_mode(orbit);
    camera_.update(0.0, render::CameraInput{}, body_frames(snapshot_));
}

void WindowApp::handle_scene_input(const render::SceneInput& input, double dt) {
    if (input.toggle_pause) {
        physics_.send(sim::SetPaused{!status_.paused});
    }
    if (input.single_step) {
        physics_.send(sim::SingleStep{});
    }
    if (input.reset_view) {
        reframe_pending_ = true;
    }
    camera_input_ = render::CameraInput{};
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
    // One table drives every key (docs/app.md, "Keyboard shortcuts").
    const std::vector<ShortcutAction> actions =
        match_shortcuts(request_.config.shortcuts, read_key_frame(), workspace_);
    handle_shortcuts(actions);
    const render::SceneInput scene_input = scene_input_from(render::read_scene_input(), actions);
    if (workspace_ == Workspace::aero) {
        aero_scene_ = aero_.draw_panels();
        aero_.handle_scene_input(scene_input, dt);
    } else if (workspace_ == Workspace::entry) {
        entry_scene_ = entry_.draw_panels();
        entry_.handle_scene_input(scene_input, dt);
    } else {
        build_space_ui(scene_input, dt, height);
    }
    if (std::find(actions.begin(), actions.end(), ShortcutAction::open) != actions.end()) {
        open_experiments(); // after the panels exist this frame
    }
    draw_shortcut_help(request_.config.shortcuts, workspace_, help_open_,
                       request_.config_path.string());
}

void WindowApp::handle_shortcuts(std::span<const ShortcutAction> actions) {
    for (const ShortcutAction action : actions) {
        switch (action) {
            case ShortcutAction::workspace_space:
                workspace_ = Workspace::space;
                break;
            case ShortcutAction::workspace_aero:
                workspace_ = Workspace::aero;
                break;
            case ShortcutAction::workspace_entry:
                workspace_ = Workspace::entry;
                break;
            case ShortcutAction::help:
                help_open_ = !help_open_;
                break;
            case ShortcutAction::close:
                if (help_open_) {
                    help_open_ = false;
                } else if (workspace_ == Workspace::space) {
                    selected_.reset();
                } else if (workspace_ == Workspace::entry) {
                    entry_.set_cursor_time(std::nullopt);
                }
                break;
            case ShortcutAction::save:
                save_experiment();
                break;
            case ShortcutAction::warp_down:
            case ShortcutAction::warp_up:
                physics_.send(sim::SetTimeWarp{render::step_time_warp(
                    status_.time_warp, action == ShortcutAction::warp_up ? +1 : -1)});
                break;
            case ShortcutAction::reset:
                physics_.send(sim::Reset{});
                break;
            case ShortcutAction::open:
            case ShortcutAction::toggle_pause:
            case ShortcutAction::single_step:
            case ShortcutAction::reset_view:
                break; // after the panels, and through the scene input
        }
    }
}

void WindowApp::save_experiment() {
    if (workspace_ == Workspace::aero) {
        (void)aero_.save();
    } else if (workspace_ == Workspace::entry) {
        (void)entry_.save();
    } else {
        const std::filesystem::path scenario = status_.scenario_file.empty()
                                                   ? request_.experiment.scenario_path
                                                   : status_.scenario_file;
        (void)space_panel_.save(scenario, status_, snapshot_ ? snapshot_->time().value() : 0.0);
    }
}

void WindowApp::open_experiments() {
    if (workspace_ == Workspace::aero) {
        focus_window(kExperimentsWindow);
    } else if (workspace_ == Workspace::entry) {
        entry_.refresh_experiments();
        focus_window(kEntryExperimentsWindow);
    } else {
        space_panel_.refresh();
        focus_window(kSpaceExperimentWindow);
    }
}

void WindowApp::draw_space_experiment() {
    SpaceRunView run;
    run.scenario = status_.scenario;
    run.experiment = status_.experiment_file.empty()
                         ? request_.experiment.experiment_path.filename().string()
                         : status_.experiment_file.filename().string();
    run.integrator = status_.integrator;
    run.force_model = status_.force_model;
    run.dt_days = status_.dt_days;
    run.time_warp = status_.time_warp;
    run.sim_time_days = snapshot_ ? snapshot_->time().value() : 0.0;
    const SpaceExperimentIntent intent = space_panel_.draw(run);
    if (intent.save) {
        save_experiment();
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

void WindowApp::build_space_ui(const render::SceneInput& input, double dt, int height) {
    const render::UiRect scene = ui_.dockspace();
    if (reframe_pending_) {
        reframe_pending_ = false;
        reframe(scene, height);
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

int WindowApp::run(std::ostream& err) {
    std::optional<render::HdrPipeline> pipeline;
    int pipeline_width = 0;
    int pipeline_height = 0;
    Clock::time_point last = Clock::now();
    std::int64_t frames = 0;
    BenchmarkStats benchmark(kBenchmarkWarmupFrames);

    while (!gl_.should_close()) {
        gl_.poll_events();
        const int width = gl_.framebuffer_width();
        const int height = gl_.framebuffer_height();
        physics_.wake();
        if (width <= 0 || height <= 0) {
            // Minimised: keep the simulation running, render nothing.
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
            if (physics_.fetch()) {
                take(physics_.latest());
            }
            last = Clock::now();
            continue;
        }
        if (!pipeline || width != pipeline_width || height != pipeline_height) {
            gl_.make_current();
            pipeline.reset();
            render::HdrSettings hdr;
            hdr.samples = 4;
            auto created = render::HdrPipeline::create(width, height, hdr);
            if (!created) {
                err << "orrery_app: frame pipeline " << width << 'x' << height << ": "
                    << created.error() << '\n';
                return kExitFailure;
            }
            pipeline.emplace(std::move(created).value());
            pipeline_width = width;
            pipeline_height = height;
        }

        const Clock::time_point now = Clock::now();
        // Interactive updates use the clamped delta, statistics the measured one.
        const FrameDelta delta = frame_delta(std::chrono::duration<double>(now - last).count());
        const double dt = delta.interactive_seconds;
        last = now;
        frame_seconds_ += (dt - frame_seconds_) * 0.05;
        warp_limited_hold_ = std::max(0.0, warp_limited_hold_ - dt);

        if (physics_.fetch()) {
            take(physics_.latest());
        }

        if (workspace_ == Workspace::aero) {
            aero_.update(dt);
        } else if (workspace_ == Workspace::entry) {
            entry_.update(dt);
        }
        build_ui(dt, width, height);
        const render::FrameInfo frame{width, height};
        if (workspace_ == Workspace::aero) {
            pipeline->set_exposure(1.0); // the aero view's colours assume exposure 1
            pipeline->begin_scene();
            draw_aero_scene(frame, dt, err);
            pipeline->resolve();
            pipeline->begin_overlay();
            if (aero_view_) {
                aero_view_->draw_overlay(frame);
            }
        } else if (workspace_ == Workspace::entry) {
            pipeline->set_exposure(1.0); // the entry view's colours assume exposure 1
            pipeline->begin_scene();
            draw_entry_scene(frame, err);
            pipeline->resolve();
            pipeline->begin_overlay();
        } else {
            camera_.update(dt, camera_input_, body_frames(snapshot_));
            if (snapshot_) {
                const render::PreparedBodies prepared =
                    render::prepare_bodies(*snapshot_, camera_, frame, settings_);
                const std::vector<render::SceneLabel> labels =
                    render::make_scene_labels(*snapshot_, prepared.screen, selected_);
                render::draw_scene_labels(labels);
            }
            pipeline->set_exposure(settings_.exposure);
            pipeline->begin_scene();
            if (snapshot_) {
                const render::SpaceView view{snapshot_.get(), &trails_, &camera_, settings_};
                (void)space_.draw(view, frame);
            }
            pipeline->resolve();
            pipeline->begin_overlay();
        }
        ui_.end_frame();
        pipeline->present(width, height);
        gl_.swap_buffers();

        ++frames;
        if (request_.benchmark_frames > 0) {
            benchmark.add(delta);
            if (frames >= request_.benchmark_frames + kBenchmarkWarmupFrames) {
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

int run_window(WindowRequest request, std::ostream& out, std::ostream& err) {
    render::ContextOptions options;
    options.width = request.width;
    options.height = request.height;
    options.title = "Orrery";
    options.visible = true;
    options.debug = false;
    options.swap_interval = request.benchmark_frames > 0 ? 0 : 1;
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
    WindowApp app(std::move(request), gl, ui, space, physics, out);
    const int code = app.run(err);
    gl.make_current();
    return code;
}

} // namespace orrery::app
