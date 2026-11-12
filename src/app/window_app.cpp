#include "window_app.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include "orrery/app/cli.h"
#include "orrery/app/frame_stats.h"
#include "orrery/app/space_view.h"
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
          out_(out), trails_(trail_settings(settings_)) {
        for (const sim::ScenarioEntry& entry : request_.scenarios) {
            scenario_names_.push_back(entry.name);
        }
        current_scenario_ = scenario_index(request_.scenarios, request_.experiment.scenario_path);
        take(physics_.latest());
    }

    // Runs until the window closes (or the benchmark ends); returns an exit
    // code.
    int run(std::ostream& err);

private:
    void take(const sim::PublishedFrame& frame);
    void reframe(const render::UiRect& scene, int window_height);
    void build_ui(double dt, int width, int height);
    void handle_scene_input(const render::SceneInput& input, double dt);

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

    render::AppBarData bar;
    bar.paused = status_.paused;
    bar.time_warp = status_.time_warp;
    bar.effective_warp = status_.effective_warp;
    bar.warp_limited = warp_limited_hold_ > 0.0;
    bar.sim_time_days = snapshot_ ? snapshot_->time().value() : 0.0;
    bar.scenarios = scenario_names_;
    bar.scenario = current_scenario_;
    bar.scenario_label = status_.scenario;
    bar.fps = frame_seconds_ > 0.0 ? 1.0 / frame_seconds_ : 0.0;
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

    const render::UiRect scene = ui_.dockspace();
    if (reframe_pending_) {
        reframe_pending_ = false;
        reframe(scene, height);
    }
    handle_scene_input(render::read_scene_input(), dt);

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
    monitor.series_time_days = drift_.time_days;
    monitor.series_values = drift_.values;
    if (render::draw_system_monitor(monitor).clear_series) {
        drift_.clear();
    }

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

        build_ui(dt, width, height);
        camera_.update(dt, camera_input_, body_frames(snapshot_));
        const render::FrameInfo frame{width, height};
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
