#include "orrery/app/headless.h"

#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <ostream>
#include <utility>
#include <vector>

#include "orrery/app/aero_headless.h"
#include "orrery/app/frame_stats.h"
#include "orrery/app/space_view.h"
#include "orrery/render/gl_context.h"
#include "orrery/render/hdr_pipeline.h"
#include "orrery/render/png.h"
#include "orrery/render/snapshot.h"
#include "orrery/render/space_renderer.h"
#include "orrery/render/trails.h"
#include "orrery/scene/experiment.h"
#include "orrery/scene/scenario.h"
#include "orrery/sim/aero_flow.h"

namespace orrery::app {

namespace {

core::Result<sim::Simulation, std::string> run_frames(const sim::SpaceExperiment& experiment,
                                                      std::int64_t frames,
                                                      render::TrailHistory* trails) {
    sim::SimulationConfig config = experiment.config;
    config.step_time_budget_s = 0.0; // deterministic: the step count never depends on timing
    auto created = sim::Simulation::create(experiment.scene, config);
    if (!created) {
        return core::failure(created.error());
    }
    sim::Simulation simulation = std::move(created).value();
    if (trails != nullptr) {
        trails->record(*simulation.snapshot());
    }
    for (std::int64_t frame = 0; frame < frames; ++frame) {
        const sim::AdvanceResult advanced = simulation.advance(kFrameWallNs);
        if (advanced.failed) {
            return core::failure("frame " + std::to_string(frame + 1) + ": " +
                                 simulation.status().message);
        }
        if (trails != nullptr && advanced.steps > 0) {
            trails->record(*simulation.snapshot());
        }
    }
    return simulation;
}

// Everything a headless frame needs: the advanced simulation, its trails,
// an offscreen renderer and the space renderer built in its context, and the
// window's framing camera.
struct HeadlessScene {
    render::RenderSettings settings;
    render::TrailHistory trails;
    std::shared_ptr<const core::SimSnapshot> snapshot;
    sim::SimStatus status;
    std::optional<render::OffscreenRenderer> renderer;
    std::optional<render::SpaceRenderer> space;
    render::Camera camera;
};

core::Result<std::unique_ptr<HeadlessScene>, HeadlessError>
prepare_scene(const sim::SpaceExperiment& experiment, const HeadlessRequest& request) {
    auto scene = std::make_unique<HeadlessScene>();
    scene->settings = render::sanitized(request.settings);
    scene->trails = render::TrailHistory(trail_settings(scene->settings));
    auto advanced = run_frames(experiment, request.frames, &scene->trails);
    if (!advanced) {
        return core::failure(HeadlessError{HeadlessErrorKind::failure, advanced.error()});
    }
    scene->snapshot = advanced.value().snapshot();
    scene->status = advanced.value().status();

    auto context = render::GlContext::create(render::OffscreenRenderer::context_options());
    if (!context) {
        return core::failure(HeadlessError{HeadlessErrorKind::no_context, context.error()});
    }
    render::HdrSettings hdr;
    hdr.samples = request.msaa_samples;
    hdr.exposure = scene->settings.exposure;
    auto renderer = render::OffscreenRenderer::create(std::move(context).value(), request.width,
                                                      request.height, hdr);
    if (!renderer) {
        return core::failure(
            HeadlessError{HeadlessErrorKind::failure, "renderer setup: " + renderer.error()});
    }
    scene->renderer.emplace(std::move(renderer).value());
    auto space = render::SpaceRenderer::create();
    if (!space) {
        return core::failure(
            HeadlessError{HeadlessErrorKind::failure, "space renderer setup: " + space.error()});
    }
    scene->space.emplace(std::move(space).value());
    const render::FrameInfo frame{request.width, request.height};
    scene->camera = framing_camera(*scene->snapshot, frame.aspect());
    return scene;
}

double percentile(std::vector<double> values, double fraction) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const auto index = static_cast<std::size_t>(fraction * static_cast<double>(values.size() - 1));
    return values[index];
}

} // namespace

core::Result<HeadlessFrame, HeadlessError> render_headless(const sim::SpaceExperiment& experiment,
                                                           const HeadlessRequest& request) {
    auto prepared = prepare_scene(experiment, request);
    if (!prepared) {
        const HeadlessError& cause = prepared.error();
        return core::failure(HeadlessError{cause.kind, cause.message});
    }
    HeadlessScene& prepared_scene = *prepared.value();
    HeadlessFrame result;
    result.snapshot = prepared_scene.snapshot;
    result.status = prepared_scene.status;
    const render::SpaceView view{prepared_scene.snapshot.get(), &prepared_scene.trails,
                                 &prepared_scene.camera, prepared_scene.settings};
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access): set by prepare_scene.
    render::SpaceRenderer& space = *prepared_scene.space;
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access): set by prepare_scene.
    auto image = prepared_scene.renderer->render(
        [&](const render::FrameInfo& info) { (void)space.draw(view, info); });
    if (!image) {
        return core::failure(HeadlessError{HeadlessErrorKind::failure, "render: " + image.error()});
    }
    result.image = std::move(image).value();
    return result;
}

core::Result<HeadlessBenchmark, HeadlessError>
benchmark_headless(const sim::SpaceExperiment& experiment, const HeadlessRequest& request,
                   std::int64_t frames) {
    auto prepared = prepare_scene(experiment, request);
    if (!prepared) {
        const HeadlessError& cause = prepared.error();
        return core::failure(HeadlessError{cause.kind, cause.message});
    }
    HeadlessScene& prepared_scene = *prepared.value();
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access): set by prepare_scene.
    render::OffscreenRenderer& renderer = *prepared_scene.renderer;
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access): set by prepare_scene.
    render::SpaceRenderer& space = *prepared_scene.space;
    renderer.context().make_current();
    render::HdrPipeline& pipeline = renderer.pipeline();
    const render::SpaceView view{prepared_scene.snapshot.get(), &prepared_scene.trails,
                                 &prepared_scene.camera, prepared_scene.settings};
    const render::GpuTimer timer = render::GpuTimer::create();
    BenchmarkStats stats(kHeadlessBenchmarkWarmup);
    std::vector<double> gpu_ms;
    HeadlessBenchmark result;
    for (std::int64_t frame = 0; frame < frames + kHeadlessBenchmarkWarmup; ++frame) {
        const auto start = std::chrono::steady_clock::now();
        timer.begin();
        pipeline.begin_scene();
        const render::SpaceDrawStats drawn = space.draw(view, pipeline.frame());
        pipeline.resolve();
        timer.end();
        const double gpu = static_cast<double>(timer.elapsed_ns()) * 1.0e-6; // waits for the GPU
        const auto done = std::chrono::steady_clock::now();
        stats.add(frame_delta(std::chrono::duration<double>(done - start).count()));
        if (frame >= kHeadlessBenchmarkWarmup) {
            gpu_ms.push_back(gpu);
        }
        result.lensing = drawn.lensing;
    }
    if (renderer.context().debug_error_count() > 0) {
        return core::failure(HeadlessError{HeadlessErrorKind::failure,
                                           "benchmark: the driver reported OpenGL errors"});
    }
    result.frames = stats.frames();
    result.wall_mean_ms = stats.mean_ms();
    result.fps = stats.fps();
    result.gpu_median_ms = percentile(gpu_ms, 0.5);
    result.gpu_p95_ms = percentile(gpu_ms, 0.95);
    result.bodies = prepared_scene.snapshot->size();
    result.renderer = renderer.context().info().renderer;
    return result;
}

core::Result<sim::Simulation, std::string> advance_headless(const sim::SpaceExperiment& experiment,
                                                            std::int64_t frames) {
    return run_frames(experiment, frames, nullptr);
}

std::vector<std::filesystem::path> experiment_search_dirs(const std::filesystem::path& executable) {
    std::vector<std::filesystem::path> dirs;
    for (const std::filesystem::path& assets : asset_directories(executable)) {
        dirs.push_back(assets / "experiments");
    }
    return dirs;
}

namespace {

int run_aero_headless(const CliOptions& options, const std::filesystem::path& path,
                      const scene::Experiment& generic, std::ostream& out, std::ostream& err) {
    const auto experiment = scene::aero_from_experiment(generic);
    if (!experiment) {
        err << "orrery_app: experiment " << path.string() << ": " << experiment.error() << '\n';
        return kExitFailure;
    }
    if (options.benchmark_frames > 0) {
        err << "orrery_app: --benchmark times the space view; it is not available for aero "
               "experiments\n";
        return kExitFailure;
    }
    AeroHeadlessRequest request;
    request.width = options.width.value_or(request.width);
    request.height = options.height.value_or(request.height);
    request.frames = options.frame;
    const auto frame = render_aero_headless(experiment.value(), path.parent_path(), request);
    if (!frame) {
        if (frame.error().kind == HeadlessErrorKind::no_context) {
            err << "orrery_app: no OpenGL 4.6 core context for headless rendering: "
                << frame.error().message << '\n';
            return kExitNoGlContext;
        }
        err << "orrery_app: " << frame.error().message << '\n';
        return kExitFailure;
    }
    const auto written = render::write_png(options.out, frame.value().image);
    if (!written) {
        err << "orrery_app: cannot write " << options.out.string() << ": " << written.error()
            << '\n';
        return kExitFailure;
    }
    const sim::SolveOutcome& o = frame.value().outcome;
    out << "wrote " << options.out.string() << " (" << request.width << 'x' << request.height
        << "): " << frame.value().label << ", " << sim::solver_display_name(o.solver) << ", AoA "
        << experiment.value().conditions.angle_of_attack_deg << " deg, Re "
        << sim::format_significant(o.flow.reynolds, 3) << ", Mach "
        << sim::format_significant(o.flow.mach, 3) << ": Cd " << o.result.cd << ", Cl "
        << o.result.cl << ", Cm " << o.result.cm;
    if (o.diagnostics.planned_steps > 0) {
        out << ", " << o.diagnostics.lattice_steps << " of " << o.diagnostics.planned_steps
            << " lattice steps";
    }
    out << (o.result.converged ? ", converged" : ", not converged") << ", "
        << o.result.warnings.size() << " warning(s)\n";
    for (const aero::ValidityWarning& warning : o.result.warnings) {
        out << "  warning " << warning.code << ": " << warning.message << '\n';
    }
    return kExitOk;
}

} // namespace

int run_headless_command(const CliOptions& options,
                         std::span<const std::filesystem::path> search_dirs, std::ostream& out,
                         std::ostream& err) {
    const auto path = sim::resolve_experiment(options.experiment.value_or(""), search_dirs);
    if (!path) {
        err << "orrery_app: " << path.error() << '\n';
        return kExitFailure;
    }
    // The domain decides the renderer; the space loader reports every
    // problem of a file that is not an aero experiment.
    if (const auto text = scene::read_text_file(path.value())) {
        if (const auto generic = scene::parse_experiment(text.value());
            generic && generic.value().domain == scene::Domain::aero) {
            return run_aero_headless(options, path.value(), generic.value(), out, err);
        }
    }
    const auto experiment = sim::load_space_experiment(path.value());
    if (!experiment) {
        err << "orrery_app: " << experiment.error() << '\n';
        return kExitFailure;
    }
    HeadlessRequest request;
    request.width = options.width.value_or(request.width);
    request.height = options.height.value_or(request.height);
    request.frames = options.frame;
    const auto frame = render_headless(experiment.value(), request);
    if (!frame) {
        if (frame.error().kind == HeadlessErrorKind::no_context) {
            err << "orrery_app: no OpenGL 4.6 core context for headless rendering: "
                << frame.error().message << '\n';
            return kExitNoGlContext;
        }
        err << "orrery_app: " << frame.error().message << '\n';
        return kExitFailure;
    }
    const auto written = render::write_png(options.out, frame.value().image);
    if (!written) {
        err << "orrery_app: cannot write " << options.out.string() << ": " << written.error()
            << '\n';
        return kExitFailure;
    }
    const sim::SimStatus& status = frame.value().status;
    out << "wrote " << options.out.string() << " (" << request.width << 'x' << request.height
        << "): " << status.scenario << ", frame " << options.frame
        << ", t = " << frame.value().snapshot->time().value() << " d, " << status.steps
        << " steps of " << status.dt_days << " d (" << status.integrator << ", "
        << status.force_model << ")\n";
    if (options.benchmark_frames > 0) {
        const auto bench =
            benchmark_headless(experiment.value(), request, options.benchmark_frames);
        if (!bench) {
            err << "orrery_app: " << bench.error().message << '\n';
            return kExitFailure;
        }
        const HeadlessBenchmark& b = bench.value();
        out << "benchmark: " << b.frames << " frames at " << request.width << 'x' << request.height
            << " (" << status.scenario << ", " << b.bodies << " bodies, lensing "
            << (b.lensing ? "on" : "off") << "): mean " << b.wall_mean_ms << " ms, " << b.fps
            << " fps; GPU median " << b.gpu_median_ms << " ms, p95 " << b.gpu_p95_ms << " ms on "
            << b.renderer << '\n';
    }
    return kExitOk;
}

} // namespace orrery::app
