#include "orrery/app/headless.h"

#include <ostream>
#include <utility>

#include "orrery/app/space_view.h"
#include "orrery/render/gl_context.h"
#include "orrery/render/hdr_pipeline.h"
#include "orrery/render/png.h"
#include "orrery/render/snapshot.h"
#include "orrery/render/space_renderer.h"
#include "orrery/render/trails.h"

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

} // namespace

core::Result<sim::Simulation, std::string> advance_headless(const sim::SpaceExperiment& experiment,
                                                            std::int64_t frames) {
    return run_frames(experiment, frames, nullptr);
}

core::Result<HeadlessFrame, HeadlessError> render_headless(const sim::SpaceExperiment& experiment,
                                                           const HeadlessRequest& request) {
    const render::RenderSettings settings = render::sanitized(request.settings);
    render::TrailHistory trails(trail_settings(settings));
    auto advanced = run_frames(experiment, request.frames, &trails);
    if (!advanced) {
        return core::failure(HeadlessError{HeadlessErrorKind::failure, advanced.error()});
    }
    sim::Simulation& simulation = advanced.value();
    HeadlessFrame result;
    result.snapshot = simulation.snapshot();
    result.status = simulation.status();

    auto context = render::GlContext::create(render::OffscreenRenderer::context_options());
    if (!context) {
        return core::failure(HeadlessError{HeadlessErrorKind::no_context, context.error()});
    }
    render::HdrSettings hdr;
    hdr.samples = request.msaa_samples;
    hdr.exposure = settings.exposure;
    auto renderer = render::OffscreenRenderer::create(std::move(context).value(), request.width,
                                                      request.height, hdr);
    if (!renderer) {
        return core::failure(
            HeadlessError{HeadlessErrorKind::failure, "renderer setup: " + renderer.error()});
    }
    auto space = render::SpaceRenderer::create();
    if (!space) {
        return core::failure(
            HeadlessError{HeadlessErrorKind::failure, "space renderer setup: " + space.error()});
    }
    const render::FrameInfo frame{request.width, request.height};
    const render::Camera camera = framing_camera(*result.snapshot, frame.aspect());
    const render::SpaceView view{result.snapshot.get(), &trails, &camera, settings};
    auto image = renderer.value().render(
        [&](const render::FrameInfo& info) { (void)space.value().draw(view, info); });
    if (!image) {
        return core::failure(HeadlessError{HeadlessErrorKind::failure, "render: " + image.error()});
    }
    result.image = std::move(image).value();
    return result;
}

std::vector<std::filesystem::path> experiment_search_dirs(const std::filesystem::path& executable) {
    std::vector<std::filesystem::path> dirs;
    for (const std::filesystem::path& assets : asset_directories(executable)) {
        dirs.push_back(assets / "experiments");
    }
    return dirs;
}

int run_headless_command(const CliOptions& options,
                         std::span<const std::filesystem::path> search_dirs, std::ostream& out,
                         std::ostream& err) {
    const auto path = sim::resolve_experiment(options.experiment.value_or(""), search_dirs);
    if (!path) {
        err << "orrery_app: " << path.error() << '\n';
        return kExitFailure;
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
    return kExitOk;
}

} // namespace orrery::app
