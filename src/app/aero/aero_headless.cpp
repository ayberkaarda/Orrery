#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "orrery/app/aero_headless.h"
#include "orrery/app/aero_scene.h"
#include "orrery/render/aero_view.h"
#include "orrery/render/gl_context.h"
#include "orrery/render/snapshot.h"
#include "orrery/sim/aero_import.h"
#include "orrery/sim/aero_view_data.h"

namespace orrery::app {

core::Result<AeroHeadlessFrame, HeadlessError>
render_aero_headless(const scene::AeroExperiment& experiment, const std::filesystem::path& base_dir,
                     const AeroHeadlessRequest& request) {
    auto prepared = sim::load_experiment_geometry(experiment.geometry, base_dir);
    if (!prepared) {
        const sim::GeometryError& cause = prepared.error();
        return core::failure(
            HeadlessError{HeadlessErrorKind::failure, cause.title + ": " + cause.detail});
    }
    sim::AeroRunSpec spec;
    spec.geometry = std::make_shared<const sim::PreparedGeometry>(std::move(prepared).value());
    spec.experiment = experiment;
    spec.experiment.result.reset();
    std::optional<std::uint64_t> limit;
    if (request.frames > 0) {
        limit = static_cast<std::uint64_t>(request.frames) *
                static_cast<std::uint64_t>(
                    std::max<std::int64_t>(1, experiment.solver.steps_per_frame));
    }
    auto solved = sim::solve(spec, limit);
    if (!solved) {
        return core::failure(
            HeadlessError{HeadlessErrorKind::failure, std::string(solved.error())});
    }
    AeroHeadlessFrame out;
    out.outcome = std::move(solved).value();
    out.label = spec.geometry->label;
    const sim::PreparedGeometry& geometry = *spec.geometry;
    const sim::SolveOutcome& outcome = out.outcome;

    // View data, as the solver thread prepares it for the window.
    const sim::ViewFrame frame = sim::view_frame(outcome.flow.flow, geometry.two_dimensional);
    const std::vector<double> cp = geometry.two_dimensional
                                       ? sim::panel_cp(geometry.section, outcome.result.surface)
                                       : sim::triangle_cp(geometry.mesh, outcome.result.surface);
    std::optional<sim::FlowSnapshot> flow;
    if (outcome.solver_instance && outcome.solver == aero::SolverKind::lattice_boltzmann) {
        auto captured = sim::capture_flow(*outcome.solver_instance, geometry, outcome.flow.flow,
                                          outcome.reference);
        if (!captured) {
            return core::failure(HeadlessError{HeadlessErrorKind::failure,
                                               "flow field: " + std::string(captured.error())});
        }
        flow = std::move(captured).value();
    }
    auto surface = view_surface(geometry, cp, frame);
    if (!surface) {
        return core::failure(
            HeadlessError{HeadlessErrorKind::failure, "surface: " + std::string(surface.error())});
    }
    const geom::Bounds bounds = view_bounds(surface.value().mesh);

    auto context = render::GlContext::create(render::OffscreenRenderer::context_options());
    if (!context) {
        return core::failure(HeadlessError{HeadlessErrorKind::no_context, context.error()});
    }
    render::HdrSettings hdr = render::aero_hdr_settings();
    hdr.samples = request.msaa_samples;
    auto renderer = render::OffscreenRenderer::create(std::move(context).value(), request.width,
                                                      request.height, hdr);
    if (!renderer) {
        return core::failure(
            HeadlessError{HeadlessErrorKind::failure, "renderer setup: " + renderer.error()});
    }
    render::OffscreenRenderer& offscreen = renderer.value();
    offscreen.context().make_current();
    auto view = render::AeroView::create();
    if (!view) {
        return core::failure(
            HeadlessError{HeadlessErrorKind::failure, "aero view setup: " + view.error()});
    }
    render::AeroView& aero_view = view.value();
    if (auto set = aero_view.set_surface(surface.value()); !set) {
        return core::failure(HeadlessError{HeadlessErrorKind::failure, "surface: " + set.error()});
    }
    std::optional<render::GridGeometry> grid;
    if (flow) {
        render::FlowField field = view_flow(*flow);
        grid = field.velocity.grid;
        if (auto set = aero_view.set_flow(std::move(field)); !set) {
            return core::failure(HeadlessError{HeadlessErrorKind::failure, "flow: " + set.error()});
        }
    }
    aero_view.set_forces(view_forces(outcome.result, outcome.flow.flow, geometry, frame));
    const AeroLayers layers;
    const render::AeroViewOptions options =
        view_options(layers, geometry.two_dimensional, bounds, grid, cp);
    render::Camera camera;
    const render::FrameInfo info{request.width, request.height};
    camera.set_mode(view_camera(bounds, geometry.two_dimensional, flow.has_value(), info.aspect(),
                                camera.lens()));
    (void)camera.update(
        0.0, render::CameraInput{},
        [](core::BodyId) -> std::optional<render::BodyFrame> { return std::nullopt; });

    std::string draw_error;
    auto image = offscreen.render(
        [&](const render::FrameInfo& target) {
            auto drawn = aero_view.draw(camera, target, options);
            if (!drawn) {
                draw_error = drawn.error();
            }
        },
        [&](const render::FrameInfo& target) { aero_view.draw_overlay(target, options); });
    if (!image) {
        return core::failure(HeadlessError{HeadlessErrorKind::failure, "render: " + image.error()});
    }
    if (!draw_error.empty()) {
        return core::failure(HeadlessError{HeadlessErrorKind::failure, "aero view: " + draw_error});
    }
    out.image = std::move(image).value();
    // The view's GL objects go with its context.
    offscreen.context().make_current();
    return out;
}

} // namespace orrery::app
