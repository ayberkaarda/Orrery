#include "orrery/app/entry_headless.h"

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "orrery/app/entry_viewport.h"
#include "orrery/app/entry_workspace.h"
#include "orrery/app/workspace_strip.h"
#include "orrery/render/gl_context.h"
#include "orrery/render/png.h"
#include "orrery/render/snapshot.h"
#include "orrery/render/ui_context.h"

namespace orrery::app {

namespace {

// Frames drawn before the image: the dock layout and window sizes settle
// over the first frames.
constexpr int kSettleFrames = 3;
constexpr double kFrameSeconds = 1.0 / 60.0;
constexpr double kShownFps = 60.0;

HeadlessError failure(std::string message) {
    return HeadlessError{HeadlessErrorKind::failure, std::move(message)};
}

} // namespace

core::Result<EntryHeadlessFrame, HeadlessError>
render_entry_headless(const sim::EntrySetup& setup, const EntryHeadlessRequest& request) {
    auto run = sim::run_entry(setup);
    if (!run) {
        return core::failure(failure("entry: " + run.error()));
    }
    EntryHeadlessFrame out;
    out.run = std::move(run).value();

    auto context = render::GlContext::create(render::OffscreenRenderer::context_options());
    if (!context) {
        return core::failure(HeadlessError{HeadlessErrorKind::no_context, context.error()});
    }
    render::HdrSettings hdr = render::entry_hdr_settings();
    hdr.samples = request.msaa_samples;
    auto renderer = render::OffscreenRenderer::create(std::move(context).value(), request.width,
                                                      request.height, hdr);
    if (!renderer) {
        return core::failure(failure("renderer setup: " + renderer.error()));
    }
    render::OffscreenRenderer& offscreen = renderer.value();
    offscreen.context().make_current();
    auto ui_created = render::UiContext::create(offscreen.context());
    if (!ui_created) {
        return core::failure(failure("user interface: " + ui_created.error()));
    }
    render::UiContext ui = std::move(ui_created).value();
    auto viewport = EntryViewport::create();
    if (!viewport) {
        return core::failure(failure("entry view setup: " + viewport.error()));
    }

    // No experiments folder: the panels show only this run.
    EntryWorkspace workspace(EntryWorkspaceOptions{});
    sim::StoredEntry shown;
    shown.setup = setup;
    shown.result = std::make_shared<const sim::EntryTrace>(out.run);
    workspace.present(std::move(shown));

    std::string draw_error;
    std::optional<render::ImageRgba8> image;
    for (int frame = 0; frame <= kSettleFrames; ++frame) {
        render::UiFrameInput input;
        input.width = request.width;
        input.height = request.height;
        input.delta_time = kFrameSeconds;
        input.pointer = render::UiPointer{};
        ui.begin_frame(input);
        workspace.update(kFrameSeconds);
        workspace.draw_bar(kShownFps);
        (void)draw_workspace_strip(Workspace::entry);
        const render::UiRect scene = workspace.draw_panels();
        const render::Camera& camera = workspace.camera(scene, request.width, request.height);
        const EntryViewState state = workspace.view_state();
        auto rendered = offscreen.render(
            [&](const render::FrameInfo& target) {
                auto drawn = viewport.value().draw(state, camera, target);
                if (!drawn) {
                    draw_error = drawn.error();
                }
            },
            [&](const render::FrameInfo&) { ui.end_frame(); });
        if (!rendered) {
            return core::failure(failure("render: " + rendered.error()));
        }
        if (!draw_error.empty()) {
            return core::failure(failure("entry view: " + draw_error));
        }
        if (const std::string problem = viewport.value().problem(); !problem.empty()) {
            return core::failure(failure("entry view: " + problem));
        }
        image = std::move(rendered).value();
    }
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access): set by the last frame.
    out.image = std::move(*image);
    offscreen.context().make_current();
    return out;
}

int run_entry_headless_command(const CliOptions& options, const std::filesystem::path& path,
                               const scene::Experiment& generic, std::ostream& out,
                               std::ostream& err) {
    const auto entry = sim::entry_from_experiment(generic);
    if (!entry) {
        err << "orrery_app: experiment " << path.string() << ": " << entry.error() << '\n';
        return kExitFailure;
    }
    if (options.benchmark_frames > 0) {
        err << "orrery_app: --benchmark times the space view; it is not available for entry "
               "experiments\n";
        return kExitFailure;
    }
    EntryHeadlessRequest request;
    request.width = options.width.value_or(request.width);
    request.height = options.height.value_or(request.height);
    const auto frame = render_entry_headless(entry.value().setup, request);
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
    const sim::EntrySummary& s = frame.value().run.summary;
    out << "wrote " << options.out.string() << " (" << request.width << 'x' << request.height
        << "): " << entry.value().setup.name << ", "
        << sim::planet_constants(entry.value().setup.planet).name << ": "
        << sim::to_string(s.outcome) << " after " << s.duration_s << " s, peak " << s.peak_g
        << " g, peak heat flux " << s.peak_heat_flux_w_m2 << " W/m2, downrange " << s.downrange_m
        << " m\n";
    return kExitOk;
}

} // namespace orrery::app
