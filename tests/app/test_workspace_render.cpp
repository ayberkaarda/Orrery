#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <functional>
#include <iostream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "../render/gl_test_support.h"
#include "app_test_support.h"
#include "orrery/app/aero_viewport.h"
#include "orrery/app/aero_workspace.h"
#include "orrery/app/shortcut_input.h"
#include "orrery/app/shortcuts.h"
#include "orrery/app/space_experiment_panel.h"
#include "orrery/app/workspace_strip.h"
#include "orrery/render/aero_view.h"
#include "orrery/render/png.h"
#include "orrery/render/ssim.h"
#include "orrery/render/ui_app_bar.h"
#include "orrery/render/ui_context.h"
#include "orrery/render/ui_panels.h"
#include "orrery/sim/simulation.h"

// Phase 10 offscreen renders (hidden context, never a visible window):
//   - the Space workspace with the time-warp budget at work (LIMITED chip,
//     System Monitor tile and reason), the Experiment panel and the help
//     overlay, written to <build>/tests/app_output for inspection;
//   - the Aero workspace golden: bar, panels and the view of a finished
//     NACA 0012 panel-method run, against
//     tests/render/golden/aero/aero_workspace_naca0012.png (SSIM > 0.98;
//     written only with ORRERY_UPDATE_GOLDEN=1 orrery_test_app "[golden]").

using namespace orrery;
using render::test::environment_flag;

namespace {

constexpr int kWidth = 1600;
constexpr int kHeight = 900;

std::filesystem::path output(const std::string& name) {
    const std::filesystem::path dir = std::filesystem::path(ORRERY_RENDER_OUTPUT_DIR);
    std::filesystem::create_directories(dir);
    return dir / name;
}

void check_golden(const render::ImageRgba8& image, const std::string& name) {
    const std::filesystem::path actual = output(name + "_actual.png");
    REQUIRE(render::write_png(actual, image).has_value());
    const std::filesystem::path golden =
        std::filesystem::path(ORRERY_RENDER_GOLDEN_DIR) / "aero" / (name + ".png");
    if (environment_flag("ORRERY_UPDATE_GOLDEN")) {
        REQUIRE(render::write_png(golden, image).has_value());
        WARN("ORRERY_UPDATE_GOLDEN=1: wrote " << golden.string() << "; review it");
        return;
    }
    if (!std::filesystem::exists(golden)) {
        FAIL("golden reference " << golden.string()
                                 << " is missing; generate it with ORRERY_UPDATE_GOLDEN=1");
    }
    auto reference = render::read_png(golden);
    REQUIRE(reference.has_value());
    auto score = render::ssim(reference.value(), image);
    REQUIRE(score.has_value());
    std::cout << "measured: workspace golden " << name << " SSIM " << score.value().value << "\n";
    INFO("actual image: " << actual.string());
    CHECK(score.value().value > 0.98);
}

} // namespace

TEST_CASE("The Space workspace shows the warp budget, its experiments and the shortcut help",
          "[app][ui][gl][warp]") {
    render::OffscreenRenderer renderer = render::test::renderer_or_skip(kWidth, kHeight);
    auto created = render::UiContext::create(renderer.context());
    REQUIRE(created.has_value());
    render::UiContext ui = std::move(created).value();

    sim::Simulation simulation = app_test::make_simulation("sun_earth_moon.json", {});
    const auto snapshot = simulation.snapshot();
    const std::vector<std::string> scenarios{"figure_eight", "solar_system_j2000",
                                             "sun_earth_moon"};
    render::AppBarData bar;
    bar.time_warp = 10'000'000;
    bar.effective_warp = 200'000.0;
    bar.warp_limited = true;
    bar.warp_reduced = true;
    bar.step_ms = 1.95;
    bar.physics_load = 0.47;
    bar.load_limit = 0.6;
    bar.sim_time_days = 412.75;
    bar.scenarios = scenarios;
    bar.scenario = 2;
    bar.fps = 144.0;
    render::SystemMonitorData monitor;
    monitor.sim_time_days = 412.75;
    monitor.body_count = 3;
    monitor.frame_time_ms = 6.9;
    monitor.physics_step_ms = 1.95;
    monitor.integrator = "leapfrog_kdk";
    monitor.warp = render::WarpReadout{10'000'000, 200'000.0, true, 0.47, 0.6};
    const std::filesystem::path saved_dir =
        std::filesystem::temp_directory_path() / "orrery_tests" / "render_space";
    std::filesystem::remove_all(saved_dir);
    app::SpaceExperimentPanel panel(saved_dir, app_test::experiment_dir());
    panel.set_name("earth moon month");
    REQUIRE(
        panel.save(app_test::scenario_dir() / "sun_earth_moon.json", simulation.status(), 412.75)
            .has_value());
    app::SpaceRunView run;
    run.scenario = "Sun, Earth and Moon";
    run.experiment = "sun_earth_moon.json";
    run.integrator = "leapfrog_kdk";
    run.force_model = "newtonian";
    run.dt_days = 0.02;
    run.time_warp = 10'000'000;

    const auto render_frames = [&](bool help, bool experiment_tab) {
        std::optional<render::ImageRgba8> image;
        bool open = help;
        for (int frame = 0; frame < 4; ++frame) {
            auto rendered = renderer.render(
                [](const render::FrameInfo&) {},
                [&](const render::FrameInfo& f) {
                    render::UiFrameInput input;
                    input.width = f.width;
                    input.height = f.height;
                    ui.begin_frame(input);
                    (void)render::draw_app_bar(bar);
                    (void)app::draw_workspace_strip(app::Workspace::space);
                    (void)ui.dockspace();
                    (void)render::draw_body_inspector(
                        render::make_body_inspector_data(*snapshot, std::nullopt));
                    (void)render::draw_system_monitor(monitor);
                    (void)panel.draw(run);
                    (void)render::draw_render_settings(render::RenderSettings{});
                    app::focus_window(experiment_tab ? app::kSpaceExperimentWindow
                                                     : render::kSystemMonitorWindow);
                    app::draw_shortcut_help(app::default_shortcuts(), app::Workspace::space, open,
                                            "config.toml");
                    ui.end_frame();
                });
            REQUIRE(rendered.has_value());
            image = std::move(rendered).value();
        }
        REQUIRE(image.has_value());
        return std::move(*image);
    };

    const render::ImageRgba8 budget = render_frames(false, false);
    REQUIRE(render::write_png(output("space_warp_budget_actual.png"), budget).has_value());
    CHECK(ui.window_rect(render::kSystemMonitorWindow).has_value());
    CHECK_FALSE(ui.window_rect(app::kShortcutHelpWindow).has_value());

    const render::ImageRgba8 experiments = render_frames(false, true);
    REQUIRE(
        render::write_png(output("space_experiment_panel_actual.png"), experiments).has_value());
    CHECK(ui.window_rect(app::kSpaceExperimentWindow).has_value());

    const render::ImageRgba8 help = render_frames(true, false);
    REQUIRE(render::write_png(output("space_shortcut_help_actual.png"), help).has_value());
    const auto overlay = ui.window_rect(app::kShortcutHelpWindow);
    REQUIRE(overlay.has_value());
    CHECK(overlay->y + overlay->height < kHeight);
}

TEST_CASE("Golden snapshot: Aero workspace with a NACA 0012 panel-method run",
          "[app][aero][gl][golden]") {
    render::HdrSettings hdr = render::aero_hdr_settings();
    hdr.samples = 4;
    render::OffscreenRenderer renderer = render::test::renderer_or_skip(kWidth, kHeight, hdr);
    auto created = render::UiContext::create(renderer.context());
    REQUIRE(created.has_value());
    render::UiContext ui = std::move(created).value();
    renderer.context().make_current();
    auto viewport = app::AeroViewport::create();
    REQUIRE(viewport.has_value());

    // No experiments folder; the default inputs (NACA 0012 section, 4 deg,
    // panel method) run to completion on the solver thread first.
    app::AeroWorkspace workspace(app::AeroWorkspaceOptions{});
    const auto ui_frame = [&] {
        render::UiFrameInput input;
        input.width = kWidth;
        input.height = kHeight;
        input.delta_time = 1.0 / 60.0;
        input.pointer = render::UiPointer{};
        ui.begin_frame(input);
        workspace.update(1.0 / 60.0);
        workspace.draw_bar(60.0);
        (void)app::draw_workspace_strip(app::Workspace::aero);
        return workspace.draw_panels();
    };
    const auto settle = [&](const std::function<bool()>& done) {
        return app_test::wait_until([&] {
            (void)ui_frame();
            ui.end_frame();
            return done();
        });
    };
    REQUIRE(settle([&] { return workspace.geometry_ready(); }));
    REQUIRE(workspace.start_run());
    REQUIRE(
        settle([&] { return workspace.progress().status.state == sim::AeroRunState::finished; }));

    std::optional<render::ImageRgba8> image;
    std::string draw_error;
    for (int frame = 0; frame < 4; ++frame) {
        const render::UiRect scene = ui_frame();
        const render::Camera& camera = workspace.camera(scene, kWidth, kHeight);
        const app::AeroViewState state = workspace.view_state();
        auto rendered = renderer.render(
            [&](const render::FrameInfo& target) {
                auto drawn = viewport.value().draw(state, camera, target, 1.0 / 60.0);
                if (!drawn) {
                    draw_error = drawn.error();
                }
            },
            [&](const render::FrameInfo& target) {
                viewport.value().draw_overlay(target);
                ui.end_frame();
            });
        REQUIRE(rendered.has_value());
        REQUIRE(draw_error.empty());
        CHECK(viewport.value().problem().empty());
        image = std::move(rendered).value();
    }
    REQUIRE(image.has_value());
    const auto shown = workspace.shown_result();
    REQUIRE(shown.has_value());
    std::cout << "measured: aero workspace NACA 0012 at 4 deg: cl " << shown->cl << ", cd "
              << shown->cd << "\n";
    CHECK(shown->cl > 0.3);
    check_golden(*image, "aero_workspace_naca0012");
    renderer.context().make_current();
}
