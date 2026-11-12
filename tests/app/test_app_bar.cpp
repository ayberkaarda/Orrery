#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "../render/gl_test_support.h"
#include "orrery/render/png.h"
#include "orrery/render/ui_app_bar.h"
#include "orrery/render/ui_context.h"
#include "orrery/render/ui_panels.h"

using namespace orrery::render;

TEST_CASE("Time-warp presets step through 1-2-5 decades", "[app][ui]") {
    const auto presets = time_warp_presets();
    REQUIRE(presets.size() == 22);
    CHECK(presets.front() == 1);
    CHECK(presets.back() == 10'000'000);
    CHECK(step_time_warp(1, +1) == 2);
    CHECK(step_time_warp(2, +1) == 5);
    CHECK(step_time_warp(5, +1) == 10);
    CHECK(step_time_warp(1'000'000, -1) == 500'000);
    CHECK(step_time_warp(10'000'000, +1) == 10'000'000);
    CHECK(step_time_warp(1, -1) == 1);
    // Off-preset values move to the neighbouring preset.
    CHECK(step_time_warp(300, +1) == 500);
    CHECK(step_time_warp(300, -1) == 200);
    CHECK(step_time_warp(0, 0) == 1);
}

TEST_CASE("Time-warp labels are compact", "[app][ui]") {
    const std::string times = "\xC3\x97";
    CHECK(format_time_warp(1.0) == "1" + times);
    CHECK(format_time_warp(5000.0) == "5000" + times);
    CHECK(format_time_warp(20'000.0) == "2e4" + times);
    CHECK(format_time_warp(1'000'000.0) == "1e6" + times);
    CHECK(format_time_warp(2'590'000.0) == "2.6e6" + times);
    CHECK(format_time_warp(9'990'000.0) == "1e7" + times);
    CHECK(format_time_warp(-1.0) == "-" + times);
}

TEST_CASE("The app bar sits above the dock space and reports no intent at rest", "[app][ui]") {
    auto created = UiContext::create_headless();
    REQUIRE(created.has_value());
    UiContext ui = std::move(created).value();
    const std::vector<std::string> scenarios{"figure_eight", "solar_system_j2000", "sun_earth"};
    AppBarData data;
    data.paused = false;
    data.time_warp = 1'000'000;
    data.effective_warp = 250'000.0;
    data.warp_limited = true;
    data.sim_time_days = 1234.5;
    data.scenarios = scenarios;
    data.scenario = 1;
    data.fps = 60.0;
    data.message = "Cannot load scenario x.json: io_error";

    UiRect scene{};
    AppBarIntent intent;
    UiFrameInput input;
    input.width = 1600;
    input.height = 900;
    // A few frames: the dock layout settles under the bar.
    for (int frame = 0; frame < 3; ++frame) {
        ui.begin_frame(input);
        intent = draw_app_bar(data);
        scene = ui.dockspace();
        (void)draw_render_settings(RenderSettings{});
        const SceneInput scene_input = read_scene_input();
        CHECK(scene_input.drag_x == 0.0);
        CHECK_FALSE(scene_input.toggle_pause);
        ui.end_frame();
    }
    CHECK_FALSE(intent.set_paused.has_value());
    CHECK_FALSE(intent.single_step);
    CHECK_FALSE(intent.reset);
    CHECK_FALSE(intent.set_time_warp.has_value());
    CHECK_FALSE(intent.load_scenario.has_value());
    CHECK(ui.last_vertex_count() > 0);
    // The scene area starts below the bar.
    CHECK(scene.y > 20.0);
    CHECK(scene.y < 60.0);
    const auto settings = ui.window_rect(kRenderSettingsWindow);
    REQUIRE(settings.has_value());
    CHECK(settings->y >= scene.y - 0.5);
}

TEST_CASE("The space workspace renders with the app bar offscreen", "[app][ui][gl]") {
    // The window's frame composition (scene, panels, bar) through the
    // offscreen pipeline; the image is written for inspection.
    OffscreenRenderer renderer = orrery::render::test::renderer_or_skip(1280, 720);
    auto created = UiContext::create(renderer.context());
    REQUIRE(created.has_value());
    UiContext ui = std::move(created).value();
    const std::vector<std::string> scenarios{"figure_eight", "solar_system_j2000"};
    AppBarData data;
    data.time_warp = 2'000'000;
    data.effective_warp = 640'000.0;
    data.warp_limited = true;
    data.sim_time_days = 412.75;
    data.scenarios = scenarios;
    data.scenario = 1;
    data.fps = 144.0;
    SystemMonitorData monitor;
    monitor.integrator = "leapfrog_kdk / newtonian";
    BodyInspectorData inspector;
    UiRect scene{};
    std::optional<ImageRgba8> image;
    for (int frame = 0; frame < 3; ++frame) {
        auto rendered = renderer.render([](const FrameInfo&) {},
                                        [&](const FrameInfo& f) {
                                            UiFrameInput input;
                                            input.width = f.width;
                                            input.height = f.height;
                                            ui.begin_frame(input);
                                            (void)draw_app_bar(data);
                                            scene = ui.dockspace();
                                            (void)draw_body_inspector(inspector);
                                            (void)draw_system_monitor(monitor);
                                            (void)draw_render_settings(RenderSettings{});
                                            ui.end_frame();
                                        });
        REQUIRE(rendered.has_value());
        image = std::move(rendered).value();
    }
    REQUIRE(image.has_value());
    const std::filesystem::path out =
        std::filesystem::path(ORRERY_RENDER_OUTPUT_DIR) / "app_workspace_actual.png";
    std::filesystem::create_directories(out.parent_path());
    REQUIRE(write_png(out, *image).has_value());
    // The bar is drawn across the top (not the black scene background).
    const std::size_t top = image->offset(640, 8);
    CHECK(int{image->pixels[top]} + image->pixels[top + 1] + image->pixels[top + 2] > 15);
    CHECK(scene.y > 20.0);
}

TEST_CASE("App bar buttons turn clicks into intents", "[app][ui]") {
    auto created = UiContext::create_headless();
    REQUIRE(created.has_value());
    UiContext ui = std::move(created).value();
    AppBarData data;
    data.paused = true;
    data.time_warp = 1000;

    // The play button sits after the word mark, the workspace tab and the
    // reset button; find it by scanning the bar for the click that pauses
    // or resumes.
    UiFrameInput input;
    input.width = 1280;
    input.height = 720;
    ui.begin_frame(input);
    (void)draw_app_bar(data);
    ui.end_frame();
    bool resumed = false;
    for (double x = 4.0; x < 400.0 && !resumed; x += 4.0) {
        // Hover, press, release: one input change per frame.
        for (const bool down : {false, true, false}) {
            UiPointer pointer;
            pointer.x = x;
            pointer.y = 14.0;
            pointer.buttons[0] = down;
            input.pointer = pointer;
            ui.begin_frame(input);
            const AppBarIntent intent = draw_app_bar(data);
            ui.end_frame();
            if (intent.set_paused == std::optional<bool>(false)) {
                resumed = true;
            }
        }
    }
    CHECK(resumed);
}
