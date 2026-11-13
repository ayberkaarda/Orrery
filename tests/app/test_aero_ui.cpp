#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "app_test_support.h"
#include "orrery/app/aero_workspace.h"
#include "orrery/app/workspace_strip.h"
#include "orrery/render/ui_app_bar.h"
#include "orrery/render/ui_context.h"
#include "orrery/sim/aero_comparison.h"

// Phase 8b: the Aero workspace's user interface under a headless UiContext
// (no window, no OpenGL): layout, background geometry, a run through the
// solver thread, the error modal, saving and the workspace strip.

using namespace orrery;

namespace {

std::filesystem::path scratch(const char* name) {
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() / "orrery_test_aero_ui" / name;
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}

struct Harness {
    explicit Harness(const std::filesystem::path& dir) : ui(make_ui()), workspace(options(dir)) {}

    static render::UiContext make_ui() {
        auto created = render::UiContext::create_headless();
        REQUIRE(created.has_value());
        return std::move(created).value();
    }

    static app::AeroWorkspaceOptions options(const std::filesystem::path& dir) {
        app::AeroWorkspaceOptions o;
        o.experiment_dir = dir;
        o.steps_per_frame = 50;
        return o;
    }

    // One frame of the window loop for the Aero workspace.
    std::optional<app::Workspace> frame() {
        render::UiFrameInput input;
        input.width = 1600;
        input.height = 900;
        ui.begin_frame(input);
        workspace.update(1.0 / 60.0);
        workspace.draw_bar(60.0);
        const auto picked = app::draw_workspace_strip(app::Workspace::aero);
        scene = workspace.draw_panels();
        workspace.handle_scene_input(render::read_scene_input(), 1.0 / 60.0);
        ui.end_frame();
        (void)workspace.camera(scene, 1600, 900);
        return picked;
    }

    bool frames_until(const std::function<bool()>& done) {
        return app_test::wait_until([&] {
            (void)frame();
            return done();
        });
    }

    render::UiContext ui;
    app::AeroWorkspace workspace;
    render::UiRect scene;
};

} // namespace

TEST_CASE("Aero workspace lays out its panels around the scene", "[app][ui][aero]") {
    Harness h(scratch("layout"));
    for (int i = 0; i < 4; ++i) {
        CHECK_FALSE(h.frame().has_value()); // no clicks: no workspace switch
    }
    for (const std::string_view name :
         {app::kGeometryWindow, app::kFlowWindow, app::kResultsWindow, app::kExperimentsWindow}) {
        INFO(name);
        CHECK(h.ui.window_rect(name).has_value());
        CHECK(h.ui.window_docked(name));
    }
    // The scene keeps the middle of the window.
    CHECK(h.scene.width > 400.0);
    CHECK(h.scene.height > 300.0);
    CHECK(h.scene.x > 200.0);
    const auto geometry = h.ui.window_rect(app::kGeometryWindow);
    const auto results = h.ui.window_rect(app::kResultsWindow);
    REQUIRE(geometry.has_value());
    REQUIRE(results.has_value());
    CHECK(geometry->x < h.scene.x);
    CHECK(results->x >= h.scene.x + h.scene.width - 1.0);
}

TEST_CASE("Aero workspace builds its geometry off the UI thread and runs a solve",
          "[app][ui][aero]") {
    Harness h(scratch("run"));
    REQUIRE(h.frames_until([&] { return h.workspace.geometry_ready(); }));
    const app::AeroViewState before = h.workspace.view_state();
    REQUIRE(before.geometry != nullptr);
    CHECK(before.geometry->two_dimensional); // the default NACA 0012 section
    CHECK(before.result == nullptr);

    REQUIRE(h.workspace.start_run());
    REQUIRE(h.frames_until(
        [&] { return h.workspace.progress().status.state == sim::AeroRunState::finished; }));
    const app::AeroViewState after = h.workspace.view_state();
    REQUIRE(after.result != nullptr);
    CHECK(after.result->cl > 0.3); // 4 degrees
    REQUIRE(after.surface_cp != nullptr);
    CHECK(after.surface_cp->size() == after.geometry->section.size());
    // The camera frames the section from above (2D view).
    const render::Camera& camera = h.workspace.camera(h.scene, 1600, 900);
    CHECK(camera.mode_kind() == render::CameraModeKind::turntable);
}

TEST_CASE("Aero workspace shows geometry errors in its modal", "[app][ui][aero]") {
    Harness h(scratch("modal"));
    scene::AeroExperiment broken;
    broken.geometry.source = scene::GeometrySource::file;
    broken.geometry.path = "missing_model.stl";
    h.workspace.open(broken, scratch("modal_base"));
    REQUIRE(h.frames_until([&] { return !h.workspace.busy(); }));
    (void)h.frame();
    (void)h.frame();
    const auto modal = h.ui.window_rect(app::kErrorModal);
    REQUIRE(modal.has_value()); // open, centred over the window
    CHECK(modal->x > 300.0);
    CHECK(h.workspace.view_state().geometry == nullptr);
}

namespace {

// Saves the workspace's experiment and reports whether the file carries a result.
bool saved_with_result(app::AeroWorkspace& workspace) {
    const std::optional<std::filesystem::path> path = workspace.save();
    REQUIRE(path.has_value());
    auto loaded = sim::load_aero_experiment_file(path.value_or(std::filesystem::path{}));
    REQUIRE(loaded.has_value());
    return loaded.value().result.has_value();
}

} // namespace

TEST_CASE("A failed geometry replacement leaves nothing to run or save", "[app][ui][aero]") {
    Harness h(scratch("failed_geometry"));
    REQUIRE(h.frames_until([&] { return h.workspace.geometry_ready(); }));
    REQUIRE(h.workspace.start_run());
    REQUIRE(h.frames_until(
        [&] { return h.workspace.progress().status.state == sim::AeroRunState::finished; }));
    // Replace the solved section by a model that cannot be loaded.
    scene::AeroExperiment broken = h.workspace.experiment();
    broken.geometry = scene::GeometryRef{};
    broken.geometry.source = scene::GeometrySource::file;
    broken.geometry.path = "missing_model.stl";
    h.workspace.open(broken, scratch("failed_geometry_base"));
    REQUIRE(h.frames_until([&] { return !h.workspace.busy(); }));
    (void)h.frame();
    CHECK_FALSE(h.workspace.geometry_ready());
    CHECK(h.workspace.view_state().geometry == nullptr);
    CHECK_FALSE(h.workspace.start_run());
    CHECK_FALSE(saved_with_result(h.workspace));
}

TEST_CASE("Aero experiments opened in the workspace keep their record", "[app][ui][aero]") {
    const std::filesystem::path dir = scratch("open");
    Harness h(dir);
    auto loaded =
        sim::load_aero_experiment_file(app_test::experiment_dir() / "aero_naca2412_wing.json");
    REQUIRE(loaded.has_value());
    h.workspace.open(loaded.value(), app_test::experiment_dir());
    REQUIRE(h.frames_until([&] {
        const app::AeroViewState state = h.workspace.view_state();
        return state.geometry != nullptr && !state.geometry->two_dimensional;
    }));
    CHECK(h.workspace.experiment().geometry.naca == "2412");
    CHECK(h.workspace.experiment().conditions.angle_of_attack_deg == 5.0);
    CHECK_FALSE(h.workspace.view_state().geometry->trailing_edge.empty());
}
