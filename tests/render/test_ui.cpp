#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "orrery/core/sim_snapshot.h"
#include "orrery/render/render_settings.h"
#include "orrery/render/ui_context.h"
#include "orrery/render/ui_panels.h"
#include "orrery/render/ui_theme.h"
#include "space_test_support.h"

using namespace orrery;
using namespace orrery::render;
using Catch::Approx;

namespace {

UiColor opaque(const UiColor& c) {
    return {c.r, c.g, c.b, 1.0F};
}

bool overlaps(const UiRect& a, const UiRect& b) {
    return a.x < b.x + b.width - 0.5 && b.x < a.x + a.width - 0.5 && a.y < b.y + b.height - 0.5 &&
           b.y < a.y + a.height - 0.5;
}

struct PanelFrame {
    BodyInspectorIntent inspector;
    SystemMonitorIntent monitor;
    RenderSettingsIntent settings;
    UiRect scene{};
};

PanelFrame draw_panels(UiContext& ui, const UiFrameInput& input, const BodyInspectorData& inspector,
                       const SystemMonitorData& monitor, const RenderSettings& settings) {
    PanelFrame frame;
    ui.begin_frame(input);
    frame.scene = ui.dockspace();
    frame.inspector = draw_body_inspector(inspector);
    frame.monitor = draw_system_monitor(monitor);
    frame.settings = draw_render_settings(settings);
    ui.end_frame();
    return frame;
}

} // namespace

TEST_CASE("The UI theme is readable and does not use the stock ImGui blue", "[render][ui]") {
    const UiTheme t = default_theme();
    const UiColor window = opaque(t.window);
    // WCAG: body text >= 7:1 (AAA), muted labels >= 4.5:1 (AA), disabled >= 2.5:1.
    CHECK(contrast_ratio(t.text, window) >= 7.0);
    CHECK(contrast_ratio(t.text_muted, window) >= 4.5);
    CHECK(contrast_ratio(t.text_disabled, window) >= 2.5);
    CHECK(contrast_ratio(t.text, t.control) >= 7.0);
    CHECK(contrast_ratio(t.text_muted, t.control) >= 4.5);
    CHECK(contrast_ratio(t.accent, window) >= 4.5);
    // A slider's value is drawn over its grab: the text stays readable there
    // (>= 4.5:1), and the grab still stands out from the track.
    const UiColor grab = composite(t.slider_grab, t.control);
    CHECK(contrast_ratio(t.text, grab) >= 4.5);
    CHECK(contrast_ratio(grab, t.control) >= 2.5);
    CHECK(grab.r > grab.b); // still reads as the accent
    // Surfaces stay dark (the scene is space) and ordered.
    CHECK(relative_luminance(window) < 0.02);
    CHECK(relative_luminance(t.title) <= relative_luminance(window));
    CHECK(relative_luminance(t.control) > relative_luminance(window));
    CHECK(relative_luminance(t.control_hover) > relative_luminance(t.control));
    CHECK(relative_luminance(t.control_active) > relative_luminance(t.control_hover));
    // The accent is warm: red over blue, unlike the default (0.26, 0.59, 0.98).
    CHECK(t.accent.r > t.accent.b);
    CHECK_FALSE(t.accent == UiColor{0.26F, 0.59F, 0.98F, 1.0F});
    // Helpers.
    CHECK(contrast_ratio({1.0F, 1.0F, 1.0F, 1.0F}, {0.0F, 0.0F, 0.0F, 1.0F}) == Approx(21.0));
    const UiColor half = composite({1.0F, 1.0F, 1.0F, 0.5F}, {0.0F, 0.0F, 0.0F, 1.0F});
    CHECK(half.r == Approx(0.5F));
}

TEST_CASE("Inspector data reports the J2000 state and osculating orbits", "[render][ui]") {
    const auto snapshot = core::SimSnapshot::capture(test::load_j2000_state());
    const auto find = [&](const char* name) {
        for (std::size_t i = 0; i < snapshot->size(); ++i) {
            if (snapshot->names()[i] == name) {
                return snapshot->ids()[i];
            }
        }
        FAIL("no body " << name);
        return core::BodyId{};
    };

    const BodyInspectorData none = make_body_inspector_data(*snapshot, std::nullopt);
    CHECK(none.bodies.size() == snapshot->size());
    CHECK_FALSE(none.selected.has_value());
    CHECK_FALSE(make_body_inspector_data(*snapshot, core::BodyId{999}).selected.has_value());

    const BodyInspectorData earth = make_body_inspector_data(*snapshot, find("Earth"));
    REQUIRE(earth.selected.has_value());
    const BodyDetails& e = *earth.selected;
    CHECK(e.name == "Earth");
    CHECK(e.kind == core::BodyKindTag::planet);
    CHECK(e.radius_km == Approx(6371.0));
    CHECK(e.radius_is_nominal);
    CHECK(e.distance_au == Approx(0.98194).epsilon(1e-4));
    CHECK(e.speed_km_s == Approx(30.29).epsilon(2e-3));
    CHECK(e.mass_kg == Approx(5.972e24).epsilon(1e-3));
    REQUIRE(e.orbit.has_value());
    CHECK(e.orbit->primary == "Sun");
    // Earth-Moon barycentre offsets make the osculating Earth orbit differ a
    // little from the textbook values (a = 1.0000 AU, e = 0.0167).
    CHECK(e.orbit->semi_major_axis_au == Approx(1.0).epsilon(0.02));
    CHECK(e.orbit->eccentricity < 0.05);
    CHECK(e.orbit->inclination_deg < 0.1);
    REQUIRE(e.orbit->period_days.has_value());
    CHECK(*e.orbit->period_days == Approx(365.25).epsilon(0.03));

    const BodyInspectorData moon = make_body_inspector_data(*snapshot, find("Moon"));
    REQUIRE(moon.selected.has_value());
    REQUIRE(moon.selected->orbit.has_value());
    CHECK(moon.selected->orbit->primary == "Earth");
    CHECK(moon.selected->orbit->semi_major_axis_au * 149597870.7 == Approx(384400.0).epsilon(0.03));
    CHECK(*moon.selected->orbit->period_days == Approx(27.32).epsilon(0.03));

    const BodyInspectorData sun = make_body_inspector_data(*snapshot, find("Sun"));
    REQUIRE(sun.selected.has_value());
    CHECK_FALSE(sun.selected->orbit.has_value());
}

TEST_CASE("Scene labels put the selected body first and skip crowded ones", "[render][ui]") {
    const auto snapshot = core::SimSnapshot::capture(test::load_j2000_state());
    std::vector<BodyScreenInfo> screen;
    for (std::size_t i = 0; i < snapshot->size(); ++i) {
        BodyScreenInfo info;
        info.id = snapshot->ids()[i];
        info.index = i;
        info.visible = i != 9; // Neptune off screen
        // Sun, Mercury and Venus within a few pixels; the others spread out.
        info.pixel = i < 3 ? glm::dvec2(100.0 + 3.0 * static_cast<double>(i), 100.0)
                           : glm::dvec2(100.0 + 80.0 * static_cast<double>(i), 300.0);
        screen.push_back(info);
    }
    const core::BodyId venus = snapshot->ids()[2];
    const std::vector<SceneLabel> labels = make_scene_labels(*snapshot, screen, venus);
    REQUIRE(labels.size() == 7); // Venus, then 6 of Earth .. Uranus (Sun, Mercury crowded out)
    CHECK(labels[0].name == "Venus");
    CHECK(labels[0].selected);
    // Then by decreasing mass.
    CHECK(labels[1].name == "Jupiter system");
    CHECK(labels[2].name == "Saturn system");
    for (const SceneLabel& label : labels) {
        CHECK(label.name != "Sun");
        CHECK(label.name != "Neptune system");
    }
    const std::vector<SceneLabel> unselected = make_scene_labels(*snapshot, screen, std::nullopt);
    REQUIRE_FALSE(unselected.empty());
    CHECK(unselected[0].name == "Sun");
    CHECK_FALSE(unselected[0].selected);
}

TEST_CASE("Headless UI builds the docked panel layout without a GPU", "[render][ui]") {
    auto created = UiContext::create_headless();
    REQUIRE(created.has_value());
    UiContext ui = std::move(created).value();
    CHECK_FALSE(ui.has_renderer());

    const auto snapshot = core::SimSnapshot::capture(test::load_j2000_state());
    const BodyInspectorData inspector = make_body_inspector_data(*snapshot, snapshot->ids()[3]);
    const std::vector<double> t{0.0, 1.0, 2.0, 3.0};
    const std::vector<double> drift{0.0, 1.0e-9, -2.0e-9, 1.5e-9};
    SystemMonitorData monitor;
    monitor.sim_time_days = 0.0;
    monitor.body_count = snapshot->size();
    monitor.frame_time_ms = 4.2;
    monitor.integrator = "leapfrog_kdk";
    monitor.series_time_days = t;
    monitor.series_values = drift;
    const RenderSettings settings;

    UiFrameInput input;
    input.width = 1280;
    input.height = 720;
    PanelFrame frame;
    for (int i = 0; i < 3; ++i) {
        frame = draw_panels(ui, input, inspector, monitor, settings);
    }
    // No input: no intents.
    CHECK_FALSE(frame.inspector.select.has_value());
    CHECK_FALSE(frame.inspector.focus_camera);
    CHECK_FALSE(frame.monitor.clear_series);
    CHECK_FALSE(frame.settings.changed.has_value());
    CHECK(ui.last_vertex_count() > 1000);

    // Layout: three docked panels around a free central scene area.
    const auto left = ui.window_rect(kBodyInspectorWindow);
    const auto right = ui.window_rect(kRenderSettingsWindow);
    const auto bottom = ui.window_rect(kSystemMonitorWindow);
    REQUIRE(left.has_value());
    REQUIRE(right.has_value());
    REQUIRE(bottom.has_value());
    CHECK(ui.window_docked(kBodyInspectorWindow));
    CHECK(ui.window_docked(kRenderSettingsWindow));
    CHECK(ui.window_docked(kSystemMonitorWindow));
    CHECK(left->x == Approx(0.0).margin(1.0));
    CHECK(right->x + right->width == Approx(1280.0).margin(1.0));
    CHECK(bottom->y + bottom->height == Approx(720.0).margin(1.0));
    CHECK(bottom->x >= left->x + left->width - 1.0);
    CHECK_FALSE(overlaps(*left, *right));
    CHECK_FALSE(overlaps(*left, *bottom));
    CHECK_FALSE(overlaps(*right, *bottom));
    CHECK(frame.scene.width > 1280.0 * 0.4);
    CHECK(frame.scene.height > 720.0 * 0.6);
    CHECK_FALSE(overlaps(frame.scene, *left));
    CHECK_FALSE(overlaps(frame.scene, *bottom));

    // The pointer over a panel belongs to the UI; over the scene it does not.
    UiPointer pointer;
    pointer.x = left->x + left->width * 0.5;
    pointer.y = left->y + left->height * 0.5;
    input.pointer = pointer;
    draw_panels(ui, input, inspector, monitor, settings);
    draw_panels(ui, input, inspector, monitor, settings);
    CHECK(ui.wants_pointer());
    pointer.x = frame.scene.x + frame.scene.width * 0.5;
    pointer.y = frame.scene.y + frame.scene.height * 0.5;
    input.pointer = pointer;
    draw_panels(ui, input, inspector, monitor, settings);
    draw_panels(ui, input, inspector, monitor, settings);
    CHECK_FALSE(ui.wants_pointer());
}

TEST_CASE("Clicking a body in the inspector list selects it", "[render][ui]") {
    auto created = UiContext::create_headless();
    REQUIRE(created.has_value());
    UiContext ui = std::move(created).value();
    const auto snapshot = core::SimSnapshot::capture(test::load_j2000_state());
    const BodyInspectorData inspector = make_body_inspector_data(*snapshot, std::nullopt);
    UiFrameInput input;
    input.width = 1280;
    input.height = 720;
    for (int i = 0; i < 2; ++i) {
        draw_panels(ui, input, inspector, {}, {});
    }
    const auto left = ui.window_rect(kBodyInspectorWindow);
    REQUIRE(left.has_value());
    // Scan down the list column until a row reacts to a click: the list
    // starts under the tab bar and the BODIES caption.
    std::optional<core::BodyId> picked;
    double y = left->y + 30.0;
    while (y < left->y + 220.0 && !picked) {
        UiPointer pointer;
        pointer.x = left->x + 60.0;
        pointer.y = y;
        input.pointer = pointer;
        draw_panels(ui, input, inspector, {}, {}); // hover
        pointer.buttons[0] = true;
        input.pointer = pointer;
        draw_panels(ui, input, inspector, {}, {}); // press
        pointer.buttons[0] = false;
        input.pointer = pointer;
        picked = draw_panels(ui, input, inspector, {}, {}).inspector.select; // release
        y += 6.0;
    }
    REQUIRE(picked.has_value());
    // The first row reached is the first body of the list.
    CHECK(*picked == snapshot->ids()[0]);
}
