#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>

// Application bar and scene input of the space workspace (docs/app.md,
// "Window"). Like the panels of ui_panels.h: plain data in, user intent out,
// no physics access and no retained state; drawn with Dear ImGui between
// UiContext::begin_frame and UiContext::end_frame. The bar is the main menu
// bar of the viewport, so call draw_app_bar before UiContext::dockspace: the
// dock space then fills the area below it.
namespace orrery::render {

// Workspace tabs shown in the bar; V1 phase 4 has the space workspace only.
inline constexpr std::string_view kSpaceWorkspace = "Space";

struct AppBarData {
    bool paused = false;
    std::int64_t time_warp = 1;  // requested [simulated s per wall-clock s]
    double effective_warp = 0.0; // achieved over the last physics tick
    // The physics step budget is limiting the warp (shown as a warning chip
    // with the achieved rate).
    bool warp_limited = false;
    double sim_time_days = 0.0;
    // Scenario names for the scenario selector, and the loaded one (an index
    // into `scenarios`; nullopt shows `scenario_label` instead).
    std::span<const std::string> scenarios;
    std::optional<std::size_t> scenario;
    std::string_view scenario_label;
    double fps = 0.0;
    // Error or notice from the simulation (empty: none), shown in the
    // warning colour.
    std::string_view message;
};

struct AppBarIntent {
    std::optional<bool> set_paused;
    bool single_step = false;
    bool reset = false;
    std::optional<std::int64_t> set_time_warp;
    std::optional<std::size_t> load_scenario; // index into AppBarData::scenarios
};

[[nodiscard]] AppBarIntent draw_app_bar(const AppBarData& data);

// Time-warp presets of the bar: 1, 2, 5, 10, 20, 50, ... up to 1e7.
[[nodiscard]] std::span<const std::int64_t> time_warp_presets() noexcept;
// The next preset above (direction > 0) or below (direction < 0) `current`;
// `current` itself, clamped to the preset range, when there is none.
[[nodiscard]] std::int64_t step_time_warp(std::int64_t current, int direction) noexcept;
// Compact label of a warp: "1x" ... "5000x", then "2e5x" style.
[[nodiscard]] std::string format_time_warp(double warp);

// Mouse and keyboard input aimed at the scene in this frame: nothing while
// Dear ImGui wants the pointer or the keyboard (a panel is hovered, dragged
// or being typed into).
struct SceneInput {
    double drag_x = 0.0; // left-button drag this frame [px]
    double drag_y = 0.0;
    double wheel = 0.0;        // wheel notches, positive away from the user
    bool toggle_pause = false; // Space
    bool single_step = false;  // Period or Right arrow
    bool reset_view = false;   // Home or F
};

[[nodiscard]] SceneInput read_scene_input();

} // namespace orrery::render
