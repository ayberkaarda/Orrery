#include "orrery/render/ui_app_bar.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <system_error>

#include <imgui.h>

#include "orrery/render/ui_theme.h"

namespace orrery::render {

namespace {

constexpr std::array<std::int64_t, 22> kWarpPresets{
    1,       2,       5,         10,        20,        50,        100,    200,
    500,     1'000,   2'000,     5'000,     10'000,    20'000,    50'000, 100'000,
    200'000, 500'000, 1'000'000, 2'000'000, 5'000'000, 10'000'000};

constexpr double kSecondsPerDay = 86400.0;
// Multiplication sign (U+00D7) in UTF-8; the default font covers Latin-1.
constexpr const char* kTimes = "\xC3\x97";

std::string fixed(double value, int precision) {
    std::array<char, 64> buffer{};
    const auto result = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value,
                                      std::chars_format::fixed, precision);
    return result.ec == std::errc{} ? std::string(buffer.data(), result.ptr) : std::string("-");
}

ImVec4 im(const UiColor& c, float alpha) {
    return {c.r, c.g, c.b, alpha};
}

void muted(const char* text) {
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::TextUnformatted(text);
    ImGui::PopStyleColor();
}

void item_tooltip(const char* text) {
    if (ImGui::BeginItemTooltip()) {
        ImGui::TextUnformatted(text);
        ImGui::EndTooltip();
    }
}

// Small orbit mark in front of the word mark: a ring, the sun and a planet.
void word_mark() {
    const float h = ImGui::GetFrameHeight();
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    ImGui::Dummy(ImVec2(h, h));
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImVec2 centre(pos.x + h * 0.5F, pos.y + h * 0.5F);
    const float r = h * 0.34F;
    draw->AddCircle(centre, r, ImGui::GetColorU32(ImGuiCol_TextDisabled), 32, 1.2F);
    draw->AddCircleFilled(centre, h * 0.11F, ImGui::GetColorU32(ImGuiCol_CheckMark), 16);
    draw->AddCircleFilled(ImVec2(centre.x + r * 0.766F, centre.y - r * 0.643F), h * 0.075F,
                          ImGui::GetColorU32(ImGuiCol_Text), 12);
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_CheckMark));
    ImGui::TextUnformatted("ORRERY");
    ImGui::PopStyleColor();
}

// A workspace tab: the selected one sits on the panel colour with an accent
// underline, like the docked panel tabs below it.
void workspace_tab(std::string_view name, bool selected) {
    const ImGuiStyle& style = ImGui::GetStyle();
    const std::string label(name);
    const ImVec2 text_size = ImGui::CalcTextSize(label.c_str());
    const float h = ImGui::GetFrameHeight();
    const float w = text_size.x + style.FramePadding.x * 3.0F;
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    ImGui::PushID(label.c_str());
    ImGui::InvisibleButton("##workspace", ImVec2(w, h));
    ImGui::PopID();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImVec2 end(pos.x + w, pos.y + h);
    if (selected) {
        draw->AddRectFilled(pos, end, ImGui::GetColorU32(ImGuiCol_TabSelected), style.TabRounding);
        draw->AddRectFilled(ImVec2(pos.x + 4.0F, end.y - 2.0F), ImVec2(end.x - 4.0F, end.y),
                            ImGui::GetColorU32(ImGuiCol_TabSelectedOverline));
    } else if (ImGui::IsItemHovered()) {
        draw->AddRectFilled(pos, end, ImGui::GetColorU32(ImGuiCol_TabHovered), style.TabRounding);
    }
    draw->AddText(ImVec2(pos.x + (w - text_size.x) * 0.5F, pos.y + (h - text_size.y) * 0.5F),
                  ImGui::GetColorU32(selected ? ImGuiCol_Text : ImGuiCol_TextDisabled),
                  label.c_str());
    item_tooltip(selected ? "Space workspace: orbital dynamics" : label.c_str());
}

enum class Glyph : std::uint8_t { play, pause, step, reset };

// Square-ish button with a vector glyph (the default font has no icons).
bool icon_button(const char* id, Glyph glyph, const char* tooltip, bool emphasised) {
    const ImGuiStyle& style = ImGui::GetStyle();
    const float h = ImGui::GetFrameHeight();
    const ImVec2 size(h * 1.3F, h);
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    const bool pressed = ImGui::InvisibleButton(id, size);
    const bool hovered = ImGui::IsItemHovered();
    const bool active = ImGui::IsItemActive();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    ImGuiCol background = ImGuiCol_Button;
    if (active) {
        background = ImGuiCol_ButtonActive;
    } else if (hovered) {
        background = ImGuiCol_ButtonHovered;
    }
    draw->AddRectFilled(pos, ImVec2(pos.x + size.x, pos.y + size.y), ImGui::GetColorU32(background),
                        style.FrameRounding);
    const ImU32 ink = ImGui::GetColorU32(emphasised ? ImGuiCol_CheckMark : ImGuiCol_Text);
    const float cx = pos.x + size.x * 0.5F;
    const float cy = pos.y + size.y * 0.5F;
    const float s = h * 0.22F;
    switch (glyph) {
        case Glyph::play:
            draw->AddTriangleFilled(ImVec2(cx - s * 0.75F, cy - s), ImVec2(cx - s * 0.75F, cy + s),
                                    ImVec2(cx + s, cy), ink);
            break;
        case Glyph::pause:
            draw->AddRectFilled(ImVec2(cx - s * 0.8F, cy - s), ImVec2(cx - s * 0.25F, cy + s), ink,
                                1.0F);
            draw->AddRectFilled(ImVec2(cx + s * 0.25F, cy - s), ImVec2(cx + s * 0.8F, cy + s), ink,
                                1.0F);
            break;
        case Glyph::step:
            draw->AddTriangleFilled(ImVec2(cx - s, cy - s), ImVec2(cx - s, cy + s),
                                    ImVec2(cx + s * 0.45F, cy), ink);
            draw->AddRectFilled(ImVec2(cx + s * 0.55F, cy - s), ImVec2(cx + s, cy + s), ink, 1.0F);
            break;
        case Glyph::reset:
            draw->AddRectFilled(ImVec2(cx - s, cy - s), ImVec2(cx - s * 0.55F, cy + s), ink, 1.0F);
            draw->AddTriangleFilled(ImVec2(cx + s, cy - s), ImVec2(cx + s, cy + s),
                                    ImVec2(cx - s * 0.45F, cy), ink);
            break;
    }
    item_tooltip(tooltip);
    return pressed;
}

// Rounded label in the warning colour on a faint warning background.
void warning_chip(const std::string& label, const char* tooltip) {
    const UiColor warning = default_theme().warning;
    const ImGuiStyle& style = ImGui::GetStyle();
    const ImVec2 text_size = ImGui::CalcTextSize(label.c_str());
    const float h = ImGui::GetFrameHeight();
    const float w = text_size.x + style.FramePadding.x * 2.0F;
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    ImGui::Dummy(ImVec2(w, h));
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const float inset = 3.0F;
    draw->AddRectFilled(ImVec2(pos.x, pos.y + inset), ImVec2(pos.x + w, pos.y + h - inset),
                        ImGui::GetColorU32(im(warning, 0.16F)), h * 0.5F);
    draw->AddRect(ImVec2(pos.x, pos.y + inset), ImVec2(pos.x + w, pos.y + h - inset),
                  ImGui::GetColorU32(im(warning, 0.55F)), h * 0.5F, 0, 1.0F);
    draw->AddText(ImVec2(pos.x + style.FramePadding.x, pos.y + (h - text_size.y) * 0.5F),
                  ImGui::GetColorU32(im(warning, 1.0F)), label.c_str());
    item_tooltip(tooltip);
}

std::string sim_time_label(double days) {
    if (std::abs(days) < 1000.0) {
        return "T " + std::string(days < 0.0 ? "-" : "+") + fixed(std::abs(days), 2) + " d";
    }
    return "T " + std::string(days < 0.0 ? "-" : "+") + fixed(std::abs(days) / 365.25, 2) + " yr";
}

// Simulated time per wall-clock second, in the most readable unit.
std::string rate_label(double warp) {
    const double days_per_second = warp / kSecondsPerDay;
    if (days_per_second >= 365.25) {
        return fixed(days_per_second / 365.25, 1) + " yr/s";
    }
    if (days_per_second >= 1.0) {
        return fixed(days_per_second, 1) + " d/s";
    }
    if (warp >= 3600.0) {
        return fixed(warp / 3600.0, 1) + " h/s";
    }
    if (warp >= 60.0) {
        return fixed(warp / 60.0, 1) + " min/s";
    }
    return fixed(warp, 0) + " s/s";
}

} // namespace

std::span<const std::int64_t> time_warp_presets() noexcept {
    return kWarpPresets;
}

std::int64_t step_time_warp(std::int64_t current, int direction) noexcept {
    const std::int64_t clamped = std::clamp(current, kWarpPresets.front(), kWarpPresets.back());
    if (direction > 0) {
        const auto next = std::upper_bound(kWarpPresets.begin(), kWarpPresets.end(), clamped);
        return next != kWarpPresets.end() ? *next : clamped;
    }
    if (direction < 0) {
        const auto next = std::lower_bound(kWarpPresets.begin(), kWarpPresets.end(), clamped);
        return next != kWarpPresets.begin() ? *(next - 1) : clamped;
    }
    return clamped;
}

std::string format_time_warp(double warp) {
    if (!std::isfinite(warp) || warp < 0.0) {
        return std::string("-") + kTimes;
    }
    if (warp < 9999.5) {
        return fixed(std::round(warp), 0) + kTimes;
    }
    int exponent = static_cast<int>(std::floor(std::log10(warp)));
    double mantissa = std::round(warp / std::pow(10.0, exponent) * 10.0) / 10.0;
    if (mantissa >= 10.0) {
        mantissa /= 10.0;
        ++exponent;
    }
    std::string text = fixed(mantissa, 1);
    if (text.size() >= 2 && text.substr(text.size() - 2) == ".0") {
        text.resize(text.size() - 2);
    }
    return text + "e" + std::to_string(exponent) + kTimes;
}

AppBarIntent draw_app_bar(const AppBarData& data) {
    AppBarIntent intent;
    const ImGuiStyle& base = ImGui::GetStyle();
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding,
                        ImVec2(base.FramePadding.x, base.FramePadding.y + 3.0F));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(6.0F, base.ItemSpacing.y));
    if (!ImGui::BeginMainMenuBar()) {
        ImGui::PopStyleVar(2);
        return intent;
    }
    const ImGuiStyle& style = ImGui::GetStyle();
    const float gap = style.ItemSpacing.x * 2.0F;

    word_mark();
    ImGui::Dummy(ImVec2(gap, 0.0F));
    workspace_tab(kSpaceWorkspace, true);
    ImGui::Dummy(ImVec2(gap * 0.5F, 0.0F));
    ImGui::Separator();

    // Transport.
    if (icon_button("##reset", Glyph::reset, "Reset to the initial state of the scenario", false)) {
        intent.reset = true;
    }
    if (icon_button("##play", data.paused ? Glyph::play : Glyph::pause,
                    data.paused ? "Run (Space)" : "Pause (Space)", data.paused)) {
        intent.set_paused = !data.paused;
    }
    if (icon_button("##step", Glyph::step, "Advance one physics step and pause (. or Right)",
                    false)) {
        intent.single_step = true;
    }
    ImGui::Separator();

    // Time warp: arrows step through the presets, the value opens the list.
    muted("WARP");
    if (ImGui::ArrowButton("##warp_down", ImGuiDir_Left)) {
        intent.set_time_warp = step_time_warp(data.time_warp, -1);
    }
    item_tooltip("Slower");
    const std::string warp_text = format_time_warp(static_cast<double>(data.time_warp));
    const float value_width = ImGui::CalcTextSize("0000000").x + style.FramePadding.x * 2.0F;
    if (ImGui::Button((warp_text + "##warp_value").c_str(), ImVec2(value_width, 0.0F))) {
        ImGui::OpenPopup("##warp_presets");
    }
    item_tooltip("Time warp: simulated seconds per second (click for presets)");
    if (ImGui::BeginPopup("##warp_presets")) {
        for (const std::int64_t preset : kWarpPresets) {
            const std::string label = format_time_warp(static_cast<double>(preset)) + "   " +
                                      rate_label(static_cast<double>(preset));
            if (ImGui::Selectable(label.c_str(), preset == data.time_warp)) {
                intent.set_time_warp = preset;
            }
        }
        ImGui::EndPopup();
    }
    if (ImGui::ArrowButton("##warp_up", ImGuiDir_Right)) {
        intent.set_time_warp = step_time_warp(data.time_warp, +1);
    }
    item_tooltip("Faster");
    const double shown_warp =
        data.paused
            ? static_cast<double>(data.time_warp)
            : (data.warp_limited ? data.effective_warp : static_cast<double>(data.time_warp));
    muted(rate_label(shown_warp).c_str());
    if (data.warp_limited && !data.paused) {
        warning_chip("LIMITED " + format_time_warp(data.effective_warp),
                     "The physics step budget cannot sustain the requested warp:\n"
                     "simulated time advances at the achieved rate shown here.\n"
                     "Lower the warp or use a larger time step.");
    }
    ImGui::Separator();

    // Scenario selector.
    muted("SCENARIO");
    std::string preview(data.scenario_label);
    if (data.scenario && *data.scenario < data.scenarios.size()) {
        preview = data.scenarios[*data.scenario];
    }
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 13.0F);
    if (ImGui::BeginCombo("##scenario", preview.c_str())) {
        for (std::size_t i = 0; i < data.scenarios.size(); ++i) {
            const bool selected = data.scenario == i;
            ImGui::PushID(static_cast<int>(i));
            if (ImGui::Selectable(data.scenarios[i].c_str(), selected)) {
                intent.load_scenario = i;
            }
            ImGui::PopID();
        }
        ImGui::EndCombo();
    }

    // Right-aligned status: message, simulated time, frame rate.
    const std::string time_text = sim_time_label(data.sim_time_days);
    const std::string fps_text = fixed(data.fps, 0) + " fps";
    std::string message(data.message);
    constexpr std::size_t kMaxMessage = 72;
    const bool truncated = message.size() > kMaxMessage;
    if (truncated) {
        message.resize(kMaxMessage - 3);
        message += "...";
    }
    float right = ImGui::CalcTextSize(time_text.c_str()).x +
                  ImGui::CalcTextSize(fps_text.c_str()).x + gap * 2.0F;
    if (!message.empty()) {
        right += ImGui::CalcTextSize(message.c_str()).x + gap;
    }
    const float start = ImGui::GetWindowContentRegionMax().x - right;
    if (start > ImGui::GetCursorPosX()) {
        ImGui::SetCursorPosX(start);
    }
    if (!message.empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, im(default_theme().warning, 1.0F));
        ImGui::TextUnformatted(message.c_str());
        ImGui::PopStyleColor();
        if (truncated) {
            item_tooltip(std::string(data.message).c_str());
        }
        ImGui::Dummy(ImVec2(gap * 0.5F, 0.0F));
    }
    ImGui::TextUnformatted(time_text.c_str());
    item_tooltip("Simulated time since the scenario epoch");
    ImGui::Dummy(ImVec2(gap * 0.5F, 0.0F));
    muted(fps_text.c_str());
    ImGui::EndMainMenuBar();
    ImGui::PopStyleVar(2);
    return intent;
}

SceneInput read_scene_input() {
    SceneInput input;
    const ImGuiIO& io = ImGui::GetIO();
    if (!io.WantCaptureMouse) {
        const bool valid_delta = std::isfinite(io.MouseDelta.x) && std::isfinite(io.MouseDelta.y) &&
                                 std::abs(io.MouseDelta.x) < 1.0e5F &&
                                 std::abs(io.MouseDelta.y) < 1.0e5F;
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && valid_delta) {
            input.drag_x = static_cast<double>(io.MouseDelta.x);
            input.drag_y = static_cast<double>(io.MouseDelta.y);
        }
        input.wheel = static_cast<double>(io.MouseWheel);
    }
    if (!io.WantCaptureKeyboard) {
        input.toggle_pause = ImGui::IsKeyPressed(ImGuiKey_Space, false);
        input.single_step =
            ImGui::IsKeyPressed(ImGuiKey_Period, false) || ImGui::IsKeyPressed(ImGuiKey_RightArrow);
        input.reset_view =
            ImGui::IsKeyPressed(ImGuiKey_Home, false) || ImGui::IsKeyPressed(ImGuiKey_F, false);
    }
    return input;
}

} // namespace orrery::render
