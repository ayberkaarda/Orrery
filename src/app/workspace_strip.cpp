#include "orrery/app/workspace_strip.h"

#include <array>
#include <string>

#include <imgui.h>
#include <imgui_internal.h>

namespace orrery::app {

namespace {

struct Tab {
    Workspace workspace;
    const char* label;
    const char* tooltip;
    bool enabled;
};

constexpr std::array<Tab, 3> kTabs{{
    {Workspace::space, "Space", "Space workspace: orbital dynamics and black holes (Ctrl+1)", true},
    {Workspace::aero, "Aero", "Aero workspace: geometry, flow conditions and solvers (Ctrl+2)",
     true},
    {Workspace::entry, "Entry", "Entry workspace: atmospheric entry (arrives in a later phase)",
     false},
}};

const char* caption(Workspace workspace) {
    switch (workspace) {
        case Workspace::space:
            return "Orbital dynamics";
        case Workspace::aero:
            return "Aerodynamics";
        case Workspace::entry:
            return "Atmospheric entry";
    }
    return "";
}

// One tab: the selected one on the panel colour with the accent underline,
// like the workspace tab of the main bar and the docked panel tabs.
bool tab(const Tab& t, bool selected) {
    const ImGuiStyle& style = ImGui::GetStyle();
    const ImVec2 text_size = ImGui::CalcTextSize(t.label);
    const float h = ImGui::GetFrameHeight();
    const float w = text_size.x + style.FramePadding.x * 4.0F;
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    ImGui::PushID(t.label);
    ImGui::BeginDisabled(!t.enabled);
    const bool pressed = ImGui::InvisibleButton("##workspace", ImVec2(w, h));
    ImGui::EndDisabled();
    const bool hovered = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled);
    ImGui::PopID();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImVec2 end(pos.x + w, pos.y + h);
    if (selected) {
        draw->AddRectFilled(pos, end, ImGui::GetColorU32(ImGuiCol_TabSelected), style.TabRounding);
        draw->AddRectFilled(ImVec2(pos.x + 4.0F, end.y - 2.0F), ImVec2(end.x - 4.0F, end.y),
                            ImGui::GetColorU32(ImGuiCol_TabSelectedOverline));
    } else if (hovered && t.enabled) {
        draw->AddRectFilled(pos, end, ImGui::GetColorU32(ImGuiCol_TabHovered), style.TabRounding);
    }
    ImU32 ink = ImGui::GetColorU32(ImGuiCol_TextDisabled);
    if (selected) {
        ink = ImGui::GetColorU32(ImGuiCol_Text);
    } else if (!t.enabled) {
        ink = ImGui::GetColorU32(ImGuiCol_TextDisabled, 0.55F);
    }
    draw->AddText(ImVec2(pos.x + (w - text_size.x) * 0.5F, pos.y + (h - text_size.y) * 0.5F), ink,
                  t.label);
    if (hovered && ImGui::BeginTooltip()) {
        ImGui::TextUnformatted(t.tooltip);
        ImGui::EndTooltip();
    }
    return pressed && t.enabled && !selected;
}

} // namespace

std::optional<Workspace> draw_workspace_strip(Workspace current) {
    std::optional<Workspace> picked;
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    const ImGuiStyle& base = ImGui::GetStyle();
    const float height = ImGui::GetFrameHeight() + base.WindowPadding.y * 0.6F;
    constexpr ImGuiWindowFlags kFlags = ImGuiWindowFlags_NoScrollbar |
                                        ImGuiWindowFlags_NoSavedSettings |
                                        ImGuiWindowFlags_NoScrollWithMouse;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding,
                        ImVec2(base.WindowPadding.x, base.WindowPadding.y * 0.3F));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(2.0F, base.ItemSpacing.y));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImGui::GetStyleColorVec4(ImGuiCol_TitleBg));
    if (ImGui::BeginViewportSideBar("##workspaces", viewport, ImGuiDir_Up, height, kFlags)) {
        for (const Tab& t : kTabs) {
            if (tab(t, t.workspace == current)) {
                picked = t.workspace;
            }
            ImGui::SameLine();
        }
        // Right-aligned caption of the current workspace.
        const char* text = caption(current);
        const float width = ImGui::CalcTextSize(text).x;
        ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - width);
        ImGui::AlignTextToFramePadding();
        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
        ImGui::TextUnformatted(text);
        ImGui::PopStyleColor();
    }
    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(2);
    return picked;
}

std::optional<Workspace> read_workspace_shortcut() {
    const ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureKeyboard || !io.KeyCtrl) {
        return std::nullopt;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_1, false)) {
        return Workspace::space;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_2, false)) {
        return Workspace::aero;
    }
    return std::nullopt;
}

} // namespace orrery::app
