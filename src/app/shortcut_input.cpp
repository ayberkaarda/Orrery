#include "orrery/app/shortcut_input.h"

#include <algorithm>
#include <array>
#include <string>

#include <imgui.h>
#include <imgui_internal.h>

namespace orrery::app {

namespace {

ImGuiKey imgui_key(Key key) {
    const auto index = static_cast<int>(key);
    if (key >= Key::a && key <= Key::z) {
        return static_cast<ImGuiKey>(ImGuiKey_A + (index - static_cast<int>(Key::a)));
    }
    if (key >= Key::d0 && key <= Key::d9) {
        return static_cast<ImGuiKey>(ImGuiKey_0 + (index - static_cast<int>(Key::d0)));
    }
    if (key >= Key::f1 && key <= Key::f12) {
        return static_cast<ImGuiKey>(ImGuiKey_F1 + (index - static_cast<int>(Key::f1)));
    }
    switch (key) {
        case Key::space:
            return ImGuiKey_Space;
        case Key::escape:
            return ImGuiKey_Escape;
        case Key::enter:
            return ImGuiKey_Enter;
        case Key::tab:
            return ImGuiKey_Tab;
        case Key::backspace:
            return ImGuiKey_Backspace;
        case Key::del:
            return ImGuiKey_Delete;
        case Key::insert:
            return ImGuiKey_Insert;
        case Key::home:
            return ImGuiKey_Home;
        case Key::end:
            return ImGuiKey_End;
        case Key::page_up:
            return ImGuiKey_PageUp;
        case Key::page_down:
            return ImGuiKey_PageDown;
        case Key::left:
            return ImGuiKey_LeftArrow;
        case Key::right:
            return ImGuiKey_RightArrow;
        case Key::up:
            return ImGuiKey_UpArrow;
        case Key::down:
            return ImGuiKey_DownArrow;
        case Key::period:
            return ImGuiKey_Period;
        case Key::comma:
            return ImGuiKey_Comma;
        case Key::slash:
            return ImGuiKey_Slash;
        case Key::minus:
            return ImGuiKey_Minus;
        case Key::equal:
            return ImGuiKey_Equal;
        case Key::left_bracket:
            return ImGuiKey_LeftBracket;
        case Key::right_bracket:
            return ImGuiKey_RightBracket;
        case Key::semicolon:
            return ImGuiKey_Semicolon;
        case Key::apostrophe:
            return ImGuiKey_Apostrophe;
        case Key::backslash:
            return ImGuiKey_Backslash;
        case Key::grave:
            return ImGuiKey_GraveAccent;
        default:
            return ImGuiKey_None;
    }
}

// Whether a keyboard key other than a modifier went down this frame (any
// key, also one without a Key of its own, such as a layout's extra key).
bool any_key_went_down() {
    for (int index = ImGuiKey_NamedKey_BEGIN; index < ImGuiKey_NamedKey_END; ++index) {
        const auto key = static_cast<ImGuiKey>(index);
        if (ImGui::IsKeyboardKey(key) && !ImGui::IsLRModKey(key) &&
            ImGui::IsKeyPressed(key, false)) {
            return true;
        }
    }
    return false;
}

// A key chord as a small rounded key cap on the frame colour.
void key_cap(const std::string& label) {
    const ImGuiStyle& style = ImGui::GetStyle();
    const ImVec2 text = ImGui::CalcTextSize(label.c_str());
    const float pad = style.FramePadding.x * 0.75F;
    // As tall as a framed row, so the description beside it lines up.
    const float h = ImGui::GetFrameHeight() - 2.0F;
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    const ImVec2 size(text.x + pad * 2.0F, h);
    ImGui::Dummy(size);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(pos, ImVec2(pos.x + size.x, pos.y + size.y),
                        ImGui::GetColorU32(ImGuiCol_FrameBg), 4.0F);
    draw->AddRect(pos, ImVec2(pos.x + size.x, pos.y + size.y), ImGui::GetColorU32(ImGuiCol_Border),
                  4.0F);
    draw->AddText(ImVec2(pos.x + pad, pos.y + (h - text.y) * 0.5F),
                  ImGui::GetColorU32(ImGuiCol_Text), label.c_str());
}

void muted(const char* text) {
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::TextUnformatted(text);
    ImGui::PopStyleColor();
}

struct Group {
    std::uint8_t workspaces;
    const char* caption;
};

void draw_group(const ShortcutTable& table, const Group& group) {
    ImGui::Dummy(ImVec2(0.0F, 2.0F));
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::SeparatorText(group.caption);
    ImGui::PopStyleColor();
    if (!ImGui::BeginTable(group.caption, 2, ImGuiTableFlags_SizingFixedFit)) {
        return;
    }
    // Fixed widths: every group lines up, whatever its own keys.
    ImGui::TableSetupColumn("keys", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFontSize() * 8.0F);
    ImGui::TableSetupColumn("action", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::GetFontSize() * 24.0F);
    for (const ShortcutInfo& info : shortcut_actions()) {
        if (info.workspaces != group.workspaces) {
            continue;
        }
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        const std::vector<KeyChord>& chords = table.of(info.action);
        if (chords.empty()) {
            muted("unbound");
        }
        for (std::size_t i = 0; i < chords.size(); ++i) {
            if (i > 0) {
                ImGui::SameLine(0.0F, 4.0F);
            }
            key_cap(format_key_chord(chords[i]));
        }
        ImGui::TableSetColumnIndex(1);
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(info.description.data(),
                               info.description.data() + info.description.size());
    }
    ImGui::EndTable();
}

} // namespace

KeyFrame read_key_frame() {
    KeyFrame frame;
    const ImGuiIO& io = ImGui::GetIO();
    frame.text_input = io.WantCaptureKeyboard || io.WantTextInput;
    for (std::size_t i = 1; i < kKeyCount; ++i) {
        const auto key = static_cast<Key>(i);
        const ImGuiKey imgui = imgui_key(key);
        if (imgui == ImGuiKey_None || !ImGui::IsKeyDown(imgui)) {
            continue;
        }
        KeyChord chord;
        chord.key = key;
        chord.ctrl = io.KeyCtrl;
        chord.shift = io.KeyShift;
        chord.alt = io.KeyAlt;
        if (ImGui::IsKeyPressed(imgui, false)) {
            frame.pressed.push_back(chord);
        } else if (ImGui::IsKeyPressed(imgui, true)) {
            frame.repeated.push_back(chord);
        } else {
            frame.held.push_back(chord);
        }
    }
    // A character arrives with the key event that typed it (the platform
    // backend queues both in one poll, and Dear ImGui keeps them in one frame
    // while no text field is active), so characters of a frame in which no
    // key went down were typed by key repeat.
    std::string& typed = any_key_went_down() ? frame.typed : frame.typed_repeated;
    for (const ImWchar c : io.InputQueueCharacters) {
        if (c > 0x20 && c < 0x7F) {
            typed += static_cast<char>(c);
        }
    }
    return frame;
}

render::SceneInput scene_input_from(render::SceneInput pointer,
                                    std::span<const ShortcutAction> actions) {
    const auto has = [&](ShortcutAction action) {
        return std::find(actions.begin(), actions.end(), action) != actions.end();
    };
    pointer.toggle_pause = has(ShortcutAction::toggle_pause);
    pointer.single_step = has(ShortcutAction::single_step);
    pointer.reset_view = has(ShortcutAction::reset_view);
    return pointer;
}

void draw_shortcut_help(const ShortcutTable& table, Workspace workspace, bool& open,
                        std::string_view config_path) {
    if (!open) {
        return;
    }
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.5F,
                                   viewport->WorkPos.y + viewport->WorkSize.y * 0.5F),
                            ImGuiCond_Always, ImVec2(0.5F, 0.5F));
    ImGui::SetNextWindowFocus();
    // Opaque: it sits over the panels.
    ImGui::SetNextWindowBgAlpha(1.0F);
    constexpr ImGuiWindowFlags kFlags = ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoCollapse |
                                        ImGuiWindowFlags_AlwaysAutoResize |
                                        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoMove;
    const std::string title(kShortcutHelpWindow);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(18.0F, 14.0F));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.0F);
    const bool shown = ImGui::Begin(title.c_str(), &open, kFlags);
    ImGui::PopStyleVar(2);
    if (shown) {
        const std::uint8_t here = workspace_bit(workspace);
        constexpr std::array<Group, 3> kGroups{{
            {kInAll, "ALL WORKSPACES"},
            {static_cast<std::uint8_t>(kInSpace | kInAero), "SPACE AND AERO"},
            {kInSpace, "SPACE"},
        }};
        // The groups of the shown workspace first, the others after them.
        for (const bool applies : {true, false}) {
            for (const Group& group : kGroups) {
                if (((group.workspaces & here) != 0U) == applies) {
                    draw_group(table, group);
                }
            }
        }
        ImGui::Dummy(ImVec2(0.0F, 4.0F));
        muted("Keys are set in the [shortcuts] table of");
        const std::string path(config_path);
        muted(path.empty() ? "config.toml" : path.c_str());
    }
    ImGui::End();
}

void focus_window(std::string_view name) {
    const std::string window(name);
    if (ImGui::FindWindowByName(window.c_str()) != nullptr) {
        ImGui::SetWindowFocus(window.c_str());
    }
}

} // namespace orrery::app
