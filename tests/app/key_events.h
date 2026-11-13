#pragma once

#include <imgui.h>

// Keyboard input for headless UI frames, queued on the Dear ImGui context
// the way the GLFW platform backend queues it, so that tests reach the
// shortcuts through the production path (read_key_frame and the window's
// dispatch). A held key types its character again on every key repeat;
// Dear ImGui derives key repeats from the frame times itself.
namespace orrery::app_test {

struct KeyStroke {
    ImGuiKey key = ImGuiKey_None;
    bool ctrl = false;
    bool shift = false;
    bool alt = false;
    unsigned int typed = 0; // the character the layout types, 0: none
};

// Queues the modifiers, the key going down and its character.
inline void key_down(const KeyStroke& stroke) {
    ImGuiIO& io = ImGui::GetIO();
    if (stroke.ctrl) {
        io.AddKeyEvent(ImGuiKey_LeftCtrl, true);
        io.AddKeyEvent(ImGuiMod_Ctrl, true);
    }
    if (stroke.shift) {
        io.AddKeyEvent(ImGuiKey_LeftShift, true);
        io.AddKeyEvent(ImGuiMod_Shift, true);
    }
    if (stroke.alt) {
        io.AddKeyEvent(ImGuiKey_RightAlt, true);
        io.AddKeyEvent(ImGuiMod_Alt, true);
    }
    io.AddKeyEvent(stroke.key, true);
    if (stroke.typed != 0) {
        io.AddInputCharacter(stroke.typed);
    }
}

// Queues the character of a key repeat (the key stays down).
inline void key_repeat(const KeyStroke& stroke) {
    if (stroke.typed != 0) {
        ImGui::GetIO().AddInputCharacter(stroke.typed);
    }
}

// Queues the key and its modifiers going up.
inline void key_up(const KeyStroke& stroke) {
    ImGuiIO& io = ImGui::GetIO();
    io.AddKeyEvent(stroke.key, false);
    if (stroke.ctrl) {
        io.AddKeyEvent(ImGuiKey_LeftCtrl, false);
        io.AddKeyEvent(ImGuiMod_Ctrl, false);
    }
    if (stroke.shift) {
        io.AddKeyEvent(ImGuiKey_LeftShift, false);
        io.AddKeyEvent(ImGuiMod_Shift, false);
    }
    if (stroke.alt) {
        io.AddKeyEvent(ImGuiKey_RightAlt, false);
        io.AddKeyEvent(ImGuiMod_Alt, false);
    }
}

} // namespace orrery::app_test
