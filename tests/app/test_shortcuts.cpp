#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "key_events.h"
#include "orrery/app/config.h"
#include "orrery/app/shortcut_input.h"
#include "orrery/app/shortcuts.h"
#include "orrery/render/ui_context.h"

// Keyboard shortcuts (docs/app.md, "Keyboard shortcuts"): the table, the
// chord parser, conflicts, the configuration overrides, matching against a
// frame's keys, and the help overlay under a headless UI context. No window.

using namespace orrery;
using namespace orrery::app;

namespace {

KeyChord chord(std::string_view text) {
    auto parsed = parse_key_chord(text);
    INFO(text);
    REQUIRE(parsed.has_value());
    return parsed.value();
}

KeyFrame pressed(std::string_view text) {
    KeyFrame frame;
    frame.pressed.push_back(chord(text));
    return frame;
}

bool fires(const ShortcutTable& table, const KeyFrame& frame, Workspace workspace,
           ShortcutAction action) {
    const auto actions = match_shortcuts(table, frame, workspace);
    return std::find(actions.begin(), actions.end(), action) != actions.end();
}

bool contains(const std::string& text, const char* part) {
    return text.find(part) != std::string::npos;
}

bool any_contains(const std::vector<std::string>& lines, const char* part) {
    return std::any_of(lines.begin(), lines.end(),
                       [&](const std::string& line) { return contains(line, part); });
}

} // namespace

TEST_CASE("Every shortcut action has a unique name and a description", "[app][shortcuts]") {
    const auto actions = shortcut_actions();
    REQUIRE(actions.size() == kShortcutActionCount);
    for (std::size_t i = 0; i < actions.size(); ++i) {
        INFO(actions[i].name);
        CHECK(static_cast<std::size_t>(actions[i].action) == i);
        CHECK_FALSE(actions[i].description.empty());
        CHECK(shortcut_action_from_name(actions[i].name) == actions[i].action);
    }
    CHECK_FALSE(shortcut_action_from_name("fly").has_value());
}

TEST_CASE("Key chords parse and format canonically", "[app][shortcuts]") {
    CHECK(chord("Ctrl+S") == KeyChord{Key::s, '\0', true, false, false});
    CHECK(chord("ctrl+shift+f1") == KeyChord{Key::f1, '\0', true, true, false});
    CHECK(chord("Space").key == Key::space);
    CHECK(chord("escape").key == Key::escape);
    CHECK(chord("Esc").key == Key::escape);
    CHECK(chord("?").character == '?');
    CHECK(chord("[").character == '[');
    CHECK(chord("+").character == '+');
    CHECK(chord("Ctrl+[") == KeyChord{Key::left_bracket, '\0', true, false, false});
    CHECK(chord("Alt+1") == KeyChord{Key::d1, '\0', false, false, true});
    CHECK_FALSE(parse_key_chord("").has_value());
    CHECK_FALSE(parse_key_chord("Hyper+S").has_value());
    CHECK_FALSE(parse_key_chord("Ctrl+").has_value());
    CHECK_FALSE(parse_key_chord("Banana").has_value());
    const auto shifted = parse_key_chord("Ctrl+?");
    REQUIRE_FALSE(shifted.has_value());
    CHECK(contains(shifted.error(), "Shift+/"));

    for (const KeyChord& c : {chord("Ctrl+S"), chord("Shift+F12"), chord("?"), chord("]"),
                              chord("Ctrl+Alt+Delete"), chord("PageDown"), chord("Ctrl+/")}) {
        const std::string text = format_key_chord(c);
        INFO(text);
        CHECK(chord(text) == c);
    }
    CHECK(format_key_chord(chord("ctrl+s")) == "Ctrl+S");
    CHECK(format_key_chord(chord("shift+ctrl+right")) == "Ctrl+Shift+Right");
}

TEST_CASE("The default table covers the documented keys without a conflict", "[app][shortcuts]") {
    const ShortcutTable table = default_shortcuts();
    CHECK(find_conflicts(table).empty());
    for (const ShortcutInfo& info : shortcut_actions()) {
        INFO(info.name);
        CHECK_FALSE(table.of(info.action).empty());
    }
    CHECK(fires(table, pressed("Space"), Workspace::entry, ShortcutAction::toggle_pause));
    CHECK(fires(table, pressed("Ctrl+3"), Workspace::space, ShortcutAction::workspace_entry));
    CHECK(fires(table, pressed("Ctrl+S"), Workspace::aero, ShortcutAction::save));
    CHECK(fires(table, pressed("Ctrl+O"), Workspace::entry, ShortcutAction::open));
    CHECK(fires(table, pressed("F1"), Workspace::aero, ShortcutAction::help));
    CHECK(fires(table, pressed("Esc"), Workspace::space, ShortcutAction::close));
    CHECK(fires(table, pressed("Ctrl+R"), Workspace::space, ShortcutAction::reset));
    KeyFrame typed;
    typed.typed = "?";
    CHECK(fires(table, typed, Workspace::entry, ShortcutAction::help));
    typed.typed = "]";
    CHECK(fires(table, typed, Workspace::space, ShortcutAction::warp_up));
    // The warp keys belong to the Space workspace only.
    CHECK_FALSE(fires(table, typed, Workspace::aero, ShortcutAction::warp_up));
    // Modifiers must match exactly: Ctrl+Space is not Space.
    CHECK_FALSE(
        fires(table, pressed("Ctrl+Space"), Workspace::space, ShortcutAction::toggle_pause));
}

TEST_CASE("Matching respects repeats and text input", "[app][shortcuts]") {
    const ShortcutTable table = default_shortcuts();
    KeyFrame held;
    held.repeated.push_back(chord("Right"));
    held.repeated.push_back(chord("Space"));
    // Stepping repeats while held; run/pause does not.
    CHECK(fires(table, held, Workspace::space, ShortcutAction::single_step));
    CHECK_FALSE(fires(table, held, Workspace::space, ShortcutAction::toggle_pause));
    // Characters typed by key repeat follow the same rule.
    KeyFrame repeated_text;
    repeated_text.typed_repeated = "?.";
    CHECK(fires(table, repeated_text, Workspace::space, ShortcutAction::single_step));
    CHECK_FALSE(fires(table, repeated_text, Workspace::space, ShortcutAction::help));
    KeyFrame typing = pressed("Space");
    typing.typed = "?";
    typing.text_input = true;
    CHECK(match_shortcuts(table, typing, Workspace::space).empty());
    // Each action at most once, in table order.
    KeyFrame both = pressed("Right");
    both.typed = "..";
    const auto actions = match_shortcuts(table, both, Workspace::aero);
    CHECK(std::count(actions.begin(), actions.end(), ShortcutAction::single_step) == 1);
}

TEST_CASE("Conflicts are found across key and character chords", "[app][shortcuts]") {
    ShortcutTable table = default_shortcuts();
    table.of(ShortcutAction::reset) = {chord("Ctrl+S")};
    auto conflicts = find_conflicts(table);
    REQUIRE(conflicts.size() == 1);
    CHECK(describe(conflicts.front()) == "Ctrl+S is bound to both reset and save");

    // "." typed is the period key on a US layout.
    table = default_shortcuts();
    table.of(ShortcutAction::warp_up) = {chord("Shift+/")};
    conflicts = find_conflicts(table);
    REQUIRE(conflicts.size() == 1); // with help's "?"
    CHECK(conflicts.front().second == ShortcutAction::help);

    // Actions of different workspaces may share a chord.
    table = default_shortcuts();
    table.of(ShortcutAction::warp_up) = {chord("Ctrl+Shift+W")};
    table.of(ShortcutAction::reset) = {chord("Ctrl+Shift+W")};
    CHECK(find_conflicts(table).size() == 1); // both are Space actions
}

TEST_CASE("Overrides apply, swap and are rejected on conflict", "[app][shortcuts]") {
    ShortcutTable table = default_shortcuts();
    std::vector<ShortcutOverride> overrides{
        {ShortcutAction::save, {chord("F5")}, "config.toml:2:1"},
        // A swap: each takes the other's default.
        {ShortcutAction::warp_down, {chord("]")}, "config.toml:3:1"},
        {ShortcutAction::warp_up, {chord("[")}, "config.toml:4:1"},
        {ShortcutAction::open, {}, "config.toml:5:1"},
        // Conflicts with toggle_pause's default: rejected.
        {ShortcutAction::reset_view, {chord("Space")}, "config.toml:6:1"},
    };
    const std::vector<std::string> warnings = apply_shortcut_overrides(table, overrides);
    REQUIRE(warnings.size() == 1);
    CHECK(contains(warnings.front(), "config.toml:6:1: shortcuts.reset_view: Space is bound to "
                                     "both toggle_pause and reset_view; keeping F, Home"));
    CHECK(table.of(ShortcutAction::save) == std::vector<KeyChord>{chord("F5")});
    CHECK(table.of(ShortcutAction::warp_down) == std::vector<KeyChord>{chord("]")});
    CHECK(table.of(ShortcutAction::warp_up) == std::vector<KeyChord>{chord("[")});
    CHECK(table.of(ShortcutAction::open).empty());
    CHECK(table.of(ShortcutAction::reset_view) ==
          default_shortcuts().of(ShortcutAction::reset_view));
    CHECK(find_conflicts(table).empty());
}

TEST_CASE("The configuration reads shortcuts, the physics budget and the entry workspace",
          "[app][shortcuts][config]") {
    const auto loaded = parse_config(R"(
[window]
workspace = "entry"
[space]
physics_budget = 0.4
[shortcuts]
save = "F5"
help = ["F2", "?"]
open = []
fly = "F"
reset = "Hyper+R"
close = 3
single_step = "Ctrl+1"
)",
                                     "config.toml");
    REQUIRE(loaded.has_value());
    const AppConfig& c = loaded.value().config;
    const std::vector<std::string>& warnings = loaded.value().warnings;
    CHECK(c.start_workspace == Workspace::entry);
    CHECK(c.physics_budget == 0.4);
    CHECK(c.shortcuts.of(ShortcutAction::save) == std::vector<KeyChord>{chord("F5")});
    CHECK(c.shortcuts.of(ShortcutAction::help) == std::vector<KeyChord>{chord("F2"), chord("?")});
    CHECK(c.shortcuts.of(ShortcutAction::open).empty());
    const ShortcutTable defaults = default_shortcuts();
    CHECK(c.shortcuts.of(ShortcutAction::reset) == defaults.of(ShortcutAction::reset));
    CHECK(c.shortcuts.of(ShortcutAction::close) == defaults.of(ShortcutAction::close));
    CHECK(c.shortcuts.of(ShortcutAction::single_step) == defaults.of(ShortcutAction::single_step));
    CHECK(find_conflicts(c.shortcuts).empty());
    // Nothing bad is dropped silently: one warning per rejected value.
    REQUIRE(warnings.size() == 4);
    CHECK(any_contains(warnings, "config.toml:10:"));
    CHECK(any_contains(warnings, "shortcuts.fly: unknown action"));
    CHECK(any_contains(warnings, "shortcuts.reset: 'Hyper' is not a modifier"));
    CHECK(any_contains(warnings, "shortcuts.close: expected a key chord string"));
    CHECK(any_contains(warnings, "shortcuts.single_step: Ctrl+1 is bound to both"));

    const auto bad = parse_config("[space]\nphysics_budget = 2\n[window]\nworkspace = \"lab\"\n"
                                  "shortcuts = 1\n",
                                  "c.toml");
    REQUIRE(bad.has_value());
    CHECK(bad.value().config.physics_budget == 0.6);
    REQUIRE(bad.value().warnings.size() == 3);
    CHECK(any_contains(bad.value().warnings, "space.physics_budget: expected a number"));
    CHECK(any_contains(bad.value().warnings, "window.shortcuts: unknown key"));
    CHECK(any_contains(bad.value().warnings, R"(expected "space", "aero" or "entry")"));
}

TEST_CASE("Shortcuts and the budget round-trip through the configuration file",
          "[app][shortcuts][config]") {
    AppConfig c = default_config();
    c.start_workspace = Workspace::entry;
    c.physics_budget = 0.35;
    c.shortcuts.of(ShortcutAction::save) = {chord("Ctrl+Shift+S"), chord("F5")};
    c.shortcuts.of(ShortcutAction::open) = {};
    const std::string text = config_to_toml(c);
    const auto loaded = parse_config(text, "config.toml");
    REQUIRE(loaded.has_value());
    CHECK(loaded.value().warnings.empty());
    CHECK(loaded.value().config == c);
}

TEST_CASE("The help overlay lists every action under a headless UI", "[app][shortcuts][ui]") {
    auto created = render::UiContext::create_headless();
    REQUIRE(created.has_value());
    render::UiContext ui = std::move(created).value();
    render::UiFrameInput input;
    input.width = 1600;
    input.height = 900;
    bool open = true;
    for (int frame = 0; frame < 3; ++frame) {
        ui.begin_frame(input);
        const KeyFrame keys = read_key_frame();
        CHECK(keys.pressed.empty());
        CHECK(keys.typed.empty());
        draw_shortcut_help(default_shortcuts(), Workspace::space, open, "config.toml");
        ui.end_frame();
    }
    CHECK(open);
    const auto rect = ui.window_rect(kShortcutHelpWindow);
    REQUIRE(rect.has_value());
    // Centred, readable, inside a 1600x900 window.
    CHECK(rect->width > 300.0);
    CHECK(rect->height > 300.0);
    CHECK(rect->x > 0.0);
    CHECK(rect->y > 0.0);
    CHECK(rect->x + rect->width < 1600.0);
    CHECK(rect->y + rect->height < 900.0);
    CHECK(std::abs(rect->x + rect->width * 0.5 - 800.0) < 2.0);

    open = false;
    for (int frame = 0; frame < 2; ++frame) {
        ui.begin_frame(input);
        draw_shortcut_help(default_shortcuts(), Workspace::space, open, "config.toml");
        ui.end_frame();
    }
    CHECK_FALSE(ui.window_rect(kShortcutHelpWindow).has_value());
}

namespace {

// Headless UI frames fed with keyboard events (key_events.h) and matched
// through the production path: read_key_frame, then match_shortcuts.
class KeyFrames {
public:
    explicit KeyFrames(ShortcutTable table) : table_(std::move(table)) {
        auto created = render::UiContext::create_headless();
        REQUIRE(created.has_value());
        ui_.emplace(std::move(created).value());
        input_.width = 1600;
        input_.height = 900;
        input_.delta_time = 0.1; // past the key repeat delay within a few frames
        (void)frame([] {});      // makes the context current for the events
    }

    // Queues the events of `queue`, runs one frame and returns its actions.
    template <typename Queue> std::vector<ShortcutAction> frame(Queue queue) {
        queue();
        ui_->begin_frame(input_);
        std::vector<ShortcutAction> actions =
            match_shortcuts(table_, read_key_frame(), Workspace::space);
        ui_->end_frame();
        return actions;
    }

    // Presses `stroke`, holds it for `frames` frames in all (the character
    // typed again on each) and releases it; how often `action` fired.
    std::ptrdiff_t hold(const app_test::KeyStroke& stroke, int frames, ShortcutAction action) {
        std::ptrdiff_t count = 0;
        for (int i = 0; i < frames; ++i) {
            const auto actions = frame([&] {
                if (i == 0) {
                    app_test::key_down(stroke);
                } else {
                    app_test::key_repeat(stroke);
                }
            });
            count += std::count(actions.begin(), actions.end(), action);
        }
        (void)frame([&] { app_test::key_up(stroke); });
        return count;
    }

private:
    ShortcutTable table_;
    std::optional<render::UiContext> ui_;
    render::UiFrameInput input_;
};

} // namespace

TEST_CASE("A held character fires a non-repeating action once per press",
          "[app][shortcuts][input]") {
    ShortcutTable table = default_shortcuts();
    table.of(ShortcutAction::save) = {chord(";")};
    REQUIRE(find_conflicts(table).empty());
    KeyFrames keys(table);
    const app_test::KeyStroke semicolon{ImGuiKey_Semicolon, false, false, false, ';'};
    const app_test::KeyStroke question{ImGuiKey_Slash, false, true, false, '?'};
    const app_test::KeyStroke bracket{ImGuiKey_LeftBracket, false, false, false, '['};

    CHECK(keys.hold(semicolon, 6, ShortcutAction::save) == 1);
    CHECK(keys.hold(semicolon, 1, ShortcutAction::save) == 1); // a new press fires again
    CHECK(keys.hold(question, 6, ShortcutAction::help) == 1);
    // Warp shortcuts repeat: one step per typed character.
    CHECK(keys.hold(bracket, 6, ShortcutAction::warp_down) == 6);
}

TEST_CASE("One key event fires one action when a layout types a bound character",
          "[app][shortcuts][input]") {
    // German layout: AltGr+8 (Ctrl+Alt on Windows) types "[", warp_down's
    // default. A table that also binds Ctrl+Alt+8 (bypassing the conflict
    // check) must not run both.
    ShortcutTable table = default_shortcuts();
    table.of(ShortcutAction::save) = {chord("Ctrl+Alt+8")};
    KeyFrames keys(table);
    const app_test::KeyStroke altgr_8{ImGuiKey_8, true, false, true, '['};
    const auto first = keys.frame([&] { app_test::key_down(altgr_8); });
    CHECK(first == std::vector<ShortcutAction>{ShortcutAction::save});
    // Held: the key repeats and types "[" again; still not warp_down.
    for (int i = 0; i < 5; ++i) {
        const auto held = keys.frame([&] { app_test::key_repeat(altgr_8); });
        CHECK(std::count(held.begin(), held.end(), ShortcutAction::warp_down) == 0);
        CHECK(std::count(held.begin(), held.end(), ShortcutAction::save) == 0);
    }
    (void)keys.frame([&] { app_test::key_up(altgr_8); });
    // The same character from an unbound key still lowers the warp.
    const app_test::KeyStroke bracket{ImGuiKey_LeftBracket, false, false, false, '['};
    CHECK(keys.hold(bracket, 1, ShortcutAction::warp_down) == 1);

    // The conflict check knows that Alt chords type layout characters.
    const auto conflicts = find_conflicts(table);
    CHECK(std::any_of(conflicts.begin(), conflicts.end(), [](const ShortcutConflict& c) {
        return c.first == ShortcutAction::warp_down && c.second == ShortcutAction::save;
    }));
    ShortcutTable configured = default_shortcuts();
    const std::vector<ShortcutOverride> overrides{
        {ShortcutAction::save, {chord("Ctrl+Alt+8")}, "config.toml:2:1"}};
    const std::vector<std::string> warnings = apply_shortcut_overrides(configured, overrides);
    REQUIRE(warnings.size() == 1);
    CHECK(contains(warnings.front(), "config.toml:2:1: shortcuts.save: "));
    CHECK(contains(warnings.front(), "keeping Ctrl+S"));
    CHECK(configured == default_shortcuts());
    // Ctrl alone types nothing: Ctrl+8 stays free.
    configured.of(ShortcutAction::save) = {chord("Ctrl+8")};
    CHECK(find_conflicts(configured).empty());
}

TEST_CASE("Scene input takes its keys from the shortcut actions", "[app][shortcuts]") {
    render::SceneInput pointer;
    pointer.drag_x = 3.0;
    pointer.toggle_pause = true; // a hard-wired key is overridden
    const std::vector<ShortcutAction> actions{ShortcutAction::reset_view};
    const render::SceneInput input = scene_input_from(pointer, actions);
    CHECK(input.drag_x == 3.0);
    CHECK_FALSE(input.toggle_pause);
    CHECK_FALSE(input.single_step);
    CHECK(input.reset_view);
}
