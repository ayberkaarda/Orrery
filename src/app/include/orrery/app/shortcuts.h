#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "orrery/app/workspace.h"
#include "orrery/core/result.h"

// Keyboard shortcuts of the window (docs/app.md, "Keyboard shortcuts"): one
// table of actions and their key chords, read from the configuration's
// [shortcuts] section, checked for conflicts, and matched against the keys
// of a frame. Graphics-free: the window turns Dear ImGui's key state into a
// KeyFrame (shortcut_input.h), everything else here is plain data, so the
// table, the parser, the conflict check and the matching are tested without
// a window.
namespace orrery::app {

enum class ShortcutAction : std::uint8_t {
    toggle_pause,
    single_step,
    reset_view,
    warp_down,
    warp_up,
    reset,
    workspace_space,
    workspace_aero,
    workspace_entry,
    save,
    open,
    help,
    close,
};

inline constexpr std::size_t kShortcutActionCount = 13;

// Workspaces an action applies in (bit mask).
inline constexpr std::uint8_t kInSpace = 1U;
inline constexpr std::uint8_t kInAero = 2U;
inline constexpr std::uint8_t kInEntry = 4U;
inline constexpr std::uint8_t kInAll = kInSpace | kInAero | kInEntry;

[[nodiscard]] std::uint8_t workspace_bit(Workspace workspace) noexcept;

struct ShortcutInfo {
    ShortcutAction action = ShortcutAction::help;
    std::string_view name;        // configuration key, e.g. "toggle_pause"
    std::string_view description; // help overlay text
    std::uint8_t workspaces = kInAll;
    bool repeats = false; // also fires on key repeat while held
};

// Every action in declaration order.
[[nodiscard]] std::span<const ShortcutInfo> shortcut_actions() noexcept;
[[nodiscard]] const ShortcutInfo& shortcut_info(ShortcutAction action) noexcept;
[[nodiscard]] std::optional<ShortcutAction>
shortcut_action_from_name(std::string_view name) noexcept;

// Keys a chord can name. Letters and digits are the US-layout key positions
// GLFW reports; punctuation keys are named by their unshifted US symbol.
enum class Key : std::uint8_t {
    none,
    a,
    b,
    c,
    d,
    e,
    f,
    g,
    h,
    i,
    j,
    k,
    l,
    m,
    n,
    o,
    p,
    q,
    r,
    s,
    t,
    u,
    v,
    w,
    x,
    y,
    z,
    d0,
    d1,
    d2,
    d3,
    d4,
    d5,
    d6,
    d7,
    d8,
    d9,
    f1,
    f2,
    f3,
    f4,
    f5,
    f6,
    f7,
    f8,
    f9,
    f10,
    f11,
    f12,
    space,
    escape,
    enter,
    tab,
    backspace,
    del,
    insert,
    home,
    end,
    page_up,
    page_down,
    left,
    right,
    up,
    down,
    period,
    comma,
    slash,
    minus,
    equal,
    left_bracket,
    right_bracket,
    semicolon,
    apostrophe,
    backslash,
    grave,
};

inline constexpr std::size_t kKeyCount = static_cast<std::size_t>(Key::grave) + 1;

// A key with modifiers, or a typed character. Character chords (a single
// punctuation character such as "?", "[" or ".") match what the keyboard
// layout types, so they work on any layout; key chords match a key position
// with exactly the given modifiers.
struct KeyChord {
    Key key = Key::none;
    char character = '\0'; // set for character chords (key is none)
    bool ctrl = false;
    bool shift = false;
    bool alt = false;

    friend bool operator==(const KeyChord&, const KeyChord&) = default;
};

// Parses "Ctrl+S", "Shift+F1", "F1", "Space", "Esc", "Right", "?", "[",
// "Ctrl+[" (modifier names and key names are case-insensitive; "+" alone is
// the plus character). A punctuation character with a modifier names its
// key ("Ctrl+[" is the left bracket key); a shifted symbol such as "?" has no
// key of its own and takes no modifier. The error is one sentence.
[[nodiscard]] core::Result<KeyChord, std::string> parse_key_chord(std::string_view text);
// Canonical text: modifiers in the order Ctrl, Shift, Alt, then the key
// ("Ctrl+S", "F1", "Space", "[" ...). parse_key_chord(format_key_chord(c)) == c.
[[nodiscard]] std::string format_key_chord(const KeyChord& chord);
[[nodiscard]] std::string_view key_name(Key key) noexcept;

struct ShortcutTable {
    std::array<std::vector<KeyChord>, kShortcutActionCount> bindings;

    [[nodiscard]] const std::vector<KeyChord>& of(ShortcutAction action) const noexcept;
    [[nodiscard]] std::vector<KeyChord>& of(ShortcutAction action) noexcept;

    friend bool operator==(const ShortcutTable&, const ShortcutTable&) = default;
};

// Space runs or pauses; "." or Right steps; F or Home reframes; "[" and "]"
// lower and raise the time warp, Ctrl+R resets the scenario (Space
// workspace); Ctrl+1/2/3 switch workspaces; Ctrl+S saves and Ctrl+O shows
// the saved experiments; F1 or "?" toggles the help overlay; Esc closes it.
[[nodiscard]] ShortcutTable default_shortcuts();

// Two actions that would both fire on one chord in some workspace. Chords
// conflict when they are equal, or when a character chord is what an
// unmodified or shifted key types on a US layout ("." and the period key).
struct ShortcutConflict {
    ShortcutAction first = ShortcutAction::help;
    ShortcutAction second = ShortcutAction::help;
    KeyChord chord;
};

[[nodiscard]] std::vector<ShortcutConflict> find_conflicts(const ShortcutTable& table);
// "Ctrl+S is bound to both save and reset".
[[nodiscard]] std::string describe(const ShortcutConflict& conflict);

// A user binding from the configuration (an empty list unbinds the action).
struct ShortcutOverride {
    ShortcutAction action = ShortcutAction::help;
    std::vector<KeyChord> chords;
    std::string where; // "config.toml:12:1", for the warning
};

// Applies the overrides to `table`. An override that conflicts with another
// action is not applied (that action keeps its default) and reported, so the
// resulting table never has a conflict; swapping two keys between actions
// works. Returns one warning per rejected override.
[[nodiscard]] std::vector<std::string>
apply_shortcut_overrides(ShortcutTable& table, std::span<const ShortcutOverride> overrides);

// The keys of one frame.
struct KeyFrame {
    std::vector<KeyChord> pressed;  // keys going down this frame, with modifiers
    std::vector<KeyChord> repeated; // keys repeating while held
    // Characters typed this frame by a key going down, and by key repeat (a
    // held key types its character again on every repeat).
    std::string typed;
    std::string typed_repeated;
    // A text field has the keyboard: no shortcut fires.
    bool text_input = false;
};

// The actions whose chords occurred in `frame` and that apply in
// `workspace`, in table order, each at most once. Like key chords, typed
// characters fire on key repeat only the actions that repeat.
[[nodiscard]] std::vector<ShortcutAction>
match_shortcuts(const ShortcutTable& table, const KeyFrame& frame, Workspace workspace);

} // namespace orrery::app
