#include "orrery/app/shortcuts.h"

#include <algorithm>
#include <cctype>
#include <utility>

namespace orrery::app {

namespace {

constexpr std::array<ShortcutInfo, kShortcutActionCount> kActions{{
    {ShortcutAction::toggle_pause, "toggle_pause", "Run or pause", kInAll, false},
    {ShortcutAction::single_step, "single_step", "Advance one step and pause", kInSpace | kInAero,
     true},
    {ShortcutAction::reset_view, "reset_view", "Frame the view again", kInAll, false},
    {ShortcutAction::warp_down, "warp_down", "Lower the time warp", kInSpace, true},
    {ShortcutAction::warp_up, "warp_up", "Raise the time warp", kInSpace, true},
    {ShortcutAction::reset, "reset", "Reset the scenario to its initial state", kInSpace, false},
    {ShortcutAction::workspace_space, "workspace_space", "Space workspace", kInAll, false},
    {ShortcutAction::workspace_aero, "workspace_aero", "Aero workspace", kInAll, false},
    {ShortcutAction::workspace_entry, "workspace_entry", "Entry workspace", kInAll, false},
    {ShortcutAction::save, "save", "Save the experiment", kInAll, false},
    {ShortcutAction::open, "open", "Show the saved experiments", kInAll, false},
    {ShortcutAction::help, "help", "Show or hide this list", kInAll, false},
    {ShortcutAction::close, "close", "Close this list, or release the selection", kInAll, false},
}};

struct KeySpec {
    Key key = Key::none;
    std::string_view name;
    char plain = '\0';   // character the key types on a US layout
    char shifted = '\0'; // ... with Shift
};

constexpr std::array<KeySpec, kKeyCount - 1> kKeys{{
    {Key::a, "A", 'a', 'A'},
    {Key::b, "B", 'b', 'B'},
    {Key::c, "C", 'c', 'C'},
    {Key::d, "D", 'd', 'D'},
    {Key::e, "E", 'e', 'E'},
    {Key::f, "F", 'f', 'F'},
    {Key::g, "G", 'g', 'G'},
    {Key::h, "H", 'h', 'H'},
    {Key::i, "I", 'i', 'I'},
    {Key::j, "J", 'j', 'J'},
    {Key::k, "K", 'k', 'K'},
    {Key::l, "L", 'l', 'L'},
    {Key::m, "M", 'm', 'M'},
    {Key::n, "N", 'n', 'N'},
    {Key::o, "O", 'o', 'O'},
    {Key::p, "P", 'p', 'P'},
    {Key::q, "Q", 'q', 'Q'},
    {Key::r, "R", 'r', 'R'},
    {Key::s, "S", 's', 'S'},
    {Key::t, "T", 't', 'T'},
    {Key::u, "U", 'u', 'U'},
    {Key::v, "V", 'v', 'V'},
    {Key::w, "W", 'w', 'W'},
    {Key::x, "X", 'x', 'X'},
    {Key::y, "Y", 'y', 'Y'},
    {Key::z, "Z", 'z', 'Z'},
    {Key::d0, "0", '0', ')'},
    {Key::d1, "1", '1', '!'},
    {Key::d2, "2", '2', '@'},
    {Key::d3, "3", '3', '#'},
    {Key::d4, "4", '4', '$'},
    {Key::d5, "5", '5', '%'},
    {Key::d6, "6", '6', '^'},
    {Key::d7, "7", '7', '&'},
    {Key::d8, "8", '8', '*'},
    {Key::d9, "9", '9', '('},
    {Key::f1, "F1"},
    {Key::f2, "F2"},
    {Key::f3, "F3"},
    {Key::f4, "F4"},
    {Key::f5, "F5"},
    {Key::f6, "F6"},
    {Key::f7, "F7"},
    {Key::f8, "F8"},
    {Key::f9, "F9"},
    {Key::f10, "F10"},
    {Key::f11, "F11"},
    {Key::f12, "F12"},
    {Key::space, "Space", ' ', ' '},
    {Key::escape, "Esc"},
    {Key::enter, "Enter"},
    {Key::tab, "Tab"},
    {Key::backspace, "Backspace"},
    {Key::del, "Delete"},
    {Key::insert, "Insert"},
    {Key::home, "Home"},
    {Key::end, "End"},
    {Key::page_up, "PageUp"},
    {Key::page_down, "PageDown"},
    {Key::left, "Left"},
    {Key::right, "Right"},
    {Key::up, "Up"},
    {Key::down, "Down"},
    {Key::period, ".", '.', '>'},
    {Key::comma, ",", ',', '<'},
    {Key::slash, "/", '/', '?'},
    {Key::minus, "-", '-', '_'},
    {Key::equal, "=", '=', '+'},
    {Key::left_bracket, "[", '[', '{'},
    {Key::right_bracket, "]", ']', '}'},
    {Key::semicolon, ";", ';', ':'},
    {Key::apostrophe, "'", '\'', '"'},
    {Key::backslash, "\\", '\\', '|'},
    {Key::grave, "`", '`', '~'},
}};

std::string lower(std::string_view text) {
    std::string out(text);
    std::transform(out.begin(), out.end(), out.begin(), [](char c) {
        return static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    });
    return out;
}

const KeySpec* spec_of(Key key) noexcept {
    for (const KeySpec& spec : kKeys) {
        if (spec.key == key) {
            return &spec;
        }
    }
    return nullptr;
}

// A key by name, case-insensitive, with a few aliases.
std::optional<Key> key_from_name(std::string_view name) {
    const std::string wanted = lower(name);
    if (wanted == "escape") {
        return Key::escape;
    }
    if (wanted == "return") {
        return Key::enter;
    }
    if (wanted == "del") {
        return Key::del;
    }
    for (const KeySpec& spec : kKeys) {
        if (lower(spec.name) == wanted) {
            return spec.key;
        }
    }
    return std::nullopt;
}

bool printable_punctuation(char c) {
    const auto u = static_cast<unsigned char>(c);
    return std::ispunct(u) != 0;
}

// The key that types `c` unshifted, if any.
std::optional<Key> key_typing(char c) {
    for (const KeySpec& spec : kKeys) {
        if (spec.plain == c && printable_punctuation(c)) {
            return spec.key;
        }
    }
    return std::nullopt;
}

// Character a key chord types on a US layout ('\0': none, or a modifier
// other than Shift is held).
char typed_by(const KeyChord& chord) {
    if (chord.key == Key::none || chord.ctrl || chord.alt) {
        return '\0';
    }
    const KeySpec* spec = spec_of(chord.key);
    if (spec == nullptr) {
        return '\0';
    }
    return chord.shift ? spec->shifted : spec->plain;
}

// Whether a key chord can type a character on some layout that a US layout
// does not tell: Alt on a key that types (AltGr is Ctrl+Alt on Windows, and
// Option is Alt on macOS) types layout-dependent characters, e.g. "[" for
// AltGr+8 on a German layout.
bool may_type_any(const KeyChord& chord) {
    const KeySpec* spec = chord.character == '\0' ? spec_of(chord.key) : nullptr;
    return chord.alt && spec != nullptr && spec->plain != '\0' && spec->plain != ' ';
}

// A character chord and a key chord that can type it.
bool types_character(const KeyChord& key, char character) {
    return typed_by(key) == character || may_type_any(key);
}

bool chords_collide(const KeyChord& a, const KeyChord& b) {
    if (a == b) {
        return true;
    }
    if (a.character != '\0' && b.character == '\0') {
        return types_character(b, a.character);
    }
    if (b.character != '\0' && a.character == '\0') {
        return types_character(a, b.character);
    }
    return false;
}

std::string chord_list(const std::vector<KeyChord>& chords) {
    std::string out;
    for (const KeyChord& chord : chords) {
        if (!out.empty()) {
            out += ", ";
        }
        out += format_key_chord(chord);
    }
    return out;
}

} // namespace

std::uint8_t workspace_bit(Workspace workspace) noexcept {
    switch (workspace) {
        case Workspace::space:
            return kInSpace;
        case Workspace::aero:
            return kInAero;
        case Workspace::entry:
            return kInEntry;
    }
    return kInSpace;
}

std::span<const ShortcutInfo> shortcut_actions() noexcept {
    return kActions;
}

const ShortcutInfo& shortcut_info(ShortcutAction action) noexcept {
    return kActions[static_cast<std::size_t>(action)];
}

std::optional<ShortcutAction> shortcut_action_from_name(std::string_view name) noexcept {
    for (const ShortcutInfo& info : kActions) {
        if (info.name == name) {
            return info.action;
        }
    }
    return std::nullopt;
}

std::string_view key_name(Key key) noexcept {
    const KeySpec* spec = spec_of(key);
    return spec != nullptr ? spec->name : std::string_view("?");
}

core::Result<KeyChord, std::string> parse_key_chord(std::string_view text) {
    if (text.empty()) {
        return core::failure(std::string("an empty key chord"));
    }
    KeyChord chord;
    std::string_view rest = text;
    // Modifiers: every '+'-separated part but the last (a trailing "++" is a
    // plus key after the modifiers).
    while (rest.size() > 1) {
        const std::size_t plus = rest.find('+');
        if (plus == std::string_view::npos || plus + 1 >= rest.size()) {
            break;
        }
        const std::string modifier = lower(rest.substr(0, plus));
        if (modifier == "ctrl" || modifier == "control") {
            chord.ctrl = true;
        } else if (modifier == "shift") {
            chord.shift = true;
        } else if (modifier == "alt") {
            chord.alt = true;
        } else {
            return core::failure("'" + std::string(rest.substr(0, plus)) +
                                 "' is not a modifier (Ctrl, Shift or Alt)");
        }
        rest = rest.substr(plus + 1);
    }
    const bool modified = chord.ctrl || chord.shift || chord.alt;
    if (rest.size() == 1 && printable_punctuation(rest.front())) {
        if (!modified) {
            chord.character = rest.front();
            return chord;
        }
        if (const auto key = key_typing(rest.front())) {
            chord.key = *key;
            return chord;
        }
        return core::failure("'" + std::string(rest) +
                             "' has no key of its own; name the key with its modifiers, "
                             "e.g. Shift+/ for ?");
    }
    const auto key = key_from_name(rest);
    if (!key) {
        return core::failure("'" + std::string(rest) + "' is not a key name");
    }
    chord.key = *key;
    return chord;
}

std::string format_key_chord(const KeyChord& chord) {
    std::string out;
    if (chord.ctrl) {
        out += "Ctrl+";
    }
    if (chord.shift) {
        out += "Shift+";
    }
    if (chord.alt) {
        out += "Alt+";
    }
    if (chord.character != '\0') {
        out += chord.character;
    } else {
        out += key_name(chord.key);
    }
    return out;
}

const std::vector<KeyChord>& ShortcutTable::of(ShortcutAction action) const noexcept {
    return bindings[static_cast<std::size_t>(action)];
}

std::vector<KeyChord>& ShortcutTable::of(ShortcutAction action) noexcept {
    return bindings[static_cast<std::size_t>(action)];
}

ShortcutTable default_shortcuts() {
    const auto key = [](Key k, bool ctrl = false) {
        KeyChord chord;
        chord.key = k;
        chord.ctrl = ctrl;
        return chord;
    };
    const auto typed = [](char c) {
        KeyChord chord;
        chord.character = c;
        return chord;
    };
    ShortcutTable table;
    table.of(ShortcutAction::toggle_pause) = {key(Key::space)};
    table.of(ShortcutAction::single_step) = {typed('.'), key(Key::right)};
    table.of(ShortcutAction::reset_view) = {key(Key::f), key(Key::home)};
    table.of(ShortcutAction::warp_down) = {typed('[')};
    table.of(ShortcutAction::warp_up) = {typed(']')};
    table.of(ShortcutAction::reset) = {key(Key::r, true)};
    table.of(ShortcutAction::workspace_space) = {key(Key::d1, true)};
    table.of(ShortcutAction::workspace_aero) = {key(Key::d2, true)};
    table.of(ShortcutAction::workspace_entry) = {key(Key::d3, true)};
    table.of(ShortcutAction::save) = {key(Key::s, true)};
    table.of(ShortcutAction::open) = {key(Key::o, true)};
    table.of(ShortcutAction::help) = {key(Key::f1), typed('?')};
    table.of(ShortcutAction::close) = {key(Key::escape)};
    return table;
}

std::vector<ShortcutConflict> find_conflicts(const ShortcutTable& table) {
    std::vector<ShortcutConflict> conflicts;
    for (std::size_t a = 0; a < kShortcutActionCount; ++a) {
        for (std::size_t b = a + 1; b < kShortcutActionCount; ++b) {
            if ((kActions[a].workspaces & kActions[b].workspaces) == 0U) {
                continue;
            }
            for (const KeyChord& first : table.bindings[a]) {
                const auto clash = std::find_if(
                    table.bindings[b].begin(), table.bindings[b].end(),
                    [&](const KeyChord& second) { return chords_collide(first, second); });
                if (clash != table.bindings[b].end()) {
                    conflicts.push_back({kActions[a].action, kActions[b].action, first});
                }
            }
        }
    }
    return conflicts;
}

std::string describe(const ShortcutConflict& conflict) {
    return format_key_chord(conflict.chord) + " is bound to both " +
           std::string(shortcut_info(conflict.first).name) + " and " +
           std::string(shortcut_info(conflict.second).name);
}

std::vector<std::string> apply_shortcut_overrides(ShortcutTable& table,
                                                  std::span<const ShortcutOverride> overrides) {
    const ShortcutTable defaults = default_shortcuts();
    std::array<const ShortcutOverride*, kShortcutActionCount> applied{};
    for (const ShortcutOverride& override_entry : overrides) {
        table.of(override_entry.action) = override_entry.chords;
        applied[static_cast<std::size_t>(override_entry.action)] = &override_entry;
    }
    // Revert one user binding of the first conflict per round until none is
    // left: every round reverts one, and the defaults are conflict-free.
    std::vector<std::string> warnings;
    for (std::size_t round = 0; round <= kShortcutActionCount; ++round) {
        const std::vector<ShortcutConflict> conflicts = find_conflicts(table);
        if (conflicts.empty()) {
            break;
        }
        const ShortcutConflict& conflict = conflicts.front();
        const auto later = static_cast<std::size_t>(conflict.second);
        const ShortcutAction action = applied[later] != nullptr ? conflict.second : conflict.first;
        const auto index = static_cast<std::size_t>(action);
        if (applied[index] == nullptr) {
            break; // two defaults: cannot happen (tested)
        }
        std::string warning = applied[index]->where;
        warning += ": shortcuts.";
        warning += shortcut_info(action).name;
        warning += ": ";
        warning += describe(conflict);
        warning += "; keeping ";
        warning += chord_list(defaults.of(action));
        warnings.push_back(std::move(warning));
        table.of(action) = defaults.of(action);
        applied[index] = nullptr;
    }
    return warnings;
}

namespace {

bool contains_chord(const std::vector<KeyChord>& keys, const KeyChord& chord) {
    return std::find(keys.begin(), keys.end(), chord) != keys.end();
}

// Whether a key chord of an action of `bit` is down: that key event belongs
// to the action, the characters it types included (on its own repeats too,
// whose timing is the system's, not Dear ImGui's).
bool keys_claimed(const ShortcutTable& table, const KeyFrame& frame, std::uint8_t bit) {
    return std::any_of(kActions.begin(), kActions.end(), [&](const ShortcutInfo& info) {
        const std::vector<KeyChord>& chords = table.of(info.action);
        return (info.workspaces & bit) != 0U &&
               std::any_of(chords.begin(), chords.end(), [&](const KeyChord& chord) {
                   return chord.character == '\0' && (contains_chord(frame.pressed, chord) ||
                                                      contains_chord(frame.repeated, chord) ||
                                                      contains_chord(frame.held, chord));
               });
    });
}

bool chord_fired(const KeyChord& chord, const KeyFrame& frame, bool repeats, bool claimed) {
    if (chord.character != '\0') {
        return !claimed &&
               (frame.typed.find(chord.character) != std::string::npos ||
                (repeats && frame.typed_repeated.find(chord.character) != std::string::npos));
    }
    return contains_chord(frame.pressed, chord) ||
           (repeats && contains_chord(frame.repeated, chord));
}

} // namespace

std::vector<ShortcutAction> match_shortcuts(const ShortcutTable& table, const KeyFrame& frame,
                                            Workspace workspace) {
    std::vector<ShortcutAction> actions;
    if (frame.text_input) {
        return actions;
    }
    const std::uint8_t bit = workspace_bit(workspace);
    // One key event, one action: a layout may type a bound character with a
    // bound key chord (AltGr+8 types "[" on a German layout), and the key
    // chord, the more specific binding, wins.
    const bool claimed = keys_claimed(table, frame, bit);
    for (const ShortcutInfo& info : kActions) {
        const std::vector<KeyChord>& chords = table.of(info.action);
        if ((info.workspaces & bit) != 0U &&
            std::any_of(chords.begin(), chords.end(), [&](const KeyChord& chord) {
                return chord_fired(chord, frame, info.repeats, claimed);
            })) {
            actions.push_back(info.action);
        }
    }
    return actions;
}

} // namespace orrery::app
