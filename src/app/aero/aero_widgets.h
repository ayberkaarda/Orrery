#pragma once

// Small Dear ImGui helpers shared by the Aero workspace's bar and panels, in
// the look of the space workspace (render/ui_panels.cpp, ui_app_bar.cpp):
// muted captions, label/control rows, property tables, chips. Private to
// orrery_app_core.

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

#include <imgui.h>

#include "orrery/render/ui_theme.h"

namespace orrery::app::widgets {

[[nodiscard]] const render::UiTheme& theme();
[[nodiscard]] ImVec4 color(const render::UiColor& c, float alpha = 1.0F);

// Number formatting without printf format strings.
[[nodiscard]] std::string fixed(double value, int precision);
// `digits` significant digits, scientific outside [1e-3, 1e6).
[[nodiscard]] std::string significant(double value, int digits = 4);
// Thousands separated with spaces ("36 000"; the UI font has neither a thin
// nor a no-break space, which it would draw as "?").
[[nodiscard]] std::string grouped(std::uint64_t value);
// A count before and after a change: "12 -> 0".
[[nodiscard]] std::string change(std::size_t before, std::size_t after);

void text(const std::string& value);
void muted(const std::string& value);
void wrapped(const std::string& value, const ImVec4& color);
void warning(const std::string& value);
void error(const std::string& value);
void tooltip(const char* text);

// Small upper-case section caption with a hairline under it.
void section(const char* caption);

// Label on the left, the next control filling the rest of the row.
void row_label(const char* label);
// The same with the control `unit_width` short of the right edge, for a
// unit drawn after it with unit().
void row_label_with_unit(const char* label, const char* unit);
void unit(const char* unit);

// Two-column property table (muted label, value).
bool begin_properties(const char* id, const char* longest_label);
void property(const char* label, const std::string& value);
void end_properties();

// Rounded label on a faint background of `tint` (warning chip, badges).
void chip(const std::string& label, const render::UiColor& tint, const char* tooltip_text);
// A chip after the previous item on its line.
void chip_after(const std::string& label, const render::UiColor& tint, const char* tooltip_text);

// Square-ish button with a vector glyph (the default font has no icons).
enum class Glyph : std::uint8_t { play, pause, step, stop };
bool icon_button(const char* id, Glyph glyph, const char* tooltip_text, bool emphasised,
                 bool enabled = true);

// The orbit mark and word mark at the start of a main bar.
void word_mark();
// The selected-workspace tab of a main bar (decoration only).
void workspace_label(const char* name, const char* tooltip_text);

// Fixed-size text buffers for InputText.
template <std::size_t N> void set_buffer(std::array<char, N>& buffer, std::string_view value) {
    const std::size_t count = value.size() < N - 1 ? value.size() : N - 1;
    for (std::size_t i = 0; i < count; ++i) {
        buffer[i] = value[i];
    }
    buffer[count] = '\0';
    for (std::size_t i = count + 1; i < N; ++i) {
        buffer[i] = '\0';
    }
}

template <std::size_t N> [[nodiscard]] std::string from_buffer(const std::array<char, N>& buffer) {
    std::size_t length = 0;
    while (length < N && buffer[length] != '\0') {
        ++length;
    }
    return {buffer.data(), length};
}

} // namespace orrery::app::widgets
