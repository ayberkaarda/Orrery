#include "aero_widgets.h"

#include <algorithm>
#include <charconv>
#include <system_error>

#include "orrery/sim/aero_flow.h"

namespace orrery::app::widgets {

const render::UiTheme& theme() {
    static const render::UiTheme instance = render::default_theme();
    return instance;
}

ImVec4 color(const render::UiColor& c, float alpha) {
    return {c.r, c.g, c.b, c.a * alpha};
}

std::string fixed(double value, int precision) {
    std::array<char, 64> buffer{};
    const auto result = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value,
                                      std::chars_format::fixed, precision);
    return result.ec == std::errc{} ? std::string(buffer.data(), result.ptr) : std::string("-");
}

std::string significant(double value, int digits) {
    return sim::format_significant(value, digits);
}

std::string grouped(std::uint64_t value) {
    std::string digits = std::to_string(value);
    std::string out;
    const std::size_t first = digits.size() % 3 == 0 ? 3 : digits.size() % 3;
    out += digits.substr(0, first);
    for (std::size_t i = first; i < digits.size(); i += 3) {
        out += ' '; // the UI font has neither a thin nor a no-break space
        out += digits.substr(i, 3);
    }
    return out;
}

std::string change(std::size_t before, std::size_t after) {
    return std::to_string(before) + " -> " + std::to_string(after);
}

void text(const std::string& value) {
    ImGui::TextUnformatted(value.c_str());
}

void muted(const std::string& value) {
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::TextUnformatted(value.c_str());
    ImGui::PopStyleColor();
}

void wrapped(const std::string& value, const ImVec4& ink) {
    ImGui::PushStyleColor(ImGuiCol_Text, ink);
    ImGui::PushTextWrapPos(0.0F);
    ImGui::TextUnformatted(value.c_str());
    ImGui::PopTextWrapPos();
    ImGui::PopStyleColor();
}

void warning(const std::string& value) {
    wrapped(value, color(theme().warning));
}

void error(const std::string& value) {
    wrapped(value, ImVec4(0.96F, 0.45F, 0.42F, 1.0F));
}

void tooltip(const char* text_value) {
    if (ImGui::BeginItemTooltip()) {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 28.0F);
        ImGui::TextUnformatted(text_value);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

void section(const char* caption) {
    ImGui::Dummy(ImVec2(0.0F, 2.0F));
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::SeparatorText(caption);
    ImGui::PopStyleColor();
}

namespace {

float control_column() {
    const float longest =
        ImGui::CalcTextSize("Temperature").x + ImGui::GetStyle().ItemSpacing.x * 2.0F;
    const float cap = ImGui::GetStyle().WindowPadding.x + ImGui::GetContentRegionAvail().x * 0.42F;
    return std::max(std::min(ImGui::GetFontSize() * 7.5F, cap),
                    ImGui::GetStyle().WindowPadding.x + longest);
}

// Room kept right of a control for its unit: the widest unit of the panels.
float unit_room() {
    return ImGui::CalcTextSize("kg/m3").x + ImGui::GetStyle().ItemSpacing.x;
}

} // namespace

// The control's column, or further right when the label is longer than the
// column is wide: a control never covers its label.
float control_column_after(const char* label) {
    const float label_end =
        ImGui::GetCursorPosX() + ImGui::CalcTextSize(label).x + ImGui::GetStyle().ItemSpacing.x;
    return std::max(control_column(), label_end);
}

void row_label(const char* label) {
    const float column = control_column_after(label);
    ImGui::AlignTextToFramePadding();
    muted(label);
    ImGui::SameLine(column);
    ImGui::SetNextItemWidth(-1.0F);
}

void row_label_with_unit(const char* label, const char* unit_text) {
    const float column = control_column_after(label);
    ImGui::AlignTextToFramePadding();
    muted(label);
    ImGui::SameLine(column);
    (void)unit_text;
    ImGui::SetNextItemWidth(-unit_room());
}

void unit(const char* unit_text) {
    ImGui::SameLine();
    muted(unit_text);
}

bool begin_properties(const char* id, const char* longest_label) {
    if (!ImGui::BeginTable(id, 2, ImGuiTableFlags_SizingFixedFit)) {
        return false;
    }
    ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::CalcTextSize(longest_label).x + ImGui::GetFontSize() * 1.2F);
    ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch);
    return true;
}

void property(const char* label, const std::string& value) {
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    muted(label);
    ImGui::TableSetColumnIndex(1);
    text(value);
}

void end_properties() {
    ImGui::EndTable();
}

void chip(const std::string& label, const render::UiColor& tint, const char* tooltip_text) {
    const ImGuiStyle& style = ImGui::GetStyle();
    const ImVec2 text_size = ImGui::CalcTextSize(label.c_str());
    const float h = ImGui::GetFrameHeight();
    const float w = text_size.x + style.FramePadding.x * 2.0F;
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    ImGui::Dummy(ImVec2(w, h));
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const float inset = 3.0F;
    draw->AddRectFilled(ImVec2(pos.x, pos.y + inset), ImVec2(pos.x + w, pos.y + h - inset),
                        ImGui::GetColorU32(color(tint, 0.16F)), h * 0.5F);
    draw->AddRect(ImVec2(pos.x, pos.y + inset), ImVec2(pos.x + w, pos.y + h - inset),
                  ImGui::GetColorU32(color(tint, 0.55F)), h * 0.5F, 0, 1.0F);
    draw->AddText(ImVec2(pos.x + style.FramePadding.x, pos.y + (h - text_size.y) * 0.5F),
                  ImGui::GetColorU32(color(tint, 1.0F)), label.c_str());
    if (tooltip_text != nullptr) {
        tooltip(tooltip_text);
    }
}

void chip_after(const std::string& label, const render::UiColor& tint, const char* tooltip_text) {
    ImGui::SameLine();
    chip(label, tint, tooltip_text);
}

bool icon_button(const char* id, Glyph glyph, const char* tooltip_text, bool emphasised,
                 bool enabled) {
    const ImGuiStyle& style = ImGui::GetStyle();
    const float h = ImGui::GetFrameHeight();
    const ImVec2 size(h * 1.3F, h);
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    ImGui::BeginDisabled(!enabled);
    const bool pressed = ImGui::InvisibleButton(id, size);
    ImGui::EndDisabled();
    const bool hovered = ImGui::IsItemHovered();
    const bool active = ImGui::IsItemActive();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    ImGuiCol background = ImGuiCol_Button;
    if (active) {
        background = ImGuiCol_ButtonActive;
    } else if (hovered && enabled) {
        background = ImGuiCol_ButtonHovered;
    }
    draw->AddRectFilled(pos, ImVec2(pos.x + size.x, pos.y + size.y), ImGui::GetColorU32(background),
                        style.FrameRounding);
    ImU32 ink = ImGui::GetColorU32(emphasised ? ImGuiCol_CheckMark : ImGuiCol_Text);
    if (!enabled) {
        ink = ImGui::GetColorU32(ImGuiCol_TextDisabled);
    }
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
        case Glyph::stop:
            draw->AddRectFilled(ImVec2(cx - s * 0.8F, cy - s * 0.8F),
                                ImVec2(cx + s * 0.8F, cy + s * 0.8F), ink, 1.5F);
            break;
    }
    if (tooltip_text != nullptr) {
        ImGui::SetItemTooltip("%s", tooltip_text);
    }
    return pressed && enabled;
}

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

void workspace_label(const char* name, const char* tooltip_text) {
    const ImGuiStyle& style = ImGui::GetStyle();
    const ImVec2 text_size = ImGui::CalcTextSize(name);
    const float h = ImGui::GetFrameHeight();
    const float w = text_size.x + style.FramePadding.x * 3.0F;
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    ImGui::Dummy(ImVec2(w, h));
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImVec2 end(pos.x + w, pos.y + h);
    draw->AddRectFilled(pos, end, ImGui::GetColorU32(ImGuiCol_TabSelected), style.TabRounding);
    draw->AddRectFilled(ImVec2(pos.x + 4.0F, end.y - 2.0F), ImVec2(end.x - 4.0F, end.y),
                        ImGui::GetColorU32(ImGuiCol_TabSelectedOverline));
    draw->AddText(ImVec2(pos.x + (w - text_size.x) * 0.5F, pos.y + (h - text_size.y) * 0.5F),
                  ImGui::GetColorU32(ImGuiCol_Text), name);
    tooltip(tooltip_text);
}

} // namespace orrery::app::widgets
