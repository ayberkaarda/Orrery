#include "orrery/render/ui_theme.h"

#include <algorithm>
#include <cmath>

namespace orrery::render {

namespace {

constexpr UiColor rgb(int r, int g, int b, float a = 1.0F) noexcept {
    return {static_cast<float>(r) / 255.0F, static_cast<float>(g) / 255.0F,
            static_cast<float>(b) / 255.0F, a};
}

double linear_channel(float encoded) noexcept {
    const double c = std::clamp(static_cast<double>(encoded), 0.0, 1.0);
    return c <= 0.04045 ? c / 12.92 : std::pow((c + 0.055) / 1.055, 2.4);
}

} // namespace

UiTheme default_theme() noexcept {
    UiTheme t;
    t.title = rgb(11, 13, 17);
    t.window = rgb(16, 19, 24, 0.97F);
    t.inset = rgb(12, 14, 18);
    t.control = rgb(27, 31, 39);
    t.control_hover = rgb(36, 41, 51);
    t.control_active = rgb(45, 51, 63);
    t.border = rgb(38, 43, 53);
    t.separator = rgb(34, 39, 48);
    t.text = rgb(218, 222, 229);
    t.text_muted = rgb(142, 150, 164);
    t.text_disabled = rgb(96, 104, 117);
    t.accent = rgb(226, 167, 92);
    t.accent_hover = rgb(236, 186, 122);
    t.accent_active = rgb(204, 146, 72);
    t.accent_secondary = rgb(116, 170, 204);
    t.warning = rgb(229, 115, 96);
    return t;
}

double relative_luminance(const UiColor& color) noexcept {
    return 0.2126 * linear_channel(color.r) + 0.7152 * linear_channel(color.g) +
           0.0722 * linear_channel(color.b);
}

double contrast_ratio(const UiColor& a, const UiColor& b) noexcept {
    const double la = relative_luminance(a);
    const double lb = relative_luminance(b);
    return (std::max(la, lb) + 0.05) / (std::min(la, lb) + 0.05);
}

UiColor composite(const UiColor& over, const UiColor& under) noexcept {
    const float a = std::clamp(over.a, 0.0F, 1.0F);
    return {over.r * a + under.r * (1.0F - a), over.g * a + under.g * (1.0F - a),
            over.b * a + under.b * (1.0F - a), 1.0F};
}

} // namespace orrery::render
