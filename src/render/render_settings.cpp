#include "orrery/render/render_settings.h"

#include <algorithm>
#include <cmath>

namespace orrery::render {

namespace {

double clamp_or(double value, double lo, double hi, double fallback) noexcept {
    return std::isnan(value) ? fallback : std::clamp(value, lo, hi);
}

} // namespace

RenderSettings sanitized(const RenderSettings& settings) noexcept {
    const RenderSettings defaults;
    RenderSettings s = settings;
    if (s.scale_mode != ScaleMode::true_scale && s.scale_mode != ScaleMode::clamped) {
        s.scale_mode = defaults.scale_mode;
    }
    s.min_radius_scale = clamp_or(s.min_radius_scale, 0.25, 8.0, defaults.min_radius_scale);
    s.lod_bias = std::clamp(s.lod_bias, -3, 3);
    s.trail_length = std::clamp<std::size_t>(s.trail_length, 16, 8192);
    s.trail_width_px = clamp_or(s.trail_width_px, 0.5, 6.0, defaults.trail_width_px);
    s.trail_brightness = clamp_or(s.trail_brightness, 0.0, 4.0, defaults.trail_brightness);
    s.star_brightness = clamp_or(s.star_brightness, 0.0, 4.0, defaults.star_brightness);
    s.milky_way_brightness =
        clamp_or(s.milky_way_brightness, 0.0, 4.0, defaults.milky_way_brightness);
    s.exposure = clamp_or(s.exposure, 1.0 / 64.0, 64.0, defaults.exposure);
    s.light_falloff_exponent =
        clamp_or(s.light_falloff_exponent, 0.0, 2.0, defaults.light_falloff_exponent);
    return s;
}

} // namespace orrery::render
