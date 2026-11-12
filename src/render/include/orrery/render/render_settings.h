#pragma once

#include <cstddef>

#include "orrery/render/body_visuals.h"

// User-facing settings of the space view (Render Settings panel). Plain data:
// the UI returns an edited copy, the renderer reads it.
namespace orrery::render {

struct RenderSettings {
    ScaleMode scale_mode = ScaleMode::clamped;
    double min_radius_scale = 1.0; // multiplies the per-kind minimum screen radii, [0.25, 8]
    int lod_bias = 0;              // shifts the automatic body LOD, [-3, 3]

    bool show_trails = true;
    // Samples per body, [16, 8192]. Recording belongs to the caller: apply it
    // with TrailHistory::set_settings (the renderer draws what is recorded).
    std::size_t trail_length = 512;
    double trail_width_px = 1.6;   // [0.5, 6]
    double trail_brightness = 1.0; // [0, 4]

    bool show_starfield = true;
    double star_brightness = 1.0;      // [0, 4]
    double milky_way_brightness = 1.0; // [0, 4]
    bool star_glow = true;

    double exposure = 1.0; // tone-map exposure, [1/64, 64]
    // Irradiance falls off as (1 AU / d)^exponent from each star, [0, 2].
    // 2 is physical; the default compresses the range so that the outer
    // planets stay visible (docs/rendering.md, approximations).
    double light_falloff_exponent = 0.5;

    friend bool operator==(const RenderSettings&, const RenderSettings&) = default;
};

// Every field clamped into its documented range; NaN replaced by the default.
[[nodiscard]] RenderSettings sanitized(const RenderSettings& settings) noexcept;

} // namespace orrery::render
