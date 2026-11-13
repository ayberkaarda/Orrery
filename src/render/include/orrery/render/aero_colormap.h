#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include <glm/ext/vector_double3.hpp>

// Colour maps, scales and legends of the aero visualisation
// (docs/aero-visualisation.md, "Colour maps"). Plain double-precision math,
// no OpenGL; the GPU uses tables made by colormap_table().
//
// Both maps are defined in Oklab (Ottosson 2020), whose lightness is close to
// perceptually uniform, with lightness linear in the map parameter:
//   diverging   blue (hue 255 deg) -> neutral light grey -> red-orange
//               (hue 40 deg); lightness 0.48 at both ends, 0.94 at the centre,
//               chroma 0 at the centre growing to 0.13 at the ends. The two
//               arms differ along the blue-yellow axis, which deuteranopes and
//               protanopes keep, and have equal lightness at equal distance
//               from the centre, so neither sign looks stronger.
//   sequential  dark blue-violet -> teal -> pale yellow, lightness 0.30 to
//               0.94 (for magnitudes: speed, |omega|, Q).
// The signed quantities (Cp, vorticity, pressure) use the diverging map with
// zero at its centre; negative values (suction) are blue.
namespace orrery::render {

enum class Colormap : std::uint8_t { diverging, sequential };

// Linear-light sRGB of the map at t in [0, 1] (clamped; NaN gives the centre
// of the diverging map and the start of the sequential one). Colours that
// fall outside the sRGB gamut are reduced in chroma at constant lightness
// and hue.
[[nodiscard]] glm::dvec3 colormap_linear(Colormap map, double t) noexcept;
// Same, sRGB-encoded ([0, 1], the display colour).
[[nodiscard]] glm::dvec3 colormap_srgb(Colormap map, double t) noexcept;
// `count` (>= 2) evenly spaced linear-light samples from t = 0 to t = 1.
[[nodiscard]] std::vector<glm::dvec3> colormap_table(Colormap map, std::size_t count);

// Oklab of linear-light sRGB and back (L in [0, 1] for colours in gamut).
[[nodiscard]] glm::dvec3 oklab_from_linear_srgb(const glm::dvec3& rgb) noexcept;
[[nodiscard]] glm::dvec3 linear_srgb_from_oklab(const glm::dvec3& lab) noexcept;
// CIE 1976 L* (D65 white, 0 .. 100) of a linear-light sRGB colour.
[[nodiscard]] double cie_lightness(const glm::dvec3& rgb) noexcept;

// Scene-linear colour that the HDR pipeline's display transform (exposure,
// ACES fit; tone_map.h) turns into the linear display colour `display`
// (components in [0, 0.95]; larger ones are clamped there, where the ACES
// fit still has an inverse). Colour-mapped geometry is drawn with these
// values, so that a surface facing the key light shows the colour of the
// legend exactly (within 8-bit rounding).
[[nodiscard]] glm::dvec3 scene_linear_for_display(const glm::dvec3& display,
                                                  double exposure) noexcept;

// How a colour scale chooses its range.
//   automatic  [min, max] of the data
//   symmetric  [-m, m] with m = max |value| (signed data, zero at the centre)
//   fixed      [ColorScale::min, ColorScale::max]
enum class ScaleMode : std::uint8_t { automatic, symmetric, fixed };

struct ColorScale {
    ScaleMode mode = ScaleMode::automatic;
    double min = 0.0; // fixed mode only
    double max = 1.0;
    // Widen an automatic or symmetric range to the next tick of a 1-2-2.5-5
    // sequence, so that the legend ends on round numbers.
    bool round_to_ticks = true;
};

struct ScalarRange {
    double min = 0.0;
    double max = 1.0;

    // (value - min) / (max - min) clamped to [0, 1]; 0.5 for an empty range.
    [[nodiscard]] double normalize(double value) const noexcept;
    friend constexpr bool operator==(const ScalarRange&, const ScalarRange&) = default;
};

// The range of `scale` for `values` (non-finite values are ignored; no
// finite value gives [-1, 1] for symmetric and [0, 1] otherwise). A fixed
// range with min >= max or non-finite ends is replaced by the automatic one.
[[nodiscard]] ScalarRange resolve_range(const ColorScale& scale, std::span<const double> values);

struct LegendTick {
    double value = 0.0;
    double position = 0.0; // along the bar, 0 = range.min, 1 = range.max
    std::string label;
};

// Colour bar description: what the overlay draws, and what a user interface
// draws itself when it shows the legend in its own widgets.
struct Legend {
    std::string title; // quantity and unit, "Cp", "|u| [m/s]"
    Colormap colormap = Colormap::diverging;
    ScalarRange range;
    std::vector<LegendTick> ticks;
};

// Ticks at multiples of a 1-2-2.5-5 step chosen so that about `target` (>= 2)
// ticks fall inside the range, both ends included when they are multiples.
// Labels use the fewest decimals that tell the ticks apart, a minus sign for
// negative values and scientific notation ("2e-4") outside [1e-3, 1e5).
[[nodiscard]] std::vector<LegendTick> legend_ticks(const ScalarRange& range, int target = 5);
[[nodiscard]] Legend make_legend(std::string title, Colormap map, const ScalarRange& range,
                                 int target_ticks = 5);

} // namespace orrery::render
