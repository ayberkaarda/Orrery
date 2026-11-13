#include "orrery/render/aero_colormap.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <utility>

#include <glm/common.hpp>

namespace orrery::render {

namespace {

constexpr double kPi = 3.141592653589793;
constexpr double kDegree = kPi / 180.0;

// Map definitions: CIE L* (linear in the map parameter), Oklab chroma and
// hue (docs/aero-visualisation.md).
constexpr double kDivergingCentreL = 95.0;
constexpr double kDivergingEndL = 45.0;
constexpr double kDivergingEndChroma = 0.15;
constexpr double kDivergingNegativeHue = 255.0 * kDegree; // blue
constexpr double kDivergingPositiveHue = 38.0 * kDegree;  // red-orange

constexpr double kSequentialStartL = 22.0;
constexpr double kSequentialEndL = 94.0;
constexpr double kSequentialStartChroma = 0.10;
constexpr double kSequentialEndChroma = 0.16;
constexpr double kSequentialStartHue = 290.0 * kDegree; // blue-violet
constexpr double kSequentialEndHue = 105.0 * kDegree;   // pale yellow

// Largest display value the inverse display transform accepts.
constexpr double kMaxDisplay = 0.95;

double clamp_parameter(double t) noexcept {
    return std::isnan(t) ? std::numeric_limits<double>::quiet_NaN() : std::clamp(t, 0.0, 1.0);
}

// Hill's ACES fit (tone_map.cpp), rows of the published matrices.
constexpr std::array<std::array<double, 3>, 3> kAcesInput{{
    {0.59719, 0.35458, 0.04823},
    {0.07600, 0.90834, 0.01566},
    {0.02840, 0.13383, 0.83777},
}};
constexpr std::array<std::array<double, 3>, 3> kAcesOutput{{
    {1.60475, -0.53108, -0.07367},
    {-0.10208, 1.10813, -0.00605},
    {-0.00327, -0.07276, 1.07602},
}};

using Matrix3 = std::array<std::array<double, 3>, 3>;

glm::dvec3 multiply(const Matrix3& m, const glm::dvec3& v) noexcept {
    return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
}

Matrix3 inverse(const Matrix3& m) noexcept {
    const double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                       m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                       m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    const double k = 1.0 / det;
    Matrix3 r{};
    r[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * k;
    r[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * k;
    r[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * k;
    r[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * k;
    r[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * k;
    r[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * k;
    r[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * k;
    r[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * k;
    r[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * k;
    return r;
}

// Inverse of the rational curve (a v^2 + b v + c) / (d v^2 + e v + f) = y,
// the non-negative root of (1 - 0.983729 y) v^2 + (0.0245786 - 0.4329510 y) v
// - (0.000090537 + 0.238081 y) = 0, for y in [0, 1).
double inverse_curve(double y) noexcept {
    const double target = std::clamp(y, 0.0, 1.0);
    const double a = 1.0 - 0.983729 * target;
    const double b = 0.0245786 - 0.4329510 * target;
    const double c = -(0.000090537 + 0.238081 * target);
    const double disc = std::max(b * b - 4.0 * a * c, 0.0);
    // Stable form of the positive root: 2c / (-b - sqrt(disc)) (c <= 0).
    const double denominator = -b - std::sqrt(disc);
    return denominator < 0.0 ? std::max(2.0 * c / denominator, 0.0) : 0.0;
}

const Matrix3& output_inverse() {
    static const Matrix3 m = inverse(kAcesOutput);
    return m;
}
const Matrix3& input_inverse() {
    static const Matrix3 m = inverse(kAcesInput);
    return m;
}

// A display colour the HDR pipeline can show: inside the sRGB gamut, no
// component above kMaxDisplay, and with a non-negative scene-linear
// pre-image under the ACES fit (the fit desaturates, so the most saturated
// sRGB colours have none).
bool in_gamut(const glm::dvec3& rgb) noexcept {
    constexpr double kTolerance = 1.0e-9;
    if (!(rgb.x >= -kTolerance && rgb.y >= -kTolerance && rgb.z >= -kTolerance &&
          rgb.x <= kMaxDisplay && rgb.y <= kMaxDisplay && rgb.z <= kMaxDisplay)) {
        return false;
    }
    const glm::dvec3 curve = multiply(output_inverse(), rgb);
    if (!(curve.x >= -kTolerance && curve.y >= -kTolerance && curve.z >= -kTolerance &&
          curve.x < 1.0 && curve.y < 1.0 && curve.z < 1.0)) {
        return false;
    }
    const glm::dvec3 v{inverse_curve(curve.x), inverse_curve(curve.y), inverse_curve(curve.z)};
    const glm::dvec3 linear = multiply(input_inverse(), v);
    return linear.x >= -kTolerance && linear.y >= -kTolerance && linear.z >= -kTolerance;
}

// Linear sRGB of (L, C, h); the chroma is reduced at constant L and h until
// the colour is displayable (in_gamut, bisection).
glm::dvec3 from_lch(double lightness, double chroma, double hue) noexcept {
    const auto at = [&](double c) {
        return linear_srgb_from_oklab({lightness, c * std::cos(hue), c * std::sin(hue)});
    };
    glm::dvec3 rgb = at(chroma);
    if (in_gamut(rgb)) {
        return glm::clamp(rgb, glm::dvec3(0.0), glm::dvec3(kMaxDisplay));
    }
    double low = 0.0;
    double high = chroma;
    for (int i = 0; i < 40; ++i) {
        const double mid = 0.5 * (low + high);
        if (in_gamut(at(mid))) {
            low = mid;
        } else {
            high = mid;
        }
    }
    return glm::clamp(at(low), glm::dvec3(0.0), glm::dvec3(kMaxDisplay));
}

// Linear sRGB of CIE lightness `lstar` with Oklab chroma and hue: the Oklab
// lightness is found by bisection (L* grows with it at fixed chroma and hue).
glm::dvec3 from_lstar(double lstar, double chroma, double hue) noexcept {
    double low = 0.0;
    double high = 1.0;
    for (int i = 0; i < 48; ++i) {
        const double mid = 0.5 * (low + high);
        if (cie_lightness(from_lch(mid, chroma, hue)) < lstar) {
            low = mid;
        } else {
            high = mid;
        }
    }
    return from_lch(0.5 * (low + high), chroma, hue);
}

// Ticks: multiples of a 1-2-2.5-5 step.
constexpr std::array<double, 4> kMantissas{1.0, 2.0, 2.5, 5.0};

// Nice steps in ascending order, starting a decade below `around`.
std::vector<double> candidate_steps(double around) {
    std::vector<double> steps;
    const double decade = std::pow(10.0, std::floor(std::log10(around)) - 1.0);
    steps.reserve(12);
    for (int e = 0; e < 3; ++e) {
        for (const double m : kMantissas) {
            steps.push_back(m * decade * std::pow(10.0, e));
        }
    }
    return steps;
}

double snap_down(double value, double step) noexcept {
    return std::floor(value / step + 1.0e-9) * step;
}
double snap_up(double value, double step) noexcept {
    return std::ceil(value / step - 1.0e-9) * step;
}

// Fewest decimals (<= 6) that print `step` exactly.
int decimals_of(double step) noexcept {
    for (int d = 0; d < 6; ++d) {
        const double scaled = step * std::pow(10.0, d);
        if (std::abs(scaled - std::round(scaled)) < 1.0e-6 * std::max(1.0, scaled)) {
            return d;
        }
    }
    return 6;
}

std::string format_value(double value, double step, double magnitude) {
    std::array<char, 48> text{};
    if (std::abs(value) < 0.5 * step) {
        value = 0.0; // no "-0"
    }
    if (magnitude >= 1.0e5 || step < 1.0e-3) {
        if (value == 0.0) {
            return "0";
        }
        // One exponent for every tick: that of the largest magnitude.
        const int exponent = static_cast<int>(std::floor(std::log10(magnitude)));
        const double mantissa = value / std::pow(10.0, exponent);
        const int digits = std::min(decimals_of(step / std::pow(10.0, exponent)), 3);
        (void)std::snprintf(text.data(), text.size(), "%.*fe%d", digits, mantissa, exponent);
        return text.data();
    }
    (void)std::snprintf(text.data(), text.size(), "%.*f", decimals_of(step), value);
    return text.data();
}

struct TickPlan {
    double step = 1.0;
    double low = 0.0;
    double high = 1.0;
};

// The first nice step whose snapped range holds at most target + 2 ticks.
TickPlan plan_ticks(double low, double high, int target, bool widen) {
    const double span = high - low;
    const auto limit = static_cast<double>(std::max(target, 2) + 2);
    TickPlan plan;
    for (const double step : candidate_steps(span / static_cast<double>(std::max(target, 2)))) {
        const double a = widen ? snap_down(low, step) : snap_up(low, step);
        const double b = widen ? snap_up(high, step) : snap_down(high, step);
        const double count = std::round((b - a) / step) + 1.0;
        plan = {step, a, b};
        if (count <= limit) {
            break;
        }
    }
    return plan;
}

} // namespace

glm::dvec3 oklab_from_linear_srgb(const glm::dvec3& rgb) noexcept {
    const double l = 0.4122214708 * rgb.x + 0.5363325363 * rgb.y + 0.0514459929 * rgb.z;
    const double m = 0.2119034982 * rgb.x + 0.6806995451 * rgb.y + 0.1073969566 * rgb.z;
    const double s = 0.0883024619 * rgb.x + 0.2817188376 * rgb.y + 0.6299787005 * rgb.z;
    const double l3 = std::cbrt(l);
    const double m3 = std::cbrt(m);
    const double s3 = std::cbrt(s);
    return {0.2104542553 * l3 + 0.7936177850 * m3 - 0.0040720468 * s3,
            1.9779984951 * l3 - 2.4285922050 * m3 + 0.4505937099 * s3,
            0.0259040371 * l3 + 0.7827717662 * m3 - 0.8086757660 * s3};
}

glm::dvec3 linear_srgb_from_oklab(const glm::dvec3& lab) noexcept {
    const double l3 = lab.x + 0.3963377774 * lab.y + 0.2158037573 * lab.z;
    const double m3 = lab.x - 0.1055613458 * lab.y - 0.0638541728 * lab.z;
    const double s3 = lab.x - 0.0894841775 * lab.y - 1.2914855480 * lab.z;
    const double l = l3 * l3 * l3;
    const double m = m3 * m3 * m3;
    const double s = s3 * s3 * s3;
    return {4.0767416621 * l - 3.3077115913 * m + 0.2309699292 * s,
            -1.2684380046 * l + 2.6097574011 * m - 0.3413193965 * s,
            -0.0041960863 * l - 0.7034186147 * m + 1.7076147010 * s};
}

double cie_lightness(const glm::dvec3& rgb) noexcept {
    // Relative luminance Y (D65, Y_n = 1), then L* = 116 f(Y) - 16.
    const double y = 0.2126729 * rgb.x + 0.7151522 * rgb.y + 0.0721750 * rgb.z;
    constexpr double kEpsilon = 216.0 / 24389.0;
    constexpr double kKappa = 24389.0 / 27.0;
    return y > kEpsilon ? 116.0 * std::cbrt(y) - 16.0 : kKappa * y;
}

glm::dvec3 colormap_linear(Colormap map, double t) noexcept {
    const double u = clamp_parameter(t);
    if (map == Colormap::diverging) {
        const double d = std::isnan(u) ? 0.0 : 2.0 * u - 1.0;
        const double a = std::abs(d);
        const double lightness = kDivergingCentreL + (kDivergingEndL - kDivergingCentreL) * a;
        const double hue = d < 0.0 ? kDivergingNegativeHue : kDivergingPositiveHue;
        return from_lstar(lightness, kDivergingEndChroma * a, hue);
    }
    const double s = std::isnan(u) ? 0.0 : u;
    return from_lstar(kSequentialStartL + (kSequentialEndL - kSequentialStartL) * s,
                      kSequentialStartChroma + (kSequentialEndChroma - kSequentialStartChroma) * s,
                      kSequentialStartHue + (kSequentialEndHue - kSequentialStartHue) * s);
}

glm::dvec3 colormap_srgb(Colormap map, double t) noexcept {
    const glm::dvec3 c = colormap_linear(map, t);
    const auto encode = [](double v) {
        const double x = std::clamp(v, 0.0, 1.0);
        return x <= 0.0031308 ? 12.92 * x : 1.055 * std::pow(x, 1.0 / 2.4) - 0.055;
    };
    return {encode(c.x), encode(c.y), encode(c.z)};
}

std::vector<glm::dvec3> colormap_table(Colormap map, std::size_t count) {
    const std::size_t n = std::max<std::size_t>(count, 2);
    std::vector<glm::dvec3> table;
    table.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        table.push_back(colormap_linear(map, static_cast<double>(i) / static_cast<double>(n - 1)));
    }
    return table;
}

glm::dvec3 scene_linear_for_display(const glm::dvec3& display, double exposure) noexcept {
    const glm::dvec3 target = glm::clamp(display, glm::dvec3(0.0), glm::dvec3(kMaxDisplay));
    const glm::dvec3 curve = multiply(output_inverse(), target);
    const glm::dvec3 v{inverse_curve(curve.x), inverse_curve(curve.y), inverse_curve(curve.z)};
    const glm::dvec3 linear = glm::max(multiply(input_inverse(), v), glm::dvec3(0.0));
    const double scale = exposure > 0.0 && std::isfinite(exposure) ? exposure : 1.0;
    return linear / scale;
}

double ScalarRange::normalize(double value) const noexcept {
    const double span = max - min;
    if (!(span > 0.0) || !std::isfinite(span)) {
        return 0.5;
    }
    return std::clamp((value - min) / span, 0.0, 1.0);
}

ScalarRange resolve_range(const ColorScale& scale, std::span<const double> values) {
    if (scale.mode == ScaleMode::fixed && std::isfinite(scale.min) && std::isfinite(scale.max) &&
        scale.min < scale.max) {
        return {scale.min, scale.max};
    }
    double low = std::numeric_limits<double>::infinity();
    double high = -std::numeric_limits<double>::infinity();
    for (const double v : values) {
        if (std::isfinite(v)) {
            low = std::min(low, v);
            high = std::max(high, v);
        }
    }
    const bool symmetric = scale.mode == ScaleMode::symmetric;
    if (!(low <= high)) {
        return symmetric ? ScalarRange{-1.0, 1.0} : ScalarRange{0.0, 1.0};
    }
    if (symmetric) {
        double m = std::max(std::abs(low), std::abs(high));
        if (!(m > 0.0)) {
            m = 1.0;
        }
        if (scale.round_to_ticks) {
            m = plan_ticks(-m, m, 5, true).high;
        }
        return {-m, m};
    }
    if (!(high > low)) {
        const double pad = low != 0.0 ? 0.5 * std::abs(low) : 0.5;
        low -= pad;
        high += pad;
    }
    if (scale.round_to_ticks) {
        const TickPlan plan = plan_ticks(low, high, 5, true);
        return {plan.low, plan.high};
    }
    return {low, high};
}

std::vector<LegendTick> legend_ticks(const ScalarRange& range, int target) {
    std::vector<LegendTick> ticks;
    if (!(range.max > range.min) || !std::isfinite(range.max - range.min)) {
        return ticks;
    }
    const TickPlan plan = plan_ticks(range.min, range.max, target, false);
    const double magnitude = std::max(std::abs(range.min), std::abs(range.max));
    const auto count = static_cast<long long>(std::round((plan.high - plan.low) / plan.step));
    ticks.reserve(static_cast<std::size_t>(std::max(count + 1, 0LL)));
    for (long long i = 0; i <= count; ++i) {
        const double value = plan.low + static_cast<double>(i) * plan.step;
        ticks.push_back({value, range.normalize(value), format_value(value, plan.step, magnitude)});
    }
    return ticks;
}

Legend make_legend(std::string title, Colormap map, const ScalarRange& range, int target_ticks) {
    Legend legend;
    legend.title = std::move(title);
    legend.colormap = map;
    legend.range = range;
    legend.ticks = legend_ticks(range, target_ticks);
    return legend;
}

} // namespace orrery::render
