#include "orrery/render/tone_map.h"

#include <algorithm>
#include <cmath>

#include <glm/common.hpp>

namespace orrery::render {

namespace {

// Rows of the published matrices (Hill, BakingLab ACES.hlsl): sRGB ->
// ACES AP1-like input space (with the RRT saturation folded in) and back.
constexpr double kAcesInput[3][3] = {
    {0.59719, 0.35458, 0.04823},
    {0.07600, 0.90834, 0.01566},
    {0.02840, 0.13383, 0.83777},
};
constexpr double kAcesOutput[3][3] = {
    {1.60475, -0.53108, -0.07367},
    {-0.10208, 1.10813, -0.00605},
    {-0.00327, -0.07276, 1.07602},
};

glm::dvec3 multiply(const double (&m)[3][3], const glm::dvec3& v) {
    return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
}

double rrt_and_odt_fit(double v) {
    const double a = v * (v + 0.0245786) - 0.000090537;
    const double b = v * (0.983729 * v + 0.4329510) + 0.238081;
    return a / b;
}

} // namespace

glm::dvec3 tone_map_aces(const glm::dvec3& linear_srgb) noexcept {
    glm::dvec3 v = multiply(kAcesInput, glm::max(linear_srgb, glm::dvec3(0.0)));
    v = {rrt_and_odt_fit(v.x), rrt_and_odt_fit(v.y), rrt_and_odt_fit(v.z)};
    v = multiply(kAcesOutput, v);
    return glm::clamp(v, glm::dvec3(0.0), glm::dvec3(1.0));
}

double srgb_encode(double linear) noexcept {
    const double c = std::clamp(linear, 0.0, 1.0);
    return c <= 0.0031308 ? 12.92 * c : 1.055 * std::pow(c, 1.0 / 2.4) - 0.055;
}

double srgb_decode(double encoded) noexcept {
    const double c = std::clamp(encoded, 0.0, 1.0);
    return c <= 0.04045 ? c / 12.92 : std::pow((c + 0.055) / 1.055, 2.4);
}

std::uint8_t to_srgb8(double linear) noexcept {
    return static_cast<std::uint8_t>(std::lround(srgb_encode(linear) * 255.0));
}

Rgb8 resolve_hdr_pixel(const glm::dvec3& scene_linear, double exposure) noexcept {
    const glm::dvec3 ldr = tone_map_aces(scene_linear * exposure);
    return {to_srgb8(ldr.x), to_srgb8(ldr.y), to_srgb8(ldr.z)};
}

} // namespace orrery::render
