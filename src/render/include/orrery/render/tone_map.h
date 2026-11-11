#pragma once

#include <cstdint>

#include <glm/ext/vector_double3.hpp>

// CPU reference of the HDR resolve in shaders/tonemap.frag.glsl:
//   scene-linear RGB * exposure -> ACES filmic (fitted) -> sRGB transfer -> 8 bit.
// The GPU path is checked against these functions in the [gl] tests.
namespace orrery::render {

// APPROXIMATION: ACES RRT + ODT (sRGB, 100 nit) - Stephen Hill's fitted
// matrices and rational curve instead of the reference CTL transforms - a few
// percent from the reference over scene-linear [0, ~16]; output clamped to [0, 1].
[[nodiscard]] glm::dvec3 tone_map_aces(const glm::dvec3& linear_srgb) noexcept;

// IEC 61966-2-1 sRGB transfer function, input clamped to [0, 1].
[[nodiscard]] double srgb_encode(double linear) noexcept;
[[nodiscard]] double srgb_decode(double encoded) noexcept;

// Round-to-nearest 8-bit quantisation of srgb_encode(linear).
[[nodiscard]] std::uint8_t to_srgb8(double linear) noexcept;

// Full CPU pipeline for one pixel: exposure, ACES, sRGB, 8 bit.
struct Rgb8 {
    std::uint8_t r = 0;
    std::uint8_t g = 0;
    std::uint8_t b = 0;
};
[[nodiscard]] Rgb8 resolve_hdr_pixel(const glm::dvec3& scene_linear, double exposure) noexcept;

} // namespace orrery::render
