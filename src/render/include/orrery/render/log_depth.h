#pragma once

#include <algorithm>
#include <cmath>

// CPU side of the logarithmic depth buffer (shaders/log_depth.glsl).
//
// depth(w) = log2(1 + w) / log2(1 + far), w = view-space distance along the
// view axis [m]. Depth is 0 at the eye and 1 at `far`; it is written to
// gl_FragDepth and stored in a 32-bit float depth attachment.
namespace orrery::render {

// Default far distance for log depth: 1e20 m (about 6.7e8 AU), beyond any
// scene in the space workspace. Distances past it clamp to depth 1.
inline constexpr double kDefaultLogDepthFar = 1.0e20;

// The value of u_log_depth_scale for a given far distance.
[[nodiscard]] inline double log_depth_scale(double far) noexcept {
    return 1.0 / std::log2(1.0 + far);
}

[[nodiscard]] inline double log_depth(double view_w, double far) noexcept {
    return std::log2(std::max(1.0 + view_w, 1.0e-6)) * log_depth_scale(far);
}

// Inverse of log_depth: the view distance that maps to `depth`.
[[nodiscard]] inline double log_depth_inverse(double depth, double far) noexcept {
    return std::exp2(depth / log_depth_scale(far)) - 1.0;
}

// Smallest relative distance step dw / w that a depth buffer with relative
// precision `depth_epsilon` (2^-24 for float32 near depth 1) resolves at view
// distance w: dw / w = depth_epsilon * depth * ln 2 * log2(1 + far) * (1 + w) / w,
// using the float spacing at the depth value itself.
[[nodiscard]] inline double log_depth_relative_resolution(double view_w, double far,
                                                          double depth_epsilon) noexcept {
    const double depth = log_depth(view_w, far);
    const double spacing = depth_epsilon * std::max(depth, depth_epsilon);
    return spacing * std::log(2.0) / log_depth_scale(far) * (1.0 + view_w) / view_w;
}

} // namespace orrery::render
