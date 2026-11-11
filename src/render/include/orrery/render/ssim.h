#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "orrery/core/result.h"
#include "orrery/render/image.h"

// Structural similarity (SSIM) for snapshot comparison.
//
// Z. Wang, A. C. Bovik, H. R. Sheikh, E. P. Simoncelli, "Image quality
// assessment: from error visibility to structural similarity", IEEE TIP 13(4),
// 2004. Per channel (R, G, B on the stored 8-bit values; alpha is ignored):
//   SSIM(x, y) = (2 mu_x mu_y + C1)(2 sigma_xy + C2) /
//                ((mu_x^2 + mu_y^2 + C1)(sigma_x^2 + sigma_y^2 + C2))
// with C1 = (0.01 * 255)^2, C2 = (0.03 * 255)^2 and local statistics from an
// 11x11 Gaussian window (sigma 1.5) evaluated at every position where the
// window fits inside the image ("valid" region, as in the reference
// implementation). The channel score is the mean over those positions; the
// overall score is the minimum channel score, so a change confined to one
// channel is not averaged away.
namespace orrery::render {

struct SsimResult {
    std::array<double, 3> channel{}; // mean SSIM of R, G, B
    double value = 0.0;              // min of the channel scores
};

inline constexpr int kSsimWindow = 11;

// Largest image (width * height) ssim() accepts: 2^24 pixels (4096 x 4096).
// Its double-precision work buffers take about 90 bytes per pixel.
inline constexpr std::uint64_t kSsimMaxPixels = std::uint64_t{1} << 24U;

// Both images must be valid, of equal size, at least 11x11 pixels and at most
// kSsimMaxPixels pixels. Sizes are checked before anything is allocated.
[[nodiscard]] core::Result<SsimResult, std::string> ssim(const ImageRgba8& a, const ImageRgba8& b);

} // namespace orrery::render
