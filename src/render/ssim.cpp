#include "orrery/render/ssim.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <vector>

namespace orrery::render {

namespace {

constexpr double kSigma = 1.5;
constexpr double kC1 = (0.01 * 255.0) * (0.01 * 255.0);
constexpr double kC2 = (0.03 * 255.0) * (0.03 * 255.0);

std::array<double, kSsimWindow> gaussian_weights() {
    std::array<double, kSsimWindow> weights{};
    double sum = 0.0;
    for (int i = 0; i < kSsimWindow; ++i) {
        const int offset = i - (kSsimWindow - 1) / 2;
        const double d = offset;
        weights[static_cast<std::size_t>(i)] = std::exp(-(d * d) / (2.0 * kSigma * kSigma));
        sum += weights[static_cast<std::size_t>(i)];
    }
    for (double& w : weights) {
        w /= sum;
    }
    return weights;
}

// Separable Gaussian filter over the valid region: input w x h, output
// (w - 10) x (h - 10).
std::vector<double> blur_valid(const std::vector<double>& in, std::size_t w, std::size_t h,
                               const std::array<double, kSsimWindow>& weights) {
    const std::size_t n = kSsimWindow;
    const std::size_t ow = w - n + 1;
    const std::size_t oh = h - n + 1;
    std::vector<double> horizontal(ow * h);
    for (std::size_t y = 0; y < h; ++y) {
        for (std::size_t x = 0; x < ow; ++x) {
            double sum = 0.0;
            for (std::size_t k = 0; k < n; ++k) {
                sum += weights[k] * in[y * w + x + k];
            }
            horizontal[y * ow + x] = sum;
        }
    }
    std::vector<double> out(ow * oh);
    for (std::size_t y = 0; y < oh; ++y) {
        for (std::size_t x = 0; x < ow; ++x) {
            double sum = 0.0;
            for (std::size_t k = 0; k < n; ++k) {
                sum += weights[k] * horizontal[(y + k) * ow + x];
            }
            out[y * ow + x] = sum;
        }
    }
    return out;
}

double channel_ssim(const ImageRgba8& a, const ImageRgba8& b, std::size_t channel,
                    const std::array<double, kSsimWindow>& weights) {
    const std::size_t w = a.width;
    const std::size_t h = a.height;
    std::vector<double> x(w * h);
    std::vector<double> y(w * h);
    std::vector<double> xx(w * h);
    std::vector<double> yy(w * h);
    std::vector<double> xy(w * h);
    for (std::size_t i = 0; i < w * h; ++i) {
        x[i] = a.pixels[i * 4 + channel];
        y[i] = b.pixels[i * 4 + channel];
        xx[i] = x[i] * x[i];
        yy[i] = y[i] * y[i];
        xy[i] = x[i] * y[i];
    }
    const std::vector<double> mu_x = blur_valid(x, w, h, weights);
    const std::vector<double> mu_y = blur_valid(y, w, h, weights);
    const std::vector<double> e_xx = blur_valid(xx, w, h, weights);
    const std::vector<double> e_yy = blur_valid(yy, w, h, weights);
    const std::vector<double> e_xy = blur_valid(xy, w, h, weights);

    double sum = 0.0;
    for (std::size_t i = 0; i < mu_x.size(); ++i) {
        const double mx = mu_x[i];
        const double my = mu_y[i];
        // Variances can come out marginally negative from rounding; clamp.
        const double var_x = std::max(e_xx[i] - mx * mx, 0.0);
        const double var_y = std::max(e_yy[i] - my * my, 0.0);
        const double cov = e_xy[i] - mx * my;
        sum += ((2.0 * mx * my + kC1) * (2.0 * cov + kC2)) /
               ((mx * mx + my * my + kC1) * (var_x + var_y + kC2));
    }
    return sum / static_cast<double>(mu_x.size());
}

} // namespace

core::Result<SsimResult, std::string> ssim(const ImageRgba8& a, const ImageRgba8& b) {
    if (!a.valid() || !b.valid()) {
        return core::failure(std::string("SSIM: invalid image"));
    }
    for (const ImageRgba8* image : {&a, &b}) {
        const std::uint64_t pixels = std::uint64_t{image->width} * image->height;
        if (pixels > kSsimMaxPixels) {
            return core::failure("SSIM: image " + std::to_string(image->width) + "x" +
                                 std::to_string(image->height) + " exceeds the limit of " +
                                 std::to_string(kSsimMaxPixels) + " pixels");
        }
    }
    if (a.width != b.width || a.height != b.height) {
        return core::failure("SSIM: size mismatch " + std::to_string(a.width) + "x" +
                             std::to_string(a.height) + " vs " + std::to_string(b.width) + "x" +
                             std::to_string(b.height));
    }
    constexpr auto kMinSize = static_cast<std::uint32_t>(kSsimWindow);
    if (a.width < kMinSize || a.height < kMinSize) {
        return core::failure(std::string("SSIM: images must be at least 11x11 pixels"));
    }
    const std::array<double, kSsimWindow> weights = gaussian_weights();
    SsimResult result;
    for (std::size_t c = 0; c < 3; ++c) {
        result.channel[c] = channel_ssim(a, b, c, weights);
    }
    result.value = *std::ranges::min_element(result.channel);
    return result;
}

} // namespace orrery::render
