#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "orrery/render/ssim.h"

using namespace orrery::render;

namespace {

// Smooth test pattern with structure at several scales.
ImageRgba8 pattern(std::uint32_t width, std::uint32_t height, int shift = 0) {
    ImageRgba8 image = ImageRgba8::filled(width, height, 0, 0, 0);
    for (std::uint32_t y = 0; y < height; ++y) {
        for (std::uint32_t x = 0; x < width; ++x) {
            const double fx = static_cast<double>(static_cast<int>(x) + shift);
            const double fy = static_cast<double>(y);
            const std::size_t o = image.offset(x, y);
            image.pixels[o] = static_cast<std::uint8_t>(127.5 + 100.0 * std::sin(fx * 0.21) *
                                                                    std::cos(fy * 0.13));
            image.pixels[o + 1] =
                static_cast<std::uint8_t>(127.5 + 90.0 * std::sin((fx + fy) * 0.07));
            image.pixels[o + 2] = static_cast<std::uint8_t>((x * 3U + y * 5U) % 256U);
            image.pixels[o + 3] = 255;
        }
    }
    return image;
}

double ssim_value(const ImageRgba8& a, const ImageRgba8& b) {
    auto result = ssim(a, b);
    REQUIRE(result.has_value());
    return result.value().value;
}

} // namespace

TEST_CASE("SSIM of an image with itself is 1", "[render][ssim]") {
    const ImageRgba8 image = pattern(64, 48);
    CHECK(ssim_value(image, image) == Catch::Approx(1.0).epsilon(1.0e-12));
}

TEST_CASE("SSIM of constant images matches the closed form", "[render][ssim]") {
    // Zero variance: SSIM = (2 mu_x mu_y + C1) / (mu_x^2 + mu_y^2 + C1).
    const double c1 = (0.01 * 255.0) * (0.01 * 255.0);
    const ImageRgba8 a = ImageRgba8::filled(32, 32, 100, 100, 100);
    const ImageRgba8 b = ImageRgba8::filled(32, 32, 120, 100, 100);
    auto result = ssim(a, b);
    REQUIRE(result.has_value());
    const double expected = (2.0 * 100.0 * 120.0 + c1) / (100.0 * 100.0 + 120.0 * 120.0 + c1);
    CHECK(result.value().channel[0] == Catch::Approx(expected).epsilon(1.0e-12));
    CHECK(result.value().channel[1] == Catch::Approx(1.0).epsilon(1.0e-12));
    CHECK(result.value().value == Catch::Approx(expected).epsilon(1.0e-12));
}

TEST_CASE("SSIM is symmetric, bounded and drops with distortion", "[render][ssim]") {
    const ImageRgba8 reference = pattern(80, 60);
    const ImageRgba8 shifted = pattern(80, 60, 2);
    ImageRgba8 noisy = reference;
    std::uint32_t state = 12345;
    for (std::size_t i = 0; i < noisy.pixels.size(); ++i) {
        if (i % 4 == 3) {
            continue;
        }
        state = state * 1664525U + 1013904223U;
        const int delta = static_cast<int>((state >> 24U) % 21U) - 10;
        noisy.pixels[i] = static_cast<std::uint8_t>(
            std::clamp(static_cast<int>(noisy.pixels[i]) + delta, 0, 255));
    }

    const double s_noise = ssim_value(reference, noisy);
    const double s_shift = ssim_value(reference, shifted);
    CHECK(ssim_value(noisy, reference) == Catch::Approx(s_noise).epsilon(1.0e-12));
    CHECK(s_noise < 1.0);
    CHECK(s_noise > 0.5);
    CHECK(s_shift < 0.95);
    CHECK(s_shift > -1.0);

    // A one-level change everywhere is still nearly identical.
    ImageRgba8 brighter = reference;
    for (std::size_t i = 0; i < brighter.pixels.size(); i += 4) {
        brighter.pixels[i] = static_cast<std::uint8_t>(std::min(brighter.pixels[i] + 1, 255));
    }
    CHECK(ssim_value(reference, brighter) > 0.99);
}

TEST_CASE("SSIM rejects mismatched or too small images", "[render][ssim]") {
    CHECK_FALSE(ssim(pattern(20, 20), pattern(21, 20)).has_value());
    CHECK_FALSE(ssim(pattern(10, 10), pattern(10, 10)).has_value());
    CHECK_FALSE(ssim(ImageRgba8{}, ImageRgba8{}).has_value());
}
