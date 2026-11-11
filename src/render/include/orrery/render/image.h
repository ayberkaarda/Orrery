#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace orrery::render {

// 8-bit RGBA image, rows top to bottom, 4 bytes per pixel, no padding.
// Colour values are sRGB-encoded; alpha is linear coverage.
//
// The byte size width * height * 4 is computed with overflow checks: a size
// that does not fit in std::size_t makes the image invalid and is never
// allocated.
struct ImageRgba8 {
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::vector<std::uint8_t> pixels;

    // An image of the given size with every pixel set to (r, g, b, a). When
    // width * height * 4 does not fit in std::size_t, nothing is allocated and
    // the result is the empty (invalid) image of size 0 x 0.
    [[nodiscard]] static ImageRgba8 filled(std::uint32_t width, std::uint32_t height,
                                           std::uint8_t r, std::uint8_t g, std::uint8_t b,
                                           std::uint8_t a = 255);

    // width * height * 4, or std::nullopt when that does not fit in std::size_t.
    [[nodiscard]] static std::optional<std::size_t>
    checked_byte_size(std::uint32_t width, std::uint32_t height) noexcept {
        constexpr std::size_t kMax = std::numeric_limits<std::size_t>::max();
        const std::size_t w = width;
        const std::size_t h = height;
        if (w != 0 && h > kMax / w) {
            return std::nullopt;
        }
        const std::size_t count = w * h;
        if (count > kMax / 4U) {
            return std::nullopt;
        }
        return count * 4U;
    }
    // Byte size of this image's dimensions; 0 when it is not representable
    // (such an image is never valid()).
    [[nodiscard]] std::size_t byte_size() const noexcept {
        return checked_byte_size(width, height).value_or(0);
    }
    [[nodiscard]] bool valid() const noexcept {
        const std::optional<std::size_t> size = checked_byte_size(width, height);
        return width > 0 && height > 0 && size.has_value() && pixels.size() == *size;
    }
    // Requires valid() and x < width, y < height.
    [[nodiscard]] std::size_t offset(std::uint32_t x, std::uint32_t y) const noexcept {
        return (static_cast<std::size_t>(y) * width + x) * 4U;
    }
};

} // namespace orrery::render
