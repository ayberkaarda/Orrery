#include "orrery/render/image.h"

namespace orrery::render {

ImageRgba8 ImageRgba8::filled(std::uint32_t width, std::uint32_t height, std::uint8_t r,
                              std::uint8_t g, std::uint8_t b, std::uint8_t a) {
    const std::optional<std::size_t> size = checked_byte_size(width, height);
    if (!size) {
        return {};
    }
    ImageRgba8 image;
    image.width = width;
    image.height = height;
    image.pixels.resize(*size);
    for (std::size_t i = 0; i < image.pixels.size(); i += 4) {
        image.pixels[i] = r;
        image.pixels[i + 1] = g;
        image.pixels[i + 2] = b;
        image.pixels[i + 3] = a;
    }
    return image;
}

} // namespace orrery::render
