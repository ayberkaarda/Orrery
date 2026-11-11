#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

#include "orrery/render/image.h"
#include "orrery/render/ssim.h"

// Image dimensions are unrestricted 32-bit values; their byte size
// (width * height * 4) must be computed without wrapping, so that a malformed
// image never passes validation and nothing allocates a wrapped size.

using namespace orrery::render;

TEST_CASE("Image byte sizes that overflow size_t are never valid", "[render][image]") {
    if constexpr (sizeof(std::size_t) == 8) {
        // 2^31 x 2^31 x 4 = 2^64 wraps to 0 in a 64-bit size_t.
        ImageRgba8 image;
        image.width = 0x80000000U;
        image.height = 0x80000000U;
        CHECK_FALSE(image.valid());
        CHECK_FALSE(ImageRgba8::checked_byte_size(image.width, image.height).has_value());
        CHECK(image.byte_size() == 0);

        const ImageRgba8 filled = ImageRgba8::filled(0x80000000U, 0x80000000U, 1, 2, 3);
        CHECK_FALSE(filled.valid());
        CHECK(filled.pixels.empty());

        // SSIM rejects the image before allocating anything.
        const auto score = ssim(image, image);
        REQUIRE_FALSE(score.has_value());
        CHECK(score.error().find("invalid image") != std::string::npos);

        // Around the boundary 2^64 - 1:
        //   (2^32 - 1) x 2^30 x 4 = 2^64 - 2^32 fits,
        //   (2^32 - 1) x (2^30 + 1) x 4 does not; neither does 2^31 x 2^31 x 4,
        //   while 2^31 x (2^31 - 1) x 4 = 2^64 - 2^33 fits.
        CHECK(ImageRgba8::checked_byte_size(0xFFFFFFFFU, 0x40000000U) ==
              std::optional<std::size_t>(std::numeric_limits<std::size_t>::max() - 0xFFFFFFFFU));
        CHECK_FALSE(ImageRgba8::checked_byte_size(0xFFFFFFFFU, 0x40000001U).has_value());
        CHECK(ImageRgba8::checked_byte_size(0x80000000U, 0x7FFFFFFFU) ==
              std::optional<std::size_t>(std::numeric_limits<std::size_t>::max() - 0x1FFFFFFFFU));
        ImageRgba8 past;
        past.width = 0xFFFFFFFFU;
        past.height = 0x40000001U;
        CHECK_FALSE(past.valid());
        const ImageRgba8 past_filled = ImageRgba8::filled(0xFFFFFFFFU, 0x40000001U, 0, 0, 0);
        CHECK_FALSE(past_filled.valid());
        CHECK(past_filled.pixels.empty());
    }
    // Small sizes are exact.
    CHECK(ImageRgba8::checked_byte_size(0, 7) == std::optional<std::size_t>(0));
    CHECK(ImageRgba8::checked_byte_size(3, 5) == std::optional<std::size_t>(60));
    CHECK(ImageRgba8::filled(3, 5, 1, 2, 3).valid());
}

TEST_CASE("SSIM refuses images beyond its working-memory limit before allocating",
          "[render][image]") {
    // A valid 4097 x 4096 image (67 MB) is over the 2^24-pixel limit: SSIM
    // would need about 1.5 GB of work buffers and refuses up front.
    const ImageRgba8 large = ImageRgba8::filled(4097, 4096, 10, 20, 30);
    REQUIRE(large.valid());
    const auto score = ssim(large, large);
    REQUIRE_FALSE(score.has_value());
    CHECK(score.error().find("exceeds the limit") != std::string::npos);

    // Ordinary images are unaffected.
    const ImageRgba8 small = ImageRgba8::filled(16, 16, 10, 20, 30);
    REQUIRE(small.valid());
    const auto self = ssim(small, small);
    REQUIRE(self.has_value());
    CHECK(self.value().value == Catch::Approx(1.0).epsilon(1.0e-12));
}
