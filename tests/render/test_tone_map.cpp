#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "orrery/render/tone_map.h"

using namespace orrery::render;

TEST_CASE("sRGB transfer function matches IEC 61966-2-1", "[render][tone_map]") {
    CHECK(srgb_encode(0.0) == 0.0);
    CHECK(srgb_encode(1.0) == Catch::Approx(1.0).epsilon(1.0e-15));
    CHECK(srgb_encode(0.0031308) == Catch::Approx(12.92 * 0.0031308).epsilon(1.0e-12));
    // Mid grey (18 %) encodes to about 46 %, i.e. 8-bit 118.
    CHECK(to_srgb8(0.18) == 118);
    CHECK(to_srgb8(-1.0) == 0);
    CHECK(to_srgb8(2.0) == 255);
    for (const double v : {0.001, 0.02, 0.2, 0.5, 0.9}) {
        CHECK(srgb_decode(srgb_encode(v)) == Catch::Approx(v).epsilon(1.0e-12));
    }
}

TEST_CASE("ACES fitted curve: black stays black, monotonic, soft shoulder", "[render][tone_map]") {
    CHECK(tone_map_aces(glm::dvec3(0.0)).x == Catch::Approx(0.0).margin(1.0e-3));
    double previous = -1.0;
    for (double v = 0.0; v <= 64.0; v = v * 1.25 + 0.01) {
        const double out = tone_map_aces(glm::dvec3(v)).y;
        CHECK(out >= previous);
        CHECK(out <= 1.0);
        previous = out;
    }
    // Scene-linear 1.0 is not clipped; very bright values approach white.
    CHECK(tone_map_aces(glm::dvec3(1.0)).x < 0.9);
    CHECK(tone_map_aces(glm::dvec3(16.0)).x > 0.97);
    // Negative input is treated as black.
    CHECK(tone_map_aces(glm::dvec3(-5.0)).z == Catch::Approx(0.0).margin(1.0e-3));
}

TEST_CASE("HDR pixel resolve combines exposure, ACES and sRGB", "[render][tone_map]") {
    const Rgb8 dark = resolve_hdr_pixel(glm::dvec3(0.0), 1.0);
    CHECK(dark.r == 0);
    const Rgb8 a = resolve_hdr_pixel(glm::dvec3(0.5), 2.0);
    const Rgb8 b = resolve_hdr_pixel(glm::dvec3(1.0), 1.0);
    CHECK(a.r == b.r);
    CHECK(a.g == b.g);
    CHECK(a.b == b.b);
}
