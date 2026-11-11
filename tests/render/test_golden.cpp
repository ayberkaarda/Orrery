#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <string>

#include "gl_test_support.h"
#include "orrery/render/png.h"
#include "orrery/render/ssim.h"
#include "orrery/render/test_scene.h"
#include "orrery/render/tone_map.h"

// Golden snapshot of the pipeline test scene (docs/rendering.md, "Golden
// snapshots"). The reference lives in tests/render/golden/. To regenerate it
// after an intended visual change, run
//   ORRERY_UPDATE_GOLDEN=1 orrery_test_render "[golden]"
// and review the new image before committing it. The test never writes the
// reference on its own: a missing reference fails with this instruction.

using namespace orrery::render;
using orrery::render::test::environment_flag;
using orrery::render::test::renderer_or_skip;

namespace {

constexpr std::uint32_t kWidth = 320;
constexpr std::uint32_t kHeight = 240;
constexpr double kSsimThreshold = 0.98;

int max_channel_difference(const ImageRgba8& image, std::uint32_t x, std::uint32_t y,
                           const Rgb8& expected) {
    const std::size_t o = image.offset(x, y);
    return std::max({std::abs(image.pixels[o] - expected.r),
                     std::abs(image.pixels[o + 1] - expected.g),
                     std::abs(image.pixels[o + 2] - expected.b)});
}

double luma(const ImageRgba8& image, std::uint32_t x, std::uint32_t y) {
    const std::size_t o = image.offset(x, y);
    return 0.2126 * image.pixels[o] + 0.7152 * image.pixels[o + 1] + 0.0722 * image.pixels[o + 2];
}

} // namespace

TEST_CASE("Golden snapshot: lit test shape through HDR, ACES and sRGB", "[render][gl][golden]") {
    HdrSettings settings;
    settings.samples = 4;
    settings.exposure = 1.0;
    OffscreenRenderer renderer =
        renderer_or_skip(static_cast<int>(kWidth), static_cast<int>(kHeight), settings);

    auto scene = TestShapeScene::create();
    REQUIRE(scene.has_value());
    const Camera camera = test_scene_reference_camera();
    auto rendered =
        renderer.render([&](const FrameInfo& frame) { scene.value().draw(camera, frame); });
    REQUIRE(rendered.has_value());
    const ImageRgba8& image = rendered.value();

    const std::filesystem::path output_dir = ORRERY_RENDER_OUTPUT_DIR;
    std::filesystem::create_directories(output_dir);
    const std::filesystem::path actual_path = output_dir / "test_shape_actual.png";
    REQUIRE(write_png(actual_path, image).has_value());

    // Structural checks that hold on any conforming driver.
    // The backdrop corners follow the CPU tone map of the gradient ends.
    const TestSceneLighting& lighting = scene.value().lighting;
    const Rgb8 top = resolve_hdr_pixel(glm::dvec3(lighting.backdrop_zenith), settings.exposure);
    const Rgb8 bottom = resolve_hdr_pixel(glm::dvec3(lighting.backdrop_horizon), settings.exposure);
    CHECK(max_channel_difference(image, 0, 0, top) <= 2);
    CHECK(max_channel_difference(image, kWidth - 1, kHeight - 1, bottom) <= 2);
    // The torus fills the middle of the frame and is lit well above the
    // backdrop, with highlights rolled off below pure white by ACES.
    double brightest = 0.0;
    std::uint32_t lit = 0;
    for (std::uint32_t y = 0; y < kHeight; ++y) {
        for (std::uint32_t x = 0; x < kWidth; ++x) {
            const double l = luma(image, x, y);
            brightest = std::max(brightest, l);
            lit += l > 100.0 ? 1U : 0U;
        }
    }
    CHECK(brightest > 200.0);
    CHECK(brightest < 255.0);
    CHECK(lit > kWidth * kHeight / 20);

    const std::filesystem::path golden_path =
        std::filesystem::path(ORRERY_RENDER_GOLDEN_DIR) / "test_shape.png";
    if (environment_flag("ORRERY_UPDATE_GOLDEN")) {
        std::filesystem::create_directories(golden_path.parent_path());
        REQUIRE(write_png(golden_path, image).has_value());
        WARN("ORRERY_UPDATE_GOLDEN=1: wrote the golden reference " << golden_path.string()
                                                                   << "; review it before "
                                                                      "committing");
        return;
    }
    if (!std::filesystem::exists(golden_path)) {
        FAIL("golden reference " << golden_path.string()
                                 << " is missing; generate it with ORRERY_UPDATE_GOLDEN=1 "
                                    "orrery_test_render \"[golden]\" and review it");
    }
    auto golden = read_png(golden_path);
    REQUIRE(golden.has_value());
    auto score = ssim(golden.value(), image);
    REQUIRE(score.has_value());
    INFO("SSIM R " << score.value().channel[0] << " G " << score.value().channel[1] << " B "
                   << score.value().channel[2] << "; actual image: " << actual_path.string());
    CHECK(score.value().value > kSsimThreshold);
}
