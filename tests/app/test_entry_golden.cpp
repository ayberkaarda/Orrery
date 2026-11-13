#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "../render/gl_test_support.h"
#include "orrery/app/entry_headless.h"
#include "orrery/render/png.h"
#include "orrery/render/ssim.h"

// Golden snapshot of the Entry workspace (docs/entry-workspace.md,
// "Headless"): the Apollo-like preset drawn by the headless path - bar,
// setup, profile plots, ground track map and the 3D view with its legend.
// The reference lives in tests/render/golden/entry/ and is written only with
//   ORRERY_UPDATE_GOLDEN=1 orrery_test_app "[golden]"
// (review the image before committing it).

using namespace orrery;
using render::test::environment_flag;

namespace {

constexpr double kSsimThreshold = 0.98;
constexpr int kWidth = 1600;
constexpr int kHeight = 900;

void check_golden(const render::ImageRgba8& image, const std::string& name) {
    const std::filesystem::path out_dir = std::filesystem::path(ORRERY_RENDER_OUTPUT_DIR) / "entry";
    std::filesystem::create_directories(out_dir);
    const std::filesystem::path actual = out_dir / (name + "_actual.png");
    REQUIRE(render::write_png(actual, image).has_value());
    const std::filesystem::path golden =
        std::filesystem::path(ORRERY_RENDER_GOLDEN_DIR) / "entry" / (name + ".png");
    if (environment_flag("ORRERY_UPDATE_GOLDEN")) {
        std::filesystem::create_directories(golden.parent_path());
        REQUIRE(render::write_png(golden, image).has_value());
        WARN("ORRERY_UPDATE_GOLDEN=1: wrote the golden reference " << golden.string()
                                                                   << "; review it before "
                                                                      "committing");
        return;
    }
    if (!std::filesystem::exists(golden)) {
        FAIL("golden reference " << golden.string()
                                 << " is missing; generate it with ORRERY_UPDATE_GOLDEN=1 "
                                    "orrery_test_app \"[golden]\" and review it");
    }
    auto reference = render::read_png(golden);
    REQUIRE(reference.has_value());
    auto score = render::ssim(reference.value(), image);
    REQUIRE(score.has_value());
    std::cout << "measured: entry golden " << name << " SSIM " << score.value().value << "\n";
    INFO("actual image: " << actual.string());
    CHECK(score.value().value > kSsimThreshold);
}

double luma(const render::ImageRgba8& image, std::uint32_t x, std::uint32_t y) {
    const std::size_t o = image.offset(x, y);
    return 0.2126 * image.pixels[o] + 0.7152 * image.pixels[o + 1] + 0.0722 * image.pixels[o + 2];
}

// Fraction of pixels in a box whose colour is clearly saturated (the
// colour-mapped ribbon and legend; the panels and the planet are muted).
double saturated_fraction(const render::ImageRgba8& image, std::uint32_t x0, std::uint32_t y0,
                          std::uint32_t x1, std::uint32_t y1) {
    std::size_t count = 0;
    std::size_t total = 0;
    for (std::uint32_t y = y0; y < y1; ++y) {
        for (std::uint32_t x = x0; x < x1; ++x) {
            const std::size_t o = image.offset(x, y);
            const int r = image.pixels[o];
            const int g = image.pixels[o + 1];
            const int b = image.pixels[o + 2];
            const int hi = std::max({r, g, b});
            const int lo = std::min({r, g, b});
            count += (hi > 90 && hi - lo > 60) ? 1U : 0U;
            ++total;
        }
    }
    return total > 0 ? static_cast<double>(count) / static_cast<double>(total) : 0.0;
}

} // namespace

TEST_CASE("Golden snapshot: Entry workspace with an Apollo-like entry",
          "[app][entry][gl][golden]") {
    const std::vector<sim::EntryPreset> presets = sim::entry_presets();
    REQUIRE_FALSE(presets.empty());
    app::EntryHeadlessRequest request;
    request.width = kWidth;
    request.height = kHeight;
    auto frame = app::render_entry_headless(presets.front().setup, request);
    if (!frame && frame.error().kind == app::HeadlessErrorKind::no_context) {
        if (environment_flag("ORRERY_REQUIRE_GL")) {
            FAIL("OpenGL 4.6 required (ORRERY_REQUIRE_GL=1) but unavailable: "
                 << frame.error().message);
        }
        SKIP("no OpenGL 4.6 core context: " << frame.error().message);
    }
    INFO((frame ? std::string() : frame.error().message));
    REQUIRE(frame.has_value());
    const render::ImageRgba8& image = frame.value().image;
    REQUIRE(image.width == kWidth);
    REQUIRE(image.height == kHeight);
    const sim::EntrySummary& summary = frame.value().run.summary;
    std::cout << "measured: Apollo-like entry: " << sim::to_string(summary.outcome) << ", peak "
              << summary.peak_g << " g, peak heat flux " << summary.peak_heat_flux_w_m2
              << " W/m2\n";
    CHECK(summary.outcome == sim::EntryOutcome::landed);
    CHECK(summary.peak_g > 6.0);
    CHECK(summary.peak_g < 8.0);

    // The scene (centre) holds the colour-mapped ribbon and legend; the
    // panels on both sides carry light text.
    CHECK(saturated_fraction(image, 380, 80, 1200, 560) > 0.002);
    std::size_t text = 0;
    for (std::uint32_t y = 60; y < 880; ++y) {
        for (std::uint32_t x = 1230; x < 1590; ++x) {
            text += luma(image, x, y) > 150.0 ? 1U : 0U;
        }
    }
    CHECK(text > 1500);
    check_golden(image, "entry_workspace_apollo");
}
