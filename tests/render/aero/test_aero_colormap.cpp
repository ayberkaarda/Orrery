#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <glm/geometric.hpp>

#include "orrery/render/aero_colormap.h"
#include "orrery/render/tone_map.h"

// Colour maps of the aero visualisation (docs/aero-visualisation.md, "Colour
// maps"): perceptual lightness, symmetry, colour-vision safety, the display
// pre-compensation and the legend ticks.

using namespace orrery::render;

namespace {

constexpr std::size_t kSamples = 513;

double oklab_distance(const glm::dvec3& a, const glm::dvec3& b) {
    return glm::length(oklab_from_linear_srgb(a) - oklab_from_linear_srgb(b));
}

// Machado, Oliveira and Fernandes (2009) dichromat simulation, severity 1,
// applied to linear-light sRGB.
glm::dvec3 simulate(const glm::dvec3& c, bool protan) {
    if (protan) {
        return {0.152286 * c.x + 1.052583 * c.y - 0.204868 * c.z,
                0.114503 * c.x + 0.786281 * c.y + 0.099216 * c.z,
                -0.003882 * c.x - 0.048116 * c.y + 1.051998 * c.z};
    }
    return {0.367322 * c.x + 0.860646 * c.y - 0.227968 * c.z,
            0.280085 * c.x + 0.672501 * c.y + 0.047413 * c.z,
            -0.011820 * c.x + 0.042940 * c.y + 0.968881 * c.z};
}

} // namespace

TEST_CASE("Oklab conversions round-trip linear sRGB", "[render][aero][colormap]") {
    for (const glm::dvec3 c : {glm::dvec3(0.0), glm::dvec3(1.0), glm::dvec3(0.2, 0.5, 0.9),
                               glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(0.03, 0.7, 0.1)}) {
        const glm::dvec3 back = linear_srgb_from_oklab(oklab_from_linear_srgb(c));
        CHECK(glm::length(back - c) < 1.0e-6);
    }
    CHECK(oklab_from_linear_srgb(glm::dvec3(1.0)).x == Catch::Approx(1.0).margin(1.0e-6));
    CHECK(cie_lightness(glm::dvec3(1.0)) == Catch::Approx(100.0).margin(1.0e-3));
    CHECK(cie_lightness(glm::dvec3(0.0)) == Catch::Approx(0.0).margin(1.0e-9));
}

TEST_CASE("Diverging map: monotone lightness on each arm and symmetric endpoints",
          "[render][aero][colormap]") {
    std::vector<double> lightness;
    for (std::size_t i = 0; i < kSamples; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(kSamples - 1);
        const glm::dvec3 c = colormap_linear(Colormap::diverging, t);
        CHECK(c.x >= 0.0);
        CHECK(c.x <= 1.0);
        CHECK(c.z >= 0.0);
        CHECK(c.z <= 1.0);
        lightness.push_back(cie_lightness(c));
    }
    const std::size_t centre = kSamples / 2;
    for (std::size_t i = 1; i <= centre; ++i) {
        CHECK(lightness[i] > lightness[i - 1]); // rising towards the centre
    }
    for (std::size_t i = centre + 1; i < kSamples; ++i) {
        CHECK(lightness[i] < lightness[i - 1]); // falling towards the positive end
    }
    // Mirror symmetry of lightness: equal |t - 1/2| gives equal L*.
    for (std::size_t i = 0; i < centre; ++i) {
        CHECK(lightness[i] == Catch::Approx(lightness[kSamples - 1 - i]).margin(0.5));
    }
    // The endpoints: equal lightness and chroma, opposite hues (blue negative).
    const glm::dvec3 lo = oklab_from_linear_srgb(colormap_linear(Colormap::diverging, 0.0));
    const glm::dvec3 hi = oklab_from_linear_srgb(colormap_linear(Colormap::diverging, 1.0));
    CHECK(cie_lightness(colormap_linear(Colormap::diverging, 0.0)) ==
          Catch::Approx(cie_lightness(colormap_linear(Colormap::diverging, 1.0))).margin(1.0e-6));
    CHECK(std::hypot(lo.y, lo.z) == Catch::Approx(std::hypot(hi.y, hi.z)).margin(1.0e-3));
    CHECK(lo.z < 0.0); // blue (negative b)
    CHECK(hi.y > 0.0); // red (positive a)
    // A neutral centre.
    const glm::dvec3 mid = oklab_from_linear_srgb(colormap_linear(Colormap::diverging, 0.5));
    CHECK(std::hypot(mid.y, mid.z) < 1.0e-6);
    CHECK(mid.x > 0.9);
}

TEST_CASE("Diverging map: perceptually even steps", "[render][aero][colormap]") {
    // Equal steps in t give equal Oklab distances within each arm.
    std::vector<double> steps;
    for (std::size_t i = 1; i < kSamples; ++i) {
        const double t0 = static_cast<double>(i - 1) / static_cast<double>(kSamples - 1);
        const double t1 = static_cast<double>(i) / static_cast<double>(kSamples - 1);
        if (t0 < 0.5 && t1 > 0.5) {
            continue;
        }
        steps.push_back(oklab_distance(colormap_linear(Colormap::diverging, t0),
                                       colormap_linear(Colormap::diverging, t1)));
    }
    const auto [smallest, largest] = std::ranges::minmax_element(steps);
    std::cout << "measured: diverging map Oklab step ratio " << *largest / *smallest << "\n";
    CHECK(*largest / *smallest < 1.1);
}

TEST_CASE("Dark diverging map: lightness falls towards the centre on both arms",
          "[render][aero][colormap]") {
    std::vector<double> lightness;
    for (std::size_t i = 0; i < kSamples; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(kSamples - 1);
        lightness.push_back(cie_lightness(colormap_linear(Colormap::diverging_dark, t)));
    }
    const std::size_t centre = kSamples / 2;
    for (std::size_t i = 1; i <= centre; ++i) {
        CHECK(lightness[i] < lightness[i - 1]);
    }
    for (std::size_t i = centre + 1; i < kSamples; ++i) {
        CHECK(lightness[i] > lightness[i - 1]);
    }
    for (std::size_t i = 0; i < centre; ++i) {
        CHECK(lightness[i] == Catch::Approx(lightness[kSamples - 1 - i]).margin(0.5));
    }
    CHECK(lightness[centre] < 16.0);
    CHECK(lightness.front() == Catch::Approx(lightness.back()).margin(1.0e-6));
    const glm::dvec3 lo = oklab_from_linear_srgb(colormap_linear(Colormap::diverging_dark, 0.0));
    const glm::dvec3 hi = oklab_from_linear_srgb(colormap_linear(Colormap::diverging_dark, 1.0));
    CHECK(lo.z < 0.0);
    CHECK(hi.z > 0.0);
    CHECK(std::hypot(lo.y, lo.z) == Catch::Approx(std::hypot(hi.y, hi.z)).margin(1.0e-3));
}

TEST_CASE("Sequential map: monotone lightness and even steps", "[render][aero][colormap]") {
    double previous = -1.0;
    std::vector<double> steps;
    glm::dvec3 last = colormap_linear(Colormap::sequential, 0.0);
    for (std::size_t i = 0; i < kSamples; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(kSamples - 1);
        const glm::dvec3 c = colormap_linear(Colormap::sequential, t);
        const double l = cie_lightness(c);
        CHECK(l > previous);
        previous = l;
        if (i > 0) {
            steps.push_back(oklab_distance(last, c));
        }
        last = c;
    }
    const auto [smallest, largest] = std::ranges::minmax_element(steps);
    // Lightness is linear; the hue turn and the gamut limit vary the chroma
    // part of the step (measured 1.63).
    CHECK(*largest / *smallest < 1.75);
}

TEST_CASE("The maps stay distinguishable for protanopes and deuteranopes",
          "[render][aero][colormap]") {
    for (const bool protan : {true, false}) {
        INFO((protan ? "protanopia" : "deuteranopia"));
        // The two arms of the diverging map at the same distance from the
        // centre must not collapse onto one colour.
        for (const Colormap map : {Colormap::diverging, Colormap::diverging_dark}) {
            for (const double d : {0.15, 0.3, 0.5, 0.75, 1.0}) {
                const glm::dvec3 neg =
                    glm::clamp(simulate(colormap_linear(map, 0.5 - 0.5 * d), protan),
                               glm::dvec3(0.0), glm::dvec3(1.0));
                const glm::dvec3 pos =
                    glm::clamp(simulate(colormap_linear(map, 0.5 + 0.5 * d), protan),
                               glm::dvec3(0.0), glm::dvec3(1.0));
                INFO("map " << static_cast<int>(map) << " distance from the centre " << d);
                CHECK(oklab_distance(neg, pos) > 0.12 * d);
            }
        }
        // The sequential map keeps its lightness ordering.
        double previous = -1.0;
        for (std::size_t i = 0; i <= 32; ++i) {
            const glm::dvec3 c = glm::clamp(
                simulate(colormap_linear(Colormap::sequential, static_cast<double>(i) / 32.0),
                         protan),
                glm::dvec3(0.0), glm::dvec3(1.0));
            const double l = cie_lightness(c);
            CHECK(l > previous);
            previous = l;
        }
    }
}

TEST_CASE("Colour map tables and parameter clamping", "[render][aero][colormap]") {
    const std::vector<glm::dvec3> table = colormap_table(Colormap::diverging, 5);
    REQUIRE(table.size() == 5);
    CHECK(glm::length(table[0] - colormap_linear(Colormap::diverging, 0.0)) == 0.0);
    CHECK(glm::length(table[2] - colormap_linear(Colormap::diverging, 0.5)) == 0.0);
    CHECK(colormap_table(Colormap::sequential, 0).size() == 2);
    CHECK(glm::length(colormap_linear(Colormap::diverging, -3.0) -
                      colormap_linear(Colormap::diverging, 0.0)) == 0.0);
    CHECK(
        glm::length(colormap_linear(Colormap::diverging, std::numeric_limits<double>::quiet_NaN()) -
                    colormap_linear(Colormap::diverging, 0.5)) == 0.0);
    const glm::dvec3 s = colormap_srgb(Colormap::sequential, 0.7);
    const glm::dvec3 l = colormap_linear(Colormap::sequential, 0.7);
    CHECK(srgb_decode(s.x) == Catch::Approx(l.x).margin(1.0e-9));
}

TEST_CASE("Display pre-compensation inverts the ACES display transform",
          "[render][aero][colormap]") {
    for (const Colormap map :
         {Colormap::diverging, Colormap::diverging_dark, Colormap::sequential}) {
        for (std::size_t i = 0; i <= 64; ++i) {
            const glm::dvec3 display = colormap_linear(map, static_cast<double>(i) / 64.0);
            for (const double exposure : {1.0, 0.5}) {
                const glm::dvec3 scene = scene_linear_for_display(display, exposure);
                const glm::dvec3 shown = tone_map_aces(scene * exposure);
                INFO("t " << static_cast<double>(i) / 64.0 << " exposure " << exposure);
                // Within half an 8-bit step after encoding.
                CHECK(std::abs(srgb_encode(shown.x) - srgb_encode(display.x)) < 0.5 / 255.0);
                CHECK(std::abs(srgb_encode(shown.y) - srgb_encode(display.y)) < 0.5 / 255.0);
                CHECK(std::abs(srgb_encode(shown.z) - srgb_encode(display.z)) < 0.5 / 255.0);
            }
        }
    }
}

TEST_CASE("Colour scale ranges", "[render][aero][colormap]") {
    const std::vector<double> cp{-1.38, -0.4, 0.2, 1.0, std::numeric_limits<double>::infinity()};
    const ScalarRange symmetric = resolve_range({RangeMode::symmetric}, cp);
    CHECK(symmetric.min == Catch::Approx(-1.4));
    CHECK(symmetric.max == Catch::Approx(1.4));
    ColorScale exact{RangeMode::symmetric};
    exact.round_to_ticks = false;
    CHECK(resolve_range(exact, cp).max == Catch::Approx(1.38));
    const ScalarRange automatic = resolve_range({RangeMode::automatic}, cp);
    CHECK(automatic.min == Catch::Approx(-1.4));
    CHECK(automatic.max == Catch::Approx(1.0));
    const std::vector<double> speed{0.0, 0.0831};
    const ScalarRange s = resolve_range({RangeMode::automatic}, speed);
    CHECK(s.min == 0.0);
    CHECK(s.max == Catch::Approx(0.084));
    ColorScale fixed{RangeMode::fixed, -2.0, 3.0};
    CHECK(resolve_range(fixed, cp) == ScalarRange{-2.0, 3.0});
    fixed.min = 4.0; // invalid: falls back to automatic
    CHECK(resolve_range(fixed, cp) == automatic);
    CHECK(resolve_range({RangeMode::symmetric}, {}) == ScalarRange{-1.0, 1.0});
    CHECK(resolve_range({RangeMode::automatic}, {}) == ScalarRange{0.0, 1.0});
    const std::vector<double> flat{2.0, 2.0};
    const ScalarRange widened = resolve_range({RangeMode::automatic}, flat);
    CHECK(widened.min < 2.0);
    CHECK(widened.max > 2.0);
    CHECK(ScalarRange{-1.0, 1.0}.normalize(0.0) == 0.5);
    CHECK(ScalarRange{-1.0, 1.0}.normalize(5.0) == 1.0);
    CHECK(ScalarRange{1.0, 1.0}.normalize(5.0) == 0.5);
}

TEST_CASE("Legend ticks are round numbers with consistent labels", "[render][aero][colormap]") {
    const std::vector<LegendTick> cp = legend_ticks({-1.5, 1.5}, 5);
    std::vector<std::string> labels;
    labels.reserve(cp.size());
    for (const LegendTick& t : cp) {
        labels.push_back(t.label);
    }
    CHECK(labels == std::vector<std::string>{"-1.5", "-1.0", "-0.5", "0.0", "0.5", "1.0", "1.5"});
    CHECK(cp.front().position == 0.0);
    CHECK(cp.back().position == 1.0);
    CHECK(cp[3].position == Catch::Approx(0.5));

    const std::vector<LegendTick> speed = legend_ticks({0.0, 0.1}, 5);
    REQUIRE(!speed.empty());
    CHECK(speed.front().label == "0.00");
    CHECK(speed.back().label == "0.10");

    const std::vector<LegendTick> large = legend_ticks({0.0, 2.0e5}, 4);
    REQUIRE(large.size() >= 2);
    CHECK(large.back().label == "2.0e5");
    const std::vector<LegendTick> small = legend_ticks({-4.0e-4, 4.0e-4}, 5);
    REQUIRE(small.size() >= 3);
    CHECK(small.front().label == "-4e-4");
    CHECK(small[small.size() / 2].label == "0");

    CHECK(legend_ticks({1.0, 1.0}, 5).empty());
    const Legend legend = make_legend("Cp", Colormap::diverging, {-1.5, 1.5});
    CHECK(legend.title == "Cp");
    CHECK(legend.ticks.size() == 7);
}
