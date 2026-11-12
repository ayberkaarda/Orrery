#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <vector>

#include <glm/ext/vector_double3.hpp>
#include <glm/geometric.hpp>

#include "orrery/render/starfield.h"

using namespace orrery::render;
using Catch::Approx;

TEST_CASE("The starfield is deterministic per seed", "[render][starfield]") {
    StarfieldParams params;
    params.count = 2000;
    const auto a = generate_starfield(params);
    const auto b = generate_starfield(params);
    REQUIRE(a.size() == 2000);
    bool identical = true;
    for (std::size_t i = 0; i < a.size(); ++i) {
        identical = identical && a[i].direction == b[i].direction &&
                    a[i].magnitude == b[i].magnitude && a[i].color == b[i].color;
    }
    CHECK(identical);
    params.seed += 1;
    const auto c = generate_starfield(params);
    CHECK(c[0].direction != a[0].direction);
    // The generator itself is the reference SplitMix64 sequence.
    SplitMix64 rng(0);
    CHECK(rng.next() == 0xE220A8397B1DCDAFULL);
}

TEST_CASE("Star magnitudes follow the cumulative count slope", "[render][starfield]") {
    const StarfieldParams params; // 1e5 stars, -1.5 .. 10, slope 0.35
    const auto stars = generate_starfield(params);
    REQUIRE(stars.size() == 100000);
    std::size_t brighter_than_6 = 0;
    std::size_t brighter_than_8 = 0;
    bool in_range = true;
    bool unit = true;
    for (const StarPoint& s : stars) {
        in_range = in_range && s.magnitude >= -1.5F && s.magnitude <= 10.0F;
        unit = unit && std::abs(glm::length(glm::dvec3(s.direction)) - 1.0) < 1.0e-6;
        brighter_than_6 += s.magnitude < 6.0F ? 1U : 0U;
        brighter_than_8 += s.magnitude < 8.0F ? 1U : 0U;
    }
    CHECK(in_range);
    CHECK(unit);
    // N(<m) ~ 10^(0.35 m): N(<6) / N(<10) = 10^(-1.4) = 0.0398, N(<8) / N(<10) = 0.200.
    CHECK(static_cast<double>(brighter_than_6) / 1.0e5 == Approx(0.0398).epsilon(0.08));
    CHECK(static_cast<double>(brighter_than_8) / 1.0e5 == Approx(0.200).epsilon(0.04));
    // About 4000 naked-eye stars, as on the real sky (order of magnitude).
    CHECK(brighter_than_6 > 2500);
    CHECK(brighter_than_6 < 6000);
}

TEST_CASE("Faint stars concentrate towards the galactic plane", "[render][starfield]") {
    const auto stars = generate_starfield(StarfieldParams{});
    const glm::dmat3 to_galactic = ecliptic_to_galactic();
    std::size_t faint = 0;
    std::size_t faint_low = 0;
    std::size_t centre = 0;
    std::size_t anticentre = 0;
    for (const StarPoint& s : stars) {
        const glm::dvec3 g = to_galactic * glm::dvec3(s.direction);
        if (s.magnitude > 8.0F) {
            ++faint;
            faint_low += std::abs(g.z) < std::sin(0.1745) ? 1U : 0U; // |b| < 10 deg
        }
        if (std::abs(g.z) < 0.17) {
            centre += g.x > 0.9 ? 1U : 0U;
            anticentre += g.x < -0.9 ? 1U : 0U;
        }
    }
    // An isotropic sky has 17.4 percent of its stars within 10 degrees of the plane.
    CHECK(static_cast<double>(faint_low) / static_cast<double>(faint) > 0.4);
    CHECK(centre > anticentre);
}

TEST_CASE("The galactic frame matches the IAU definition", "[render][starfield]") {
    const glm::dmat3 m = ecliptic_to_galactic();
    // Orthonormal rotation.
    const glm::dmat3 identity = m * glm::transpose(m);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            CHECK(identity[i][j] == Approx(i == j ? 1.0 : 0.0).margin(1e-12));
        }
    }
    // The north galactic pole lies at ecliptic latitude +29.81 deg, longitude 180.02 deg.
    const glm::dvec3 pole = glm::transpose(m) * glm::dvec3(0.0, 0.0, 1.0);
    CHECK(std::asin(pole.z) * 180.0 / 3.141592653589793 == Approx(29.81).margin(0.02));
    CHECK(std::atan2(pole.y, pole.x) * 180.0 / 3.141592653589793 + 360.0 ==
          Approx(180.02).margin(0.05));
    // The galactic centre lies at ecliptic latitude -5.54 deg, longitude 266.84 deg.
    const glm::dvec3 centre = glm::transpose(m) * glm::dvec3(1.0, 0.0, 0.0);
    CHECK(std::asin(centre.z) * 180.0 / 3.141592653589793 == Approx(-5.54).margin(0.03));
    CHECK(std::atan2(centre.y, centre.x) * 180.0 / 3.141592653589793 + 360.0 ==
          Approx(266.84).margin(0.05));
}

TEST_CASE("Star colours cover the spectral classes", "[render][starfield]") {
    StarfieldParams params;
    params.count = 20000;
    const auto stars = generate_starfield(params);
    std::size_t red = 0;
    std::size_t blue = 0;
    for (const StarPoint& s : stars) {
        CHECK(s.temperature >= 2400.0F);
        CHECK(s.temperature <= 45000.0F);
        red += s.color.r > s.color.b ? 1U : 0U;
        blue += s.color.b > s.color.r ? 1U : 0U;
    }
    // Both warm (K, M, G) and hot (O, B, A) stars are well represented.
    CHECK(red > 20000 / 3);
    CHECK(blue > 20000 / 5);
}
