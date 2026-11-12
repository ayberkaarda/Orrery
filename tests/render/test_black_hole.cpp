#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>

#include <glm/geometric.hpp>

#include "black_hole_test_support.h"
#include "orrery/render/black_hole.h"
#include "orrery/render/blackbody_spectrum.h"
#include "orrery/render/body_visuals.h"
#include "orrery/render/render_settings.h"
#include "orrery/render/space_renderer.h"

// CPU side of the Phase 5 black-hole pass (docs/rendering.md, "Black hole"):
// the black-body spectrum that colours the disk, the choice of the lensed
// hole, its apparent size, and the settings. No OpenGL.

using namespace orrery;
using namespace orrery::render;
using Catch::Approx;

namespace {

// Planck B_nu [W m^-2 sr^-1 Hz^-1] from B_lambda: B_nu = B_lambda lambda^2 / c.
double planck_nu(double frequency, double kelvin) {
    constexpr double c = 299792458.0;
    const double wavelength = c / frequency;
    return planck_spectral_radiance(wavelength, kelvin) * wavelength * wavelength / c;
}

// Bolometric radiance: B_lambda integrated over 10 nm .. 10 mm on a log grid.
double bolometric(double kelvin) {
    const double lo = std::log(1.0e-8);
    const double hi = std::log(1.0e-2);
    constexpr int n = 20000;
    double sum = 0.0;
    for (int i = 0; i <= n; ++i) {
        const double l = std::exp(lo + (hi - lo) * i / n);
        const double w = (i == 0 || i == n) ? 0.5 : 1.0;
        sum += w * planck_spectral_radiance(l, kelvin) * l; // d lambda = lambda d ln lambda
    }
    return sum * (hi - lo) / n;
}

} // namespace

TEST_CASE("Black-body radiance keeps the Planckian chromaticity", "[render][black_hole]") {
    for (const double t : {2000.0, 3000.0, 4500.0, 6500.0, 10000.0, 20000.0}) {
        INFO(t << " K");
        const glm::dvec3 xyz = blackbody_xyz(t);
        const double x = xyz.x / (xyz.x + xyz.y + xyz.z);
        const double y = xyz.y / (xyz.x + xyz.y + xyz.z);
        // The locus fit of body_visuals (Kim et al. 2002), back to xy.
        const glm::dvec3 rgb = blackbody_linear_srgb(t);
        const glm::dvec3 ref{0.4124564 * rgb.r + 0.3575761 * rgb.g + 0.1804375 * rgb.b,
                             0.2126729 * rgb.r + 0.7151522 * rgb.g + 0.0721750 * rgb.b,
                             0.0193339 * rgb.r + 0.1191920 * rgb.g + 0.9503041 * rgb.b};
        const double rx = ref.x / (ref.x + ref.y + ref.z);
        const double ry = ref.y / (ref.x + ref.y + ref.z);
        CHECK(std::abs(x - rx) < 0.006);
        CHECK(std::abs(y - ry) < 0.006);
    }
    CHECK(blackbody_luminance(kBlackbodyReferenceKelvin) == Approx(1.0));
    // Hot is bluer, cool is redder, and both stay non-negative.
    const glm::dvec3 hot = blackbody_radiance_srgb(15000.0);
    const glm::dvec3 cool = blackbody_radiance_srgb(2500.0);
    CHECK(hot.b > hot.r);
    CHECK(cool.r > 4.0 * cool.b);
    CHECK(cool.b >= 0.0);
    CHECK(blackbody_radiance_srgb(0.0) == glm::dvec3(0.0));
}

TEST_CASE("Doppler shift of a black body: g^3 per frequency, g^4 bolometric",
          "[render][black_hole]") {
    // Liouville: I_nu / nu^3 is invariant, so the observed spectrum of a black
    // body at T seen with frequency ratio g is g^3 B_nu(nu / g; T), which is
    // exactly the black body at g T.
    for (const double g : {0.5, 0.8, 1.3, 1.9}) {
        for (const double nu : {3.0e14, 5.5e14, 7.5e14}) {
            const double shifted = g * g * g * planck_nu(nu / g, 6000.0);
            CHECK(shifted == Approx(planck_nu(nu, g * 6000.0)).epsilon(1.0e-9));
        }
        // Integrated over all frequencies: g^4 (Stefan-Boltzmann).
        CHECK(bolometric(g * 6000.0) / bolometric(6000.0) ==
              Approx(std::pow(g, 4.0)).epsilon(1e-4));
    }
    // In the visible band the boost is steeper than g^4 on the cool side
    // (Wien tail): this is why the receding side of a disk fades so strongly.
    const double visible = blackbody_luminance(0.7 * 6000.0) / blackbody_luminance(6000.0);
    CHECK(visible < std::pow(0.7, 4.0));
}

TEST_CASE("The black-body table is log-spaced and matches the direct values",
          "[render][black_hole]") {
    const BlackbodyTable table = make_blackbody_table(256, 1000.0, 100000.0);
    REQUIRE(table.texels.size() == 256);
    CHECK(table.texels.front().g == Approx(blackbody_radiance_srgb(1000.0).g).epsilon(1e-6));
    CHECK(table.texels.back().b == Approx(blackbody_radiance_srgb(100000.0).b).epsilon(1e-6));
    const double t = std::exp(std::log(1000.0) + 100.0 / 255.0 * std::log(100.0));
    CHECK(table.texels[100].r == Approx(blackbody_radiance_srgb(t).r).epsilon(1e-6));
}

TEST_CASE("The lensed hole is chosen by its Einstein angle on screen", "[render][black_hole]") {
    const auto hole = test::lone_hole();
    const FrameInfo frame{1280, 720};
    RenderSettings settings;

    const Camera close = test::hole_camera(60.0, 0.3, 0.1, 40.0);
    const std::optional<LensingTarget> target =
        select_lensed_black_hole(*hole, close, frame, settings);
    REQUIRE(target.has_value());
    CHECK(target->index == 0);
    CHECK(target->observer_radius == Approx(60.0).epsilon(1e-9));
    CHECK(target->mass_length == Approx(test::hole_mass_length()));
    const double pps = pixel_scale(close.lens().vertical_fov, frame.height);
    CHECK(target->shadow_radius_px ==
          Approx(std::tan(std::asin(std::sqrt(27.0) * std::sqrt(1.0 - 2.0 / 60.0) / 60.0)) * pps));

    // Switched off, or too far for a quarter-pixel Einstein angle: no pass.
    settings.black_hole_lensing = false;
    CHECK_FALSE(select_lensed_black_hole(*hole, close, frame, settings).has_value());
    settings.black_hole_lensing = true;
    const double limit = 4.0 / std::pow(kLensingMinEinsteinPx / pps, 2.0);
    CHECK(select_lensed_black_hole(*hole, test::hole_camera(0.5 * limit, 0.3, 0.1, 40.0), frame,
                                   settings)
              .has_value());
    CHECK_FALSE(select_lensed_black_hole(*hole, test::hole_camera(2.0 * limit, 0.3, 0.1, 40.0),
                                         frame, settings)
                    .has_value());

    // The body pass leaves the lensed hole to the lensing pass.
    const PreparedBodies lensed = prepare_bodies(*hole, close, frame, settings);
    CHECK(lensed.instances.empty());
    CHECK(lensed.glows.empty());
    REQUIRE(lensed.screen.size() == 1);
    CHECK(lensed.screen[0].visible);
    settings.black_hole_lensing = false;
    const PreparedBodies placeholder = prepare_bodies(*hole, close, frame, settings);
    CHECK(placeholder.instances.size() == 1);
    CHECK(placeholder.glows.size() == 1);
}

TEST_CASE("Shadow and Einstein angles follow the Schwarzschild formulas", "[render][black_hole]") {
    CHECK(shadow_angular_radius(3.0) == Approx(test::kPi / 2.0));
    CHECK(shadow_angular_radius(1.0e8) == Approx(std::sqrt(27.0) / 1.0e8).epsilon(1e-6));
    CHECK(shadow_angular_radius(2.5) > test::kPi / 2.0);
    CHECK(shadow_angular_radius(1.0) == Approx(test::kPi));
    CHECK(einstein_angle(400.0) == Approx(0.1));
}

TEST_CASE("Trace grid covers every pixel with four samples", "[render][black_hole]") {
    for (const FrameInfo frame : {FrameInfo{1920, 1080}, FrameInfo{641, 359}, FrameInfo{1, 1}}) {
        const TraceGrid grid = trace_grid(frame);
        // The last pixel centre (w - 0.5) lies between samples floor(t) and
        // floor(t) + 1 with t = (w - 0.5 - 1) / 2.
        const auto last_x = static_cast<int>(std::floor((frame.width - 1.5) / 2.0)) + 1;
        const auto last_y = static_cast<int>(std::floor((frame.height - 1.5) / 2.0)) + 1;
        CHECK(grid.width >= last_x + 1);
        CHECK(grid.height >= last_y + 1);
        CHECK(grid.width <= frame.width / 2 + 1);
    }
    CHECK(trace_sample_pixel(0, 0) == glm::dvec2(1.0, 1.0));
    CHECK(trace_sample_pixel(3, 5) == glm::dvec2(7.0, 11.0));
    // The centre of the image looks at the camera target.
    const Camera camera = test::hole_camera(60.0, 0.3, 0.1, 40.0);
    const glm::dvec3 ray = pixel_ray(camera, FrameInfo{640, 360}, glm::dvec2(320.0, 180.0));
    const glm::dvec3 to_hole = glm::normalize(camera.to_render_double(glm::dvec3(0.0)));
    CHECK(glm::dot(ray, to_hole) == Approx(1.0).epsilon(1e-12));
}

TEST_CASE("Black-hole settings are clamped into their ranges", "[render][black_hole]") {
    RenderSettings s;
    s.disk_brightness = -1.0;
    s.disk_temperature = 1.0e6;
    s.geodesic_steps = 4;
    RenderSettings clean = sanitized(s);
    CHECK(clean.disk_brightness == 0.0);
    CHECK(clean.disk_temperature == 20000.0);
    CHECK(clean.geodesic_steps == 64);
    s.disk_brightness = std::nan("");
    s.disk_temperature = std::nan("");
    s.geodesic_steps = 100000;
    clean = sanitized(s);
    CHECK(clean.disk_brightness == RenderSettings{}.disk_brightness);
    CHECK(clean.disk_temperature == RenderSettings{}.disk_temperature);
    CHECK(clean.geodesic_steps == 512);
}
