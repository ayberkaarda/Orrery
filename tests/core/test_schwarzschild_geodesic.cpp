#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>
#include <optional>
#include <vector>

#include "orrery/core/schwarzschild_geodesic.h"

// Reference maths of the black-hole renderer (docs/physics.md, "Black hole
// rendering"): light deflection, capture, the disk redshift factor and the
// thin-disk temperature profile, in geometrized units (lengths in M).

using namespace orrery::core::geodesic;
using Catch::Approx;

namespace {

constexpr double kPi = std::numbers::pi;

// Static camera on the +x axis at radius r0 and a ray leaving it in the x-y
// plane with impact parameter b, inward (towards the hole) or outward.
GeoVec ray_with_impact(double r0, double b, bool inward) {
    const double sin_a = b * std::sqrt(1.0 - 2.0 / r0) / r0;
    const double cos_a = std::sqrt(1.0 - sin_a * sin_a) * (inward ? -1.0 : 1.0);
    return {cos_a, sin_a, 0.0};
}

double angle_between(const GeoVec& a, const GeoVec& b) {
    const double dot = a.x * b.x + a.y * b.y + a.z * b.z;
    const double cx = a.y * b.z - a.z * b.y;
    const double cy = a.z * b.x - a.x * b.z;
    const double cz = a.x * b.y - a.y * b.x;
    return std::atan2(std::sqrt(cx * cx + cy * cy + cz * cz), dot);
}

} // namespace

TEST_CASE("Light deflection tends to the weak-field limit 4M/b", "[core][geodesic]") {
    for (const double b : {1000.0, 2000.0, 5000.0}) {
        INFO("b = " << b << " M");
        const std::optional<double> deflection = deflection_angle(b);
        REQUIRE(deflection.has_value());
        // First order: within the size of the second-order term.
        CHECK(*deflection / weak_field_deflection(b) == Approx(1.0).margin(4.0 / b));
        // Second order: the remainder is the third-order term 128 / (3 b^3)
        // plus the integration error of the default budget (RK4 with steps of
        // 4 pi / 512: about 1e-8 rad).
        CHECK(std::abs(*deflection - second_order_deflection(b)) <
              128.0 / (3.0 * b * b * b) + 2.0e-8);
    }
    // Closer in, the second-order term is resolved to a few percent.
    const std::optional<double> near = deflection_angle(200.0);
    REQUIRE(near.has_value());
    const double second = 15.0 * kPi / (4.0 * 200.0 * 200.0);
    CHECK((*near - weak_field_deflection(200.0)) / second == Approx(1.0).margin(0.05));
}

TEST_CASE("The critical impact parameter 3 sqrt(3) M separates capture from escape",
          "[core][geodesic]") {
    CHECK(kCriticalImpactParameter == Approx(3.0 * std::sqrt(3.0)).epsilon(1.0e-15));
    for (const double epsilon : {1.0e-2, 1.0e-3}) {
        INFO("b = b_c (1 +- " << epsilon << ")");
        CHECK_FALSE(deflection_angle(kCriticalImpactParameter * (1.0 - epsilon)).has_value());
        const std::optional<double> deflection =
            deflection_angle(kCriticalImpactParameter * (1.0 + epsilon));
        REQUIRE(deflection.has_value());
        // Strong-deflection limit (Bozza 2002): -ln(b / b_c - 1) + ln(216 (7 - 4 sqrt 3)) - pi.
        const double strong =
            -std::log(epsilon) + std::log(216.0 * (7.0 - 4.0 * std::sqrt(3.0))) - kPi;
        CHECK(*deflection == Approx(strong).margin(3.0 * epsilon + 0.002));
    }

    // From a camera at 40 M: inward rays just inside b_c fall in, just
    // outside they escape; outward rays always escape; radial rays go
    // straight in or out.
    const GeoVec camera{40.0, 0.0, 0.0};
    const DiskModel no_disk{{0.0, 0.0, 1.0}, 1.0e9, 2.0e9};
    for (const double factor : {0.5, 0.99, 0.999}) {
        const RayTrace inside = trace_ray(
            camera, ray_with_impact(40.0, kCriticalImpactParameter * factor, true), no_disk);
        CHECK(inside.fate == RayFate::captured);
        CHECK(inside.impact_parameter == Approx(kCriticalImpactParameter * factor).epsilon(1e-12));
    }
    for (const double factor : {1.001, 1.01, 2.0, 6.0}) {
        const RayTrace outside = trace_ray(
            camera, ray_with_impact(40.0, kCriticalImpactParameter * factor, true), no_disk);
        CHECK(outside.fate == RayFate::escaped);
    }
    for (const double factor : {0.1, 0.9, 3.0}) {
        const RayTrace outward = trace_ray(
            camera, ray_with_impact(40.0, kCriticalImpactParameter * factor, false), no_disk);
        CHECK(outward.fate == RayFate::escaped);
    }
    CHECK(trace_ray(camera, {-1.0, 0.0, 0.0}, no_disk).fate == RayFate::captured);
    const RayTrace radial_out = trace_ray(camera, {1.0, 0.0, 0.0}, no_disk);
    CHECK(radial_out.fate == RayFate::escaped);
    CHECK(radial_out.escape_direction.x == 1.0);
    CHECK(trace_ray({1.5, 0.0, 0.0}, {1.0, 0.0, 0.0}, no_disk).fate == RayFate::captured);
}

TEST_CASE("A distant camera sees a ray bent by the integrated deflection", "[core][geodesic]") {
    // Camera far out on +x, ray passing the hole with impact parameter b:
    // the escape direction is the reversed incoming direction turned by the
    // deflection angle (plus the small angle subtended by b at the camera).
    const double r0 = 1.0e6;
    const DiskModel no_disk{{0.0, 0.0, 1.0}, 1.0e9, 2.0e9};
    for (const double b : {8.0, 20.0, 100.0}) {
        INFO("b = " << b);
        const RayTrace trace = trace_ray({r0, 0.0, 0.0}, ray_with_impact(r0, b, true), no_disk);
        REQUIRE(trace.fate == RayFate::escaped);
        const std::optional<double> deflection = deflection_angle(b);
        REQUIRE(deflection.has_value());
        const double camera_angle = std::asin(b * std::sqrt(1.0 - 2.0 / r0) / r0);
        const double turned = kPi - angle_between(trace.escape_direction, {1.0, 0.0, 0.0});
        // Escape direction measured from -x: deflection minus the camera angle
        // (the ray starts tilted towards +y by that angle) to first order.
        CHECK(turned == Approx(*deflection - camera_angle).margin(2.0e-4 + 2.0 * b / r0));
    }
}

TEST_CASE("Disk redshift factor matches the emitter four-velocity", "[core][geodesic]") {
    for (const double r : {6.0, 9.0, 20.0, 60.0}) {
        for (const double lambda : {-12.0, -3.0, 0.0, 4.0, 7.0}) {
            for (const double r_obs : {50.0, 1.0e8}) {
                // Independent derivation: u^t from the normalisation of a
                // circular orbit, -(1 - 2/r) (u^t)^2 + r^2 Omega^2 (u^t)^2 = -1.
                const double omega = std::pow(r, -1.5);
                const double ut = 1.0 / std::sqrt(1.0 - 2.0 / r - r * r * omega * omega);
                const double emitted = ut * (1.0 - omega * lambda); // -p.u / E
                const double observed = 1.0 / std::sqrt(1.0 - 2.0 / r_obs);
                CHECK(disk_redshift_factor(r, lambda, r_obs) ==
                      Approx(observed / emitted).epsilon(1.0e-12));
            }
        }
        // Face-on at infinity: gravitational plus transverse Doppler shift.
        CHECK(disk_redshift_factor(r, 0.0, 1.0e300) == Approx(std::sqrt(1.0 - 3.0 / r)));
    }
    // Light emitted forwards and backwards along the orbit at the ISCO
    // (v = c/2 for a static observer there): sqrt(2) and sqrt(2)/3.
    const double b_forward = 6.0 / std::sqrt(1.0 - 2.0 / 6.0);
    CHECK(disk_redshift_factor(6.0, b_forward, 1.0e300) == Approx(std::sqrt(2.0)));
    CHECK(disk_redshift_factor(6.0, -b_forward, 1.0e300) == Approx(std::sqrt(2.0) / 3.0));
}

TEST_CASE("Rays meet the disk with the redshift of the side they see", "[core][geodesic]") {
    // Camera at 60 M on +x, 5 degrees above the disk plane. The gas orbits
    // counter-clockwise about +z, so at -y it moves towards +x (towards the
    // camera) and at +y away from it.
    const double elevation = 5.0 * kPi / 180.0;
    const GeoVec camera{60.0 * std::cos(elevation), 0.0, 60.0 * std::sin(elevation)};
    const DiskModel disk{{0.0, 0.0, 1.0}, kIscoRadius, 30.0};
    const auto first_hit = [&](double y) {
        const GeoVec direction{-camera.x, y, -camera.z};
        const RayTrace trace = trace_ray(camera, direction, disk);
        for (const PlaneCrossing& c : trace.crossings) {
            if (c.on_disk) {
                return c;
            }
        }
        FAIL("no disk crossing towards y = " << y);
        return PlaneCrossing{};
    };
    const PlaneCrossing approaching = first_hit(-15.0);
    const PlaneCrossing receding = first_hit(15.0);
    CHECK(approaching.radius == Approx(receding.radius).epsilon(1.0e-9));
    CHECK(approaching.g > 1.0);
    CHECK(receding.g < 0.85);
    CHECK(approaching.g / receding.g > 1.4);

    // A ray aimed just over the top of the hole crosses the plane behind it
    // (the far side of the disk lensed over the shadow); one aimed below the
    // hole passes under it and meets the disk again behind it, on its
    // second crossing.
    const RayTrace over = trace_ray(camera, {-camera.x, 0.0, -camera.z + 8.0}, disk);
    REQUIRE_FALSE(over.crossings.empty());
    CHECK(over.crossings.front().on_disk);
    const RayTrace under = trace_ray(camera, {-camera.x, 0.0, -camera.z - 7.0}, disk);
    REQUIRE(under.crossings.size() >= 2);
    CHECK_FALSE(under.crossings[0].on_disk); // inside the ISCO, in front of the hole
    CHECK(under.crossings[1].on_disk);
}

TEST_CASE("Thin-disk temperature peaks at 49/36 r_in and falls as r^-3/4", "[core][geodesic]") {
    const double r_in = kIscoRadius;
    CHECK(thin_disk_temperature(r_in, r_in) == 0.0);
    CHECK(thin_disk_temperature(0.5 * r_in, r_in) == 0.0);
    CHECK(thin_disk_peak_radius(r_in) == Approx(49.0 / 36.0 * r_in));
    CHECK(thin_disk_temperature(thin_disk_peak_radius(r_in), r_in) == Approx(1.0));
    for (const double x : {1.01, 1.2, 1.5, 2.0, 5.0, 30.0}) {
        CHECK(thin_disk_temperature(x * r_in, r_in) < 1.0);
    }
    // Far out: doubling r lowers T by 2^-3/4.
    const double far = 1.0e5 * r_in;
    CHECK(thin_disk_temperature(2.0 * far, r_in) / thin_disk_temperature(far, r_in) ==
          Approx(std::pow(2.0, -0.75)).epsilon(1.0e-3));
}

TEST_CASE("The step rule bounds the change of u for steep rays", "[core][geodesic]") {
    const double base = base_step(512);
    CHECK(base == Approx(4.0 * kPi / 512.0));
    CHECK(base_step(1) == base_step(kMinSteps));
    CHECK(base_step(100000) == base_step(kMaxSteps));
    CHECK(step_size(0.1, 0.01, base) == base);
    CHECK(step_size(0.1, 1.0e3, base) == Approx(kStepGrowth * 0.1 / 1.0e3));
    CHECK(step_size(1.0e-6, 1.0e3, base) == Approx(kStepGrowth * kStepReferenceU / 1.0e3));
    // Nearly radial infall (b = 1e-3) still finds the crossing of a plane
    // tilted against the ray accurately and is captured within the budget.
    const GeoVec camera{40.0, 0.0, 0.0};
    const RayTrace steep = trace_ray(camera, ray_with_impact(40.0, 1.0e-3, true),
                                     DiskModel{{0.0, 0.0, 1.0}, kIscoRadius, 30.0});
    CHECK(steep.fate == RayFate::captured);
    CHECK(steep.steps < 64);
}
