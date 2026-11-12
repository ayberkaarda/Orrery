#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <stdexcept>
#include <vector>

#include "geom_test_support.h"
#include "orrery/geom/mesh_repair.h"
#include "orrery/geom/primitives.h"

using namespace orrery::geom;
using orrery::geom::test::relative_error;

namespace {

constexpr double pi = std::numbers::pi;

// Acceptance: primitive volume and area against the analytic value.
constexpr double acceptance = 1e-9;

// 20-point Gauss-Legendre rule on [-1, 1] (nodes by Newton iteration).
struct Rule {
    std::array<double, 20> x{};
    std::array<double, 20> w{};
    Rule() {
        for (std::size_t i = 0; i < 20; ++i) {
            double t = std::cos(pi * (static_cast<double>(i) + 0.75) / 20.5);
            double dp = 1.0;
            for (int it = 0; it < 100; ++it) {
                double p0 = 1.0;
                double p1 = t;
                for (int k = 2; k <= 20; ++k) {
                    const double p2 = ((2.0 * k - 1.0) * t * p1 - (k - 1.0) * p0) / k;
                    p0 = p1;
                    p1 = p2;
                }
                dp = 20.0 * (t * p1 - p0) / (t * t - 1.0);
                t -= p1 / dp;
            }
            x[i] = t;
            w[i] = 2.0 / ((1.0 - t * t) * dp * dp);
        }
    }
};

template <typename F> double integrate(F f, double a, double b, int panels = 64) {
    static const Rule rule;
    const double h = (b - a) / panels;
    double sum = 0.0;
    for (int p = 0; p < panels; ++p) {
        const double lo = a + h * p;
        for (std::size_t i = 0; i < 20; ++i) {
            sum += 0.5 * h * rule.w[i] * f(lo + 0.5 * h * (rule.x[i] + 1.0));
        }
    }
    return sum;
}

// Independent quadrature of the profile: integral of pi r^2 dz and of
// 2 pi r ds along each segment, from point evaluations only.
struct Integrals {
    double volume = 0.0;
    double area = 0.0;
};

Integrals quadrature(const RevolutionProfile& profile) {
    Integrals out;
    for (const ProfileSegment& s : profile.segments()) {
        if (s.kind == ProfileSegment::Kind::line) {
            const double dr = s.end.r - s.start.r;
            const double dz = s.end.z - s.start.z;
            const double len = std::hypot(dr, dz);
            out.volume += integrate(
                [&](double t) {
                    const double r = s.start.r + dr * t;
                    return pi * r * r * dz;
                },
                0.0, 1.0);
            out.area += integrate([&](double t) { return 2.0 * pi * (s.start.r + dr * t) * len; },
                                  0.0, 1.0);
        } else {
            const double sign = s.phi1 > s.phi0 ? 1.0 : -1.0;
            out.volume += integrate(
                [&](double phi) {
                    const double r = s.centre.r + s.radius * std::cos(phi);
                    return pi * r * r * s.radius * std::cos(phi);
                },
                s.phi0, s.phi1);
            out.area +=
                sign * integrate(
                           [&](double phi) {
                               return 2.0 * pi * (s.centre.r + s.radius * std::cos(phi)) * s.radius;
                           },
                           s.phi0, s.phi1);
        }
    }
    return out;
}

void check_closed_outward(const TriangleMesh& m) {
    const WatertightReport r = analyze_mesh(m);
    CHECK(r.watertight());
    CHECK(r.outward());
    CHECK(r.degenerate_triangles == 0);
    CHECK(r.duplicate_triangles == 0);
    CHECK(r.duplicate_vertices == 0);
    CHECK(r.components == 1);
}

// Mesh volume and area equal the closed-form polyhedral values, and the
// polyhedral values converge to the analytic ones at second order.
void check_tessellation(const RevolutionProfile& profile, double analytic_volume,
                        double analytic_area) {
    double previous_error = 0.0;
    for (int around : {16, 32, 64, 128}) {
        const RevolutionResolution res{around, 2.0 * pi / around};
        const TriangleMesh m = profile.mesh(res);
        check_closed_outward(m);
        CHECK(relative_error(signed_volume(m), profile.polyhedral_volume(res)) < acceptance);
        CHECK(relative_error(surface_area(m), profile.polyhedral_area(res)) < acceptance);
        const double error = relative_error(profile.polyhedral_volume(res), analytic_volume);
        CHECK(profile.polyhedral_volume(res) < analytic_volume * (1.0 + 1e-15));
        if (previous_error > 0.0) {
            CHECK(error < 0.3 * previous_error);
        }
        previous_error = error;
        CHECK(relative_error(profile.polyhedral_area(res), analytic_area) < 2.0 * pi * pi / around);
    }
}

} // namespace

TEST_CASE("sphere, cylinder and cone match their closed forms", "[geom][primitives]") {
    const double r = 0.7;
    const double h = 1.9;
    const RevolutionProfile sphere = sphere_profile(r);
    CHECK(relative_error(sphere.volume(), 4.0 / 3.0 * pi * r * r * r) < acceptance);
    CHECK(relative_error(sphere.surface_area(), 4.0 * pi * r * r) < acceptance);
    const RevolutionProfile cylinder = cylinder_profile(r, h);
    CHECK(relative_error(cylinder.volume(), pi * r * r * h) < acceptance);
    CHECK(relative_error(cylinder.surface_area(), 2.0 * pi * r * (r + h)) < acceptance);
    const RevolutionProfile cone = cone_profile(r, h);
    CHECK(relative_error(cone.volume(), pi * r * r * h / 3.0) < acceptance);
    CHECK(relative_error(cone.surface_area(), pi * r * (r + std::hypot(r, h))) < acceptance);

    check_tessellation(sphere, 4.0 / 3.0 * pi * r * r * r, 4.0 * pi * r * r);
    check_tessellation(cylinder, pi * r * r * h, 2.0 * pi * r * (r + h));
    check_tessellation(cone, pi * r * r * h / 3.0, pi * r * (r + std::hypot(r, h)));
}

TEST_CASE("flat-faced primitives are exact", "[geom][primitives]") {
    const TriangleMesh box = box_mesh(1.5, 0.25, 3.0);
    check_closed_outward(box);
    CHECK(relative_error(signed_volume(box), 1.5 * 0.25 * 3.0) < acceptance);
    CHECK(relative_error(surface_area(box), 2.0 * (1.5 * 0.25 + 0.25 * 3.0 + 3.0 * 1.5)) <
          acceptance);

    const TriangleMesh cube = cube_mesh(2.0);
    check_closed_outward(cube);
    CHECK(relative_error(signed_volume(cube), 8.0) < acceptance);
    CHECK(relative_error(surface_area(cube), 24.0) < acceptance);

    const TriangleMesh plate = flat_plate_mesh(1.0, 4.0, 0.01);
    check_closed_outward(plate);
    CHECK(relative_error(signed_volume(plate), 0.04) < acceptance);
    CHECK(relative_error(surface_area(plate), 2.0 * (4.0 + 0.04 + 0.01)) < acceptance);

    for (double degrees : {5.0, 10.0, 30.0, 60.0}) {
        const Wedge wedge{2.0, degrees * pi / 180.0, 3.0};
        const TriangleMesh m = wedge.mesh();
        check_closed_outward(m);
        const double hh = 2.0 * std::tan(wedge.half_angle);
        CHECK(relative_error(wedge.volume(), 0.5 * 2.0 * (2.0 * hh) * 3.0) < acceptance);
        CHECK(relative_error(signed_volume(m), wedge.volume()) < acceptance);
        CHECK(relative_error(surface_area(m), wedge.surface_area()) < acceptance);
    }
}

TEST_CASE("Apollo and Orion capsules are tangent-continuous and exact", "[geom][primitives]") {
    for (const CapsuleShape& shape : {CapsuleShape::apollo(), CapsuleShape::orion()}) {
        REQUIRE_FALSE(shape.validate().has_value());
        const RevolutionProfile profile = capsule_profile(shape);
        REQUIRE_FALSE(profile.validate().has_value());

        // Analytic antiderivatives against an independent quadrature.
        const Integrals q = quadrature(profile);
        CHECK(relative_error(profile.volume(), q.volume) < 1e-12);
        CHECK(relative_error(profile.surface_area(), q.area) < 1e-12);

        // Joints: the tangent directions of consecutive arcs/lines agree.
        const std::vector<ProfileSegment>& s = profile.segments();
        REQUIRE(s.size() == 4);
        const double shield_end_tangent = s[0].phi1 + pi / 2.0;
        const double shoulder_start_tangent = s[1].phi0 + pi / 2.0;
        CHECK(std::abs(shield_end_tangent - shoulder_start_tangent) < 1e-12);
        const double cone_direction =
            std::atan2(s[2].end.z - s[2].start.z, s[2].end.r - s[2].start.r);
        CHECK(std::abs(cone_direction - (s[1].phi1 + pi / 2.0)) < 1e-12);

        // The shoulder reaches the maximum diameter.
        const TriangleMesh m = profile.mesh({128, 0.01});
        const Bounds b = bounds(m);
        CHECK(b.max.x <= 0.5 * shape.max_diameter);
        CHECK(b.max.x > 0.5 * shape.max_diameter - 1e-4);
        CHECK(b.min.z == 0.0);
        check_closed_outward(m);
        const RevolutionResolution res{128, 0.01};
        CHECK(relative_error(signed_volume(m), profile.polyhedral_volume(res)) < acceptance);
        CHECK(relative_error(surface_area(m), profile.polyhedral_area(res)) < acceptance);
        CHECK(relative_error(profile.polyhedral_volume(res), profile.volume()) < 1e-3);
    }
    // Apollo-like proportions: about 3.1 m tall.
    const RevolutionProfile apollo = capsule_profile(CapsuleShape::apollo());
    const TriangleMesh m = apollo.mesh({64, 0.05});
    CHECK(bounds(m).max.z > 2.9);
    CHECK(bounds(m).max.z < 3.3);
}

TEST_CASE("capsule parameters are validated", "[geom][primitives]") {
    CapsuleShape s = CapsuleShape::apollo();
    s.nose_radius = 1.0; // deeper than a hemisphere
    CHECK(s.validate().has_value());
    CHECK_THROWS_AS(capsule_profile(s), std::invalid_argument);
    s = CapsuleShape::apollo();
    s.top_diameter = 10.0;
    CHECK(s.validate().has_value());
    s = CapsuleShape::apollo();
    s.afterbody_half_angle = 2.0;
    CHECK(s.validate().has_value());
    CHECK_THROWS_AS(sphere_profile(-1.0), std::invalid_argument);
    CHECK_THROWS_AS(cylinder_profile(1.0, std::nan("")), std::invalid_argument);
    CHECK_THROWS_AS(box_mesh(1.0, 0.0, 1.0), std::invalid_argument);
    CHECK_THROWS_AS(sphere_profile(1.0).mesh({2, 0.1}), std::invalid_argument);
    const RevolutionProfile broken({ProfileSegment::line({0.0, 0.0}, {1.0, 0.0}),
                                    ProfileSegment::line({1.0, 0.5}, {0.0, 1.0})});
    CHECK(broken.validate().has_value());
}

TEST_CASE("Carlson integrals reproduce Carlson's test values", "[geom][primitives]") {
    // Carlson (1995), Numerical computation of real or complex elliptic
    // integrals, table values.
    CHECK(relative_error(carlson_rf(1.0, 2.0, 0.0), 1.3110287771461) < 1e-12);
    CHECK(relative_error(carlson_rf(2.0, 3.0, 4.0), 0.58408284167715) < 1e-12);
    CHECK(relative_error(carlson_rd(0.0, 2.0, 1.0), 1.7972103521034) < 1e-12);
    CHECK(relative_error(carlson_rd(2.0, 3.0, 4.0), 0.16510527294261) < 1e-12);
    CHECK(relative_error(carlson_rg(0.0, 16.0, 16.0), 3.1415926535898) < 1e-12);
    CHECK(relative_error(carlson_rg(2.0, 3.0, 4.0), 1.7255030280692) < 1e-12);
    CHECK_THROWS(carlson_rf(-1.0, 1.0, 1.0));
}

TEST_CASE("ellipsoid area and volume are exact", "[geom][primitives]") {
    // Sphere.
    CHECK(relative_error(Ellipsoid{2.0, 2.0, 2.0}.surface_area(), 16.0 * pi) < acceptance);
    // Prolate spheroid a = b < c.
    {
        const double a = 1.0;
        const double c = 3.0;
        const double e = std::sqrt(1.0 - a * a / (c * c));
        const double exact = 2.0 * pi * a * a * (1.0 + c / (a * e) * std::asin(e));
        CHECK(relative_error(Ellipsoid{a, a, c}.surface_area(), exact) < acceptance);
    }
    // Oblate spheroid a = b > c.
    {
        const double a = 2.0;
        const double c = 0.5;
        const double e = std::sqrt(1.0 - c * c / (a * a));
        const double exact = 2.0 * pi * a * a * (1.0 + (1.0 - e * e) / e * std::atanh(e));
        CHECK(relative_error(Ellipsoid{a, a, c}.surface_area(), exact) < acceptance);
    }
    // Triaxial: integral over u = cos(theta) in [-1, 1] and phi in [0, 2 pi)
    // of sqrt(b^2 c^2 (1-u^2) cos^2 + a^2 c^2 (1-u^2) sin^2 + a^2 b^2 u^2).
    {
        const Ellipsoid e{3.0, 2.0, 1.0};
        const int nphi = 256;
        double area = 0.0;
        for (int k = 0; k < nphi; ++k) {
            const double phi = 2.0 * pi * k / nphi;
            const double cs = std::cos(phi);
            const double sn = std::sin(phi);
            area += integrate(
                [&](double u) {
                    const double s2 = 1.0 - u * u;
                    return std::sqrt(e.b * e.b * e.c * e.c * s2 * cs * cs +
                                     e.a * e.a * e.c * e.c * s2 * sn * sn +
                                     e.a * e.a * e.b * e.b * u * u);
                },
                -1.0, 1.0, 16);
        }
        area *= 2.0 * pi / nphi;
        CHECK(relative_error(e.surface_area(), area) < acceptance);
        CHECK(relative_error(e.volume(), 8.0 * pi) < acceptance);

        const RevolutionResolution res{96, pi / 48.0};
        const TriangleMesh m = e.mesh(res);
        check_closed_outward(m);
        CHECK(relative_error(signed_volume(m), e.polyhedral_volume(res)) < acceptance);
        CHECK(relative_error(signed_volume(m), e.volume()) < 2e-3);
        CHECK(relative_error(surface_area(m), e.surface_area()) < 2e-3);
    }
}
