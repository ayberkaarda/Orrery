#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <stdexcept>
#include <vector>

#include "orrery/aero/panel_method.h"
#include "orrery/geom/naca.h"
#include "orrery/geom/primitives.h"

using namespace orrery::aero;
using orrery::geom::Vector3;

namespace {

constexpr double pi = std::numbers::pi;
constexpr double degree = pi / 180.0;

FlowConditions low_speed(double alpha_deg) {
    FlowConditions c;
    c.speed = 50.0; // Mach 0.15, Re 3.4e6 per metre
    c.angle_of_attack = alpha_deg * degree;
    return c;
}

// Brute-force integrals over a triangle: 4^levels sub-triangles, centroid
// rule.
struct Brute {
    double source = 0.0;
    double doublet = 0.0;
};

void brute_add(Brute& out, const Vector3& p, const Vector3& n, const Vector3& a, const Vector3& b,
               const Vector3& c, int levels) {
    if (levels == 0) {
        const Vector3 q = (a + b + c) / 3.0;
        const double area = 0.5 * orrery::geom::norm(orrery::geom::cross(b - a, c - a));
        const Vector3 d = p - q;
        const double r = orrery::geom::norm(d);
        out.source += -area / (4.0 * pi * r);
        out.doublet += area * orrery::geom::dot(d, n) / (4.0 * pi * r * r * r);
        return;
    }
    const Vector3 ab = (a + b) * 0.5;
    const Vector3 bc = (b + c) * 0.5;
    const Vector3 ca = (c + a) * 0.5;
    brute_add(out, p, n, a, ab, ca, levels - 1);
    brute_add(out, p, n, ab, b, bc, levels - 1);
    brute_add(out, p, n, ca, bc, c, levels - 1);
    brute_add(out, p, n, ab, bc, ca, levels - 1);
}

AeroGeometry circle_section(double radius, int panels) {
    AeroGeometry g;
    g.section.reserve(static_cast<std::size_t>(panels));
    for (int k = 0; k < panels; ++k) {
        const double theta = 2.0 * pi * k / panels;
        g.section.push_back({radius * std::cos(theta), radius * std::sin(theta)});
    }
    g.reference = {2.0 * radius, 2.0 * radius, {}};
    return g;
}

} // namespace

TEST_CASE("triangle influence matches brute-force integration", "[aero][panel]") {
    const Vector3 a{0.0, 0.0, 0.0};
    const Vector3 b{1.0, 0.0, 0.0};
    const Vector3 c{0.2, 0.8, 0.0};
    const Vector3 n{0.0, 0.0, 1.0};
    const std::vector<Vector3> points{
        {0.4, 0.3, 0.5}, {0.4, 0.3, -0.3}, {1.5, 1.2, 0.2}, {-0.8, 0.5, 0.0}, {3.0, -2.0, 1.5}};
    for (const Vector3& p : points) {
        Brute ref;
        brute_add(ref, p, n, a, b, c, 7);
        const PanelInfluence got = triangle_influence(p, a, b, c);
        INFO("p = " << p.x << ", " << p.y << ", " << p.z);
        CHECK(std::abs(got.source - ref.source) < 1e-4 * std::abs(ref.source));
        CHECK(std::abs(got.doublet - ref.doublet) < 1e-4 * std::abs(ref.doublet) + 1e-9);
    }
    // Just above and below the panel interior the doublet tends to +-1/2.
    CHECK(std::abs(triangle_influence({0.4, 0.3, 1e-9}, a, b, c).doublet - 0.5) < 1e-6);
    CHECK(std::abs(triangle_influence({0.4, 0.3, -1e-9}, a, b, c).doublet + 0.5) < 1e-6);
    // In the plane outside the panel there is no doublet potential.
    CHECK(std::abs(triangle_influence({2.0, 2.0, 0.0}, a, b, c).doublet) < 1e-15);
}

TEST_CASE("flat-plate skin friction follows Blasius and Prandtl-Schlichting", "[aero][panel]") {
    CHECK(std::abs(flat_plate_skin_friction(1e4) - 1.328 / 100.0) < 1e-15);
    // 0.455 / 6^2.58 - 1700 / 1e6 = 0.002771.
    CHECK(std::abs(flat_plate_skin_friction(1e6) - (0.455 / std::pow(6.0, 2.58) - 1.7e-3)) < 1e-15);
    CHECK(std::abs(flat_plate_skin_friction(1e6) - 0.002771) < 1e-6);
    CHECK_THROWS_AS(flat_plate_skin_friction(0.0), std::invalid_argument);
}

TEST_CASE("2D panel method reproduces the circular cylinder Cp = 1 - 4 sin^2", "[aero][panel]") {
    PanelSolver2D solver;
    solver.setup(circle_section(0.5, 128), low_speed(0.0));
    REQUIRE(solver.step());
    const AeroResult& r = solver.result();
    REQUIRE(r.surface.cp.size() == 128);
    double worst = 0.0;
    for (std::size_t i = 0; i < r.surface.cp.size(); ++i) {
        const Vector3& p = r.surface.points[i];
        const double s = std::sin(std::atan2(p.z, p.x));
        worst = std::max(worst, std::abs(r.surface.cp[i] - (1.0 - 4.0 * s * s)));
    }
    INFO("worst |Cp - (1 - 4 sin^2)| = " << worst);
    CHECK(worst < 2e-3);
    CHECK(std::abs(r.cl) < 1e-10);
    CHECK(std::abs(r.cd) < 1e-10);
    CHECK(r.converged);
    CHECK(r.warnings.empty());
}

TEST_CASE("2D panel method: symmetric airfoil has no lift at zero incidence", "[aero][panel]") {
    PanelSolver2D solver;
    const auto naca0012 = *orrery::geom::Naca4::from_designation("0012");
    solver.setup(naca_section_geometry(naca0012, 1.0, 80), low_speed(0.0));
    solver.step();
    CHECK(std::abs(solver.result().cl) < 1e-10);
    CHECK(std::abs(solver.result().cm) < 1e-10);
    CHECK(std::abs(solver.circulation()) < 1e-9);

    // Lift from the pressure integral equals Kutta-Joukowski rho V Gamma.
    const FlowConditions c = low_speed(4.0);
    solver.setup(naca_section_geometry(naca0012, 1.0, 80), c);
    solver.step();
    const AeroResult& r = solver.result();
    const double cl_kj = 2.0 * solver.circulation() / (c.speed * 1.0);
    CHECK(r.cl > 0.4);
    CHECK(std::abs(r.cl - cl_kj) < 0.01 * cl_kj);
    // Inviscid closed body: drag vanishes up to the discretisation error.
    CHECK(std::abs(r.cd) < 0.005);
    // Friction is reported separately: Re = 3.4e6, Cf = 0.0034 on both faces.
    CHECK(r.cd_friction > 0.005);
    CHECK(r.cd_friction < 0.009);
    // Symmetric section: aerodynamic centre at the quarter chord.
    CHECK(std::abs(r.cm) < 0.01);
}

TEST_CASE("2D panel method rejects unusable input", "[aero][panel]") {
    PanelSolver2D solver;
    CHECK_THROWS_AS(solver.step(), std::logic_error);
    AeroGeometry g = circle_section(1.0, 16);
    CHECK_THROWS_AS(solver.setup(AeroGeometry{}, low_speed(0.0)), std::invalid_argument);
    FlowConditions still = low_speed(0.0);
    still.speed = 0.0;
    CHECK_THROWS_AS(solver.setup(g, still), std::invalid_argument);
    std::vector<orrery::geom::SectionPoint> clockwise(g.section.rbegin(), g.section.rend());
    g.section = clockwise;
    CHECK_THROWS_AS(solver.setup(g, low_speed(0.0)), std::invalid_argument);
}

TEST_CASE("2D panel method warns outside its envelope but still computes", "[aero][panel]") {
    PanelSolver2D solver;
    FlowConditions c = low_speed(14.0);
    c.speed = 200.0; // Mach 0.59
    c.sideslip = 2.0 * degree;
    const auto naca0012 = *orrery::geom::Naca4::from_designation("0012");
    solver.setup(naca_section_geometry(naca0012, 1.0, 40), c);
    REQUIRE(solver.step());
    const AeroResult& r = solver.result();
    CHECK(r.has_warning("panel.mach"));
    CHECK(r.has_warning("panel.angle_of_attack"));
    CHECK(r.has_warning("panel.sideslip_ignored"));
    CHECK(r.cl > 1.0);
}

namespace {

struct SphereError {
    double rms = 0.0;
    double worst = 0.0;
    std::size_t panels = 0;
};

SphereError sphere_cp_error(const AeroResult& r) {
    double sum2 = 0.0;
    SphereError e;
    e.panels = r.surface.cp.size();
    for (std::size_t i = 0; i < r.surface.cp.size(); ++i) {
        const Vector3& p = r.surface.points[i];
        const double c2 = p.x * p.x / orrery::geom::squared_norm(p);
        const double err = r.surface.cp[i] - (1.0 - 2.25 * (1.0 - c2));
        sum2 += err * err;
        e.worst = std::max(e.worst, std::abs(err));
    }
    e.rms = std::sqrt(sum2 / static_cast<double>(e.panels));
    return e;
}

} // namespace

TEST_CASE("3D panel method reproduces the sphere Cp = 1 - 9/4 sin^2", "[aero][panel]") {
    AeroGeometry g;
    g.reference = {pi, 2.0, {}};
    PanelSolver3D solver;
    g.mesh = orrery::geom::sphere_profile(1.0).mesh({16, 0.2});
    solver.setup(g, low_speed(0.0));
    REQUIRE(solver.step());
    const SphereError coarse = sphere_cp_error(solver.result());

    g.mesh = orrery::geom::sphere_profile(1.0).mesh({32, 0.1});
    solver.setup(g, low_speed(0.0));
    REQUIRE(solver.step());
    const AeroResult& r = solver.result();
    const SphereError fine = sphere_cp_error(r);
    INFO("coarse: " << coarse.panels << " panels, rms " << coarse.rms << ", worst " << coarse.worst
                    << "; fine: " << fine.panels << " panels, rms " << fine.rms << ", worst "
                    << fine.worst);
    // Constant-strength panels with a least-squares velocity: first order.
    CHECK(fine.rms < 0.75 * coarse.rms);
    CHECK(fine.rms < 0.03);
    CHECK(fine.worst < 0.08);
    // d'Alembert: no force on a closed body in potential flow.
    CHECK(std::abs(r.cd) < 0.01);
    CHECK(std::abs(r.cl) < 0.01);
    CHECK_FALSE(solver.used_gmres());
    CHECK(r.converged);
    CHECK(r.cd_induced == 0.0);
}

TEST_CASE("3D panel method: GMRES agrees with the dense LU solve", "[aero][panel]") {
    AeroGeometry g;
    g.mesh = orrery::geom::sphere_profile(1.0).mesh({16, 0.25});
    g.reference = {pi, 2.0, {}};
    PanelSolver3D dense;
    dense.setup(g, low_speed(3.0));
    dense.step();
    PanelOptions options;
    options.dense_limit = 0;
    PanelSolver3D iterative(options);
    iterative.setup(g, low_speed(3.0));
    iterative.step();
    REQUIRE(iterative.used_gmres());
    CHECK(iterative.result().converged);
    REQUIRE(dense.doublets().size() == iterative.doublets().size());
    double worst = 0.0;
    for (std::size_t i = 0; i < dense.doublets().size(); ++i) {
        worst = std::max(worst, std::abs(dense.doublets()[i] - iterative.doublets()[i]));
    }
    CHECK(worst < 1e-8);
    CHECK(iterative.result().iterations > 1);
}

TEST_CASE("3D panel method rejects unusable input", "[aero][panel]") {
    PanelSolver3D solver;
    CHECK_THROWS_AS(solver.step(), std::logic_error);
    CHECK_THROWS_AS(solver.setup(AeroGeometry{}, low_speed(0.0)), std::invalid_argument);
    AeroGeometry g;
    g.mesh = orrery::geom::cube_mesh(1.0);
    g.reference.area = 0.0;
    CHECK_THROWS_AS(solver.setup(g, low_speed(0.0)), std::invalid_argument);
}
