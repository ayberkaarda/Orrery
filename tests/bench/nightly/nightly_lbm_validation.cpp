// Large lattice Boltzmann validation on the CPU, run by the nightly
// workflow (docs/ci.md) and never by ctest: the CI runners have no GPU, and
// the pull-request suite keeps only the small CPU cases of
// tests/aero_validation. These are the GPU validation and study grids run on
// the CPU solvers instead, with the acceptance ranges of spec section 8:
//   - cylinder Re 150: Strouhal number in [0.16, 0.20];
//   - cylinder Re 100 at twice the CI resolution: St in [0.16, 0.20], mean
//     Cd in [1.2, 1.5] (section 5.4);
//   - sphere Re 100 at 16 cells per diameter: Cd in [1.0, 1.2];
//   - sphere Re 10 (12 cells per diameter, wider channel) and Re 300 (16
//     cells per diameter): within 15 percent of Schiller-Naumann (section
//     5.4).
// Each case prints a "measured:" line (cylinder_case.h, body_case.h).
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <iostream>

#include "../../aero_validation/lbm2d/cylinder_case.h"
#include "../../aero_validation/lbm3d/body_case.h"

using namespace orrery::aero::lbm;
using namespace orrery::aero::lbm::validation;

namespace {

bool optimised_build() {
#ifdef NDEBUG
    return true;
#else
    return false;
#endif
}

CylinderResult run_cpu_cylinder(double reynolds, double diameter, double height) {
    CylinderCase c;
    c.reynolds = reynolds;
    c.diameter = diameter;
    c.upstream = 8.0;
    c.downstream = 22.0;
    c.height = height;
    c.steps = static_cast<std::size_t>(2500.0 * diameter);
    const test::CylinderSetup setup = make_cylinder(c);
    auto solver = test::take(make_cpu_solver(setup.config, setup.mask));
    return run_cylinder(*solver, c, setup, "CPU nightly");
}

void check_shedding(const CylinderResult& r) {
    INFO("St " << r.strouhal << " mean Cd " << r.mean_drag << " cycles " << r.cycles);
    CHECK(r.cycles >= 5);
    CHECK(r.period_spread < 0.05);
    CHECK(r.strouhal >= 0.16);
    CHECK(r.strouhal <= 0.20);
}

BodyCase sphere_case(double reynolds, double cells, double upstream, double downstream,
                     double width) {
    BodyCase c;
    c.reynolds = reynolds;
    c.cells = cells;
    c.upstream = upstream;
    c.downstream = downstream;
    c.width = width;
    c.duration = 30.0;
    c.average = 10.0;
    return c;
}

double sphere_cd(const BodyCase& c, bool steady) {
    const BodyResult r = run_body(sphere_geometry(), c, nullptr, "sphere CPU nightly");
    const double reference = schiller_naumann(c.reynolds);
    std::cout << "  Schiller-Naumann " << reference << ", ratio " << r.cd / reference << "\n";
    if (steady) {
        CHECK(r.converged);
    }
    CHECK(std::isfinite(r.cd));
    return r.cd;
}

} // namespace

TEST_CASE("Cylinder at Re 150 sheds within the section 8 Strouhal range (CPU)",
          "[nightly][lbm][cylinder]") {
    REQUIRE(optimised_build());
    check_shedding(run_cpu_cylinder(150.0, 40.0, 16.0));
}

TEST_CASE("Cylinder at Re 100 at twice the CI resolution (CPU)", "[nightly][lbm][cylinder]") {
    REQUIRE(optimised_build());
    const CylinderResult r = run_cpu_cylinder(100.0, 40.0, 16.0);
    check_shedding(r);
    CHECK(r.mean_drag >= 1.2);
    CHECK(r.mean_drag <= 1.5);
}

TEST_CASE("Sphere drag at Re 100, 16 cells per diameter, within the section 8 range (CPU)",
          "[nightly][lbm3d][sphere]") {
    REQUIRE(optimised_build());
    const double cd = sphere_cd(sphere_case(100.0, 16.0, 4.0, 8.0, 6.0), true);
    CHECK(cd >= 1.0);
    CHECK(cd <= 1.2);
}

TEST_CASE("Sphere drag at Re 10 follows Schiller-Naumann (CPU)", "[nightly][lbm3d][sphere]") {
    REQUIRE(optimised_build());
    const BodyCase c = sphere_case(10.0, 12.0, 6.0, 10.0, 10.0);
    CHECK(std::abs(sphere_cd(c, true) / schiller_naumann(10.0) - 1.0) < 0.15);
}

TEST_CASE("Sphere drag at Re 300 follows Schiller-Naumann (CPU)", "[nightly][lbm3d][sphere]") {
    REQUIRE(optimised_build());
    const BodyCase c = sphere_case(300.0, 16.0, 4.0, 8.0, 6.0);
    CHECK(std::abs(sphere_cd(c, false) / schiller_naumann(300.0) - 1.0) < 0.15);
}
