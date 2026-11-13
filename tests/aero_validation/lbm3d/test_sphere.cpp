// Validation: drag of a sphere against the standard drag curve (docs/lbm.md,
// "3D validation"). Reference: Schiller and Naumann (1933),
//   Cd = 24 / Re (1 + 0.15 Re^0.687),
// the correlation of the measured curve for Re below about 800 (Clift, Grace
// and Weber, Bubbles, Drops, and Particles, 1978, table 5.2). Direct
// simulations agree with it in the laminar range, e.g. Cd 0.656 at Re 300
// (Johnson and Patel 1999, J. Fluid Mech. 378:19), where the correlation
// gives 0.684. Acceptance (project specification, section 5.4): within 15
// percent of the correlation at Re 10, 100, 300 and 1000.
//
// The sphere is the orrery_geom tessellation (64 vertices per ring),
// voxelized by the IAeroSolver adapter, with the interpolated bounce-back
// wall placed from the narrow-band signed distance; free-slip sides, a
// gradual start and an absorbing layer before the outlet.
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <iostream>
#include <string>

#include "../../aero/lbm/lbm_gl_support.h"
#include "body_case.h"

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

BodyCase sphere_case(double reynolds, double cells, double upstream, double downstream,
                     double width, double duration, double average) {
    BodyCase c;
    c.reynolds = reynolds;
    c.cells = cells;
    c.upstream = upstream;
    c.downstream = downstream;
    c.width = width;
    c.duration = duration;
    c.average = average;
    return c;
}

void check_drag(const BodyResult& r, double reynolds) {
    const double reference = schiller_naumann(reynolds);
    INFO("Re " << reynolds << ": Cd " << r.cd << ", Schiller-Naumann " << reference << ", ratio "
               << r.cd / reference);
    std::cout << "  Schiller-Naumann " << reference << ", ratio " << r.cd / reference << "\n";
    CHECK(std::abs(r.cd / reference - 1.0) < 0.15);
}

void run_gpu_case(const BodyCase& c, bool steady) {
    test::GlCompute gl = test::gl_compute_or_skip();
    const BodyResult r = run_body(sphere_geometry(), c, gl.backend.get(), "sphere GPU");
    check_drag(r, c.reynolds);
    if (steady) {
        CHECK(r.converged);
    }
    CHECK(gl.context.debug_error_count() == 0);
}

} // namespace

TEST_CASE("Sphere drag at Re 100 follows Schiller-Naumann (CPU)", "[aero][lbm3d][validation]") {
    if (!optimised_build()) {
        SKIP("the sphere validation runs in optimised builds (ci-headless)");
    }
    // The CI case: 12 cells per diameter, 3 D ahead, 6 D behind, 5 D wide.
    const BodyCase c = sphere_case(100.0, 12.0, 3.0, 6.0, 5.0, 25.0, 5.0);
    const BodyResult r = run_body(sphere_geometry(), c, nullptr, "sphere CPU");
    check_drag(r, c.reynolds);
    CHECK(r.converged); // steady axisymmetric wake
}

// Re 10: the disturbance of a slow sphere reaches far, so the channel is
// wider and longer.
TEST_CASE("Sphere drag at Re 10 follows Schiller-Naumann (GPU)", "[aero][lbm3d][validation][gl]") {
    run_gpu_case(sphere_case(10.0, 12.0, 6.0, 10.0, 10.0, 30.0, 10.0), true);
}

TEST_CASE("Sphere drag at Re 100 follows Schiller-Naumann (GPU)", "[aero][lbm3d][validation][gl]") {
    run_gpu_case(sphere_case(100.0, 16.0, 4.0, 8.0, 6.0, 30.0, 10.0), true);
}

// Re 300: an unsteady wake; the mean drag over 10 convective times.
TEST_CASE("Sphere drag at Re 300 follows Schiller-Naumann (GPU)", "[aero][lbm3d][validation][gl]") {
    run_gpu_case(sphere_case(300.0, 16.0, 4.0, 8.0, 6.0, 30.0, 10.0), false);
}

// Re 1000: the boundary layer (of order D / sqrt(Re)) needs 48 cells per
// diameter; there BGK is stable without the Smagorinsky closure, which only
// added dissipation at coarser resolutions (docs/lbm.md).
TEST_CASE("Sphere drag at Re 1000 follows Schiller-Naumann (GPU)",
          "[aero][lbm3d][validation][gl]") {
    run_gpu_case(sphere_case(1000.0, 48.0, 2.0, 4.0, 4.0, 20.0, 10.0), false);
}

TEST_CASE("Sphere and plate resolution study (GPU)", "[.study][aero][lbm3d][gl]") {
    test::GlCompute gl = test::gl_compute_or_skip();
    struct Row {
        bool plate;
        double reynolds;
        double cells;
        double upstream;
        double downstream;
        double width;
        double duration;
        double average;
        bool les;
        SolidWall wall;
    };
    constexpr SolidWall halfway = SolidWall::halfway;
    constexpr SolidWall interpolated = SolidWall::interpolated;
    for (const Row& row : {
             Row{false, 10.0, 16.0, 4.0, 8.0, 6.0, 30.0, 10.0, false, interpolated},
             Row{false, 10.0, 16.0, 6.0, 10.0, 10.0, 30.0, 10.0, false, interpolated},
             Row{false, 100.0, 16.0, 4.0, 8.0, 6.0, 30.0, 10.0, false, halfway},
             Row{false, 100.0, 24.0, 4.0, 8.0, 6.0, 30.0, 10.0, false, halfway},
             Row{false, 100.0, 10.0, 3.0, 6.0, 5.0, 25.0, 8.0, false, interpolated},
             Row{false, 100.0, 24.0, 4.0, 8.0, 6.0, 30.0, 10.0, false, interpolated},
             Row{false, 300.0, 24.0, 3.0, 6.0, 5.0, 30.0, 12.0, false, interpolated},
             Row{false, 1000.0, 32.0, 2.0, 4.0, 4.0, 24.0, 12.0, true, interpolated},
             Row{false, 1000.0, 32.0, 2.0, 4.0, 4.0, 24.0, 12.0, false, interpolated},
             Row{false, 1000.0, 40.0, 2.0, 4.0, 4.0, 20.0, 10.0, false, interpolated},
             Row{true, 1000.0, 16.0, 4.0, 8.0, 6.0, 40.0, 20.0, false, interpolated},
             Row{true, 1000.0, 32.0, 3.0, 6.0, 5.0, 30.0, 15.0, false, interpolated},
             Row{true, 5000.0, 24.0, 3.0, 6.0, 5.0, 30.0, 15.0, true, interpolated},
         }) {
        BodyCase c = sphere_case(row.reynolds, row.cells, row.upstream, row.downstream, row.width,
                                 row.duration, row.average);
        c.les = row.les ? LesMode::on : LesMode::off;
        c.wall = row.wall;
        const std::string label = std::string(row.plate ? "plate" : "sphere") + " GPU" +
                                  (row.wall == halfway ? " halfway" : "");
        const BodyResult r =
            run_body(row.plate ? square_plate_geometry(1.0 / 16.0) : sphere_geometry(), c,
                     gl.backend.get(), label);
        if (!row.plate) {
            std::cout << "  Schiller-Naumann " << schiller_naumann(row.reynolds) << ", ratio "
                      << r.cd / schiller_naumann(row.reynolds) << "\n";
        }
    }
}
