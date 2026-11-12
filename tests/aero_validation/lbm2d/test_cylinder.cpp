// Validation: laminar vortex shedding behind a circular cylinder at Re = 100
// (docs/lbm.md, "Validation"). Literature values at Re = 100 for the
// unconfined cylinder:
//   Strouhal number 0.164 (Williamson 1996, Annu. Rev. Fluid Mech. 28:477,
//     St = -3.3265/Re + 0.1816 + 1.6e-4 Re), 0.165 (Park, Kwon and Choi
//     1998, KSME Int. J. 12:1200), 0.16 (Braza, Chassaing and Ha Minh 1986,
//     J. Fluid Mech. 165:79);
//   mean drag coefficient 1.33 (Park et al. 1998), 1.35 (Henderson 1995,
//     Phys. Fluids 7:2102), 1.36 (Braza et al. 1986).
// Acceptance (project specification, section 5.4): St in [0.16, 0.20] and
// mean Cd in [1.2, 1.5].
#include <catch2/catch_test_macros.hpp>

#include <string>

#include "../../aero/lbm/lbm_gl_support.h"
#include "cylinder_case.h"

using namespace orrery::aero::lbm;
using namespace orrery::aero::lbm::validation;

namespace {

// The CI case: D = 20 cells (tau 0.53), inlet 8 D upstream, outlet 12 D
// downstream, a 20 D wide channel (blockage 5 percent), 26000 steps (about
// 11 shedding periods, the last half analysed).
CylinderCase ci_case() {
    CylinderCase c;
    c.reynolds = 100.0;
    c.diameter = 20.0;
    c.upstream = 8.0;
    c.downstream = 12.0;
    c.height = 20.0;
    c.steps = 26000;
    return c;
}

void check_literature(const CylinderResult& r) {
    INFO("St " << r.strouhal << " mean Cd " << r.mean_drag << " cycles " << r.cycles);
    CHECK(r.cycles >= 5);
    CHECK(r.period_spread < 0.05);
    CHECK(r.strouhal >= 0.16);
    CHECK(r.strouhal <= 0.20);
    CHECK(r.mean_drag >= 1.2);
    CHECK(r.mean_drag <= 1.5);
    CHECK(r.drag_moving_average >= 1.2);
    CHECK(r.drag_moving_average <= 1.5);
    CHECK(r.lift_amplitude > 0.1);
}

bool optimised_build() {
#ifdef NDEBUG
    return true;
#else
    return false;
#endif
}

} // namespace

TEST_CASE("Cylinder at Re 100 sheds at the literature Strouhal number and drag (CPU)",
          "[aero][lbm][validation]") {
    if (!optimised_build()) {
        SKIP("the cylinder validation runs in optimised builds (ci-headless)");
    }
    const CylinderCase c = ci_case();
    const test::CylinderSetup setup = make_cylinder(c);
    auto solver = test::take(make_cpu_solver(setup.config, setup.mask));
    check_literature(run_cylinder(*solver, c, setup, "CPU"));
}

TEST_CASE("Cylinder at Re 100 sheds at the literature Strouhal number and drag (GPU)",
          "[aero][lbm][validation][gl]") {
    test::GlCompute gl = test::gl_compute_or_skip();
    const CylinderCase c = ci_case();
    const test::CylinderSetup setup = make_cylinder(c);
    auto solver = test::take(make_gpu_solver(*gl.backend, setup.config, setup.mask));
    check_literature(run_cylinder(*solver, c, setup, "GPU"));
    CHECK(gl.context.debug_error_count() == 0);
}

TEST_CASE("Cylinder resolution and Reynolds number study (GPU)", "[.study][aero][lbm][gl]") {
    test::GlCompute gl = test::gl_compute_or_skip();
    struct Row {
        double reynolds;
        double diameter;
        double upstream;
        double downstream;
        double height;
        bool les;
    };
    for (const Row row :
         {Row{100.0, 12.0, 8.0, 22.0, 16.0, false}, Row{100.0, 20.0, 8.0, 12.0, 20.0, false},
          Row{100.0, 20.0, 8.0, 22.0, 16.0, false}, Row{100.0, 20.0, 8.0, 22.0, 16.0, true},
          Row{100.0, 40.0, 8.0, 22.0, 30.0, false}, Row{150.0, 20.0, 8.0, 22.0, 16.0, false},
          Row{150.0, 40.0, 8.0, 22.0, 30.0, false}, Row{200.0, 20.0, 8.0, 22.0, 16.0, false},
          Row{200.0, 40.0, 8.0, 22.0, 30.0, false}}) {
        CylinderCase c;
        c.reynolds = row.reynolds;
        c.diameter = row.diameter;
        c.upstream = row.upstream;
        c.downstream = row.downstream;
        c.height = row.height;
        c.smagorinsky = row.les;
        c.steps = static_cast<std::size_t>(2500.0 * row.diameter);
        const test::CylinderSetup setup = make_cylinder(c);
        auto solver = test::take(make_gpu_solver(*gl.backend, setup.config, setup.mask));
        (void)run_cylinder(*solver, c, setup, row.les ? "GPU LES" : "GPU");
    }
}
