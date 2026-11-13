// Validation: square flat plate normal to the flow (docs/lbm.md, "3D
// validation").
//
// Reference: Cd = 1.28 for a flat plate normal to the flow, frontal area as
// reference area, low-speed wind tunnel (NASA Glenn Research Center,
// Beginner's Guide to Aeronautics, "Shape Effects on Drag"; the page states
// no aspect ratio or Reynolds number). The drag of a thin plate with sharp
// edges depends on its aspect ratio rather than on the Reynolds number once
// the flow separates at the edges: about 1.2 for a square plate or a disk,
// rising towards 2 for an infinitely long one (Hoerner, Fluid-Dynamic Drag,
// 1965, chapter 3). The case here is a square plate (aspect ratio 1),
// thickness 1/16 of the side, side as reference length and side^2 as
// reference area. Acceptance (project specification, section 5.4): within
// 15 percent of 1.28, which covers the square-plate value.
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "../../aero/lbm/lbm_gl_support.h"
#include "body_case.h"

using namespace orrery::aero::lbm;
using namespace orrery::aero::lbm::validation;

namespace {

constexpr double kReferenceCd = 1.28;

} // namespace

TEST_CASE("Square plate normal to the flow has the drag of a flat plate (GPU)",
          "[aero][lbm3d][validation][gl]") {
    test::GlCompute gl = test::gl_compute_or_skip();
    // Re 1000, 24 cells per side, 3 sides ahead, 6 behind, 5 wide; the wake
    // is chaotic, so the drag is averaged over the last 25 convective times.
    BodyCase c;
    c.reynolds = 1000.0;
    c.cells = 24.0;
    c.upstream = 3.0;
    c.downstream = 6.0;
    c.width = 5.0;
    c.duration = 40.0;
    c.average = 25.0;
    const BodyResult r =
        run_body(square_plate_geometry(1.0 / 16.0), c, gl.backend.get(), "plate GPU");
    INFO("Cd " << r.cd << ", reference " << kReferenceCd << ", ratio " << r.cd / kReferenceCd);
    CHECK(std::abs(r.cd / kReferenceCd - 1.0) < 0.15);
    CHECK(gl.context.debug_error_count() == 0);
}
