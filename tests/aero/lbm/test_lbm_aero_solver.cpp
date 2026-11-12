#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <vector>

#include "orrery/aero/aero_solver.h"
#include "orrery/aero/lbm/lbm_aero_solver.h"
#include "orrery/geom/naca.h"

using namespace orrery;
using namespace orrery::aero;
using namespace orrery::aero::lbm;
using Catch::Approx;

namespace {

// Circle of diameter d (m) centred at the origin, counter-clockwise.
AeroGeometry cylinder_geometry(double d) {
    AeroGeometry g;
    const int n = 96;
    for (int k = 0; k < n; ++k) {
        const double a = 2.0 * std::numbers::pi * static_cast<double>(k) / n;
        g.section.push_back({0.5 * d * std::cos(a), 0.5 * d * std::sin(a)});
    }
    g.reference.length = d;
    g.reference.area = d * 1.0; // per metre of span
    return g;
}

// Water-like fluid at Re = rho V d / mu.
FlowConditions flow_at(double reynolds, double d) {
    FlowConditions c;
    c.speed = 0.1;
    c.density = 1000.0;
    c.viscosity = c.density * c.speed * d / reynolds;
    return c;
}

} // namespace

TEST_CASE("The LBM solver runs a cylinder through IAeroSolver", "[aero][lbm][solver]") {
    LbmSolverOptions options;
    options.resolution = LatticeResolution{10.0, 0.05};
    options.height = 12.0;
    options.downstream = 10.0;
    options.duration = 10.0;
    options.average_window = 4.0;
    options.steps_per_call = 1000;
    LbmAeroSolver2D solver(options);
    CHECK(solver.kind() == SolverKind::lattice_boltzmann);
    CHECK_THROWS_AS(solver.step(), std::logic_error);

    const AeroGeometry geometry = cylinder_geometry(0.02);
    const FlowConditions conditions = flow_at(100.0, 0.02);
    solver.setup(geometry, conditions);
    REQUIRE(solver.units());
    CHECK(solver.units()->reynolds() == Approx(100.0));
    CHECK(solver.result().has_warning("lbm.resolution")); // 10 cells per diameter
    CHECK_FALSE(solver.result().has_warning("lbm.les"));
    CHECK_FALSE(solver.result().has_warning("lbm.mach"));

    int calls = 0;
    while (!solver.step()) {
        ++calls;
        REQUIRE(calls < 100);
    }
    const AeroResult& r = solver.result();
    CHECK(r.iterations == 2000); // 10 L / U at 10 cells and u_lat 0.05
    CHECK(r.residuals.size() == 2);
    CHECK(std::isfinite(r.cd));
    CHECK(r.cd > 1.0);
    CHECK(r.cd < 2.5);
    CHECK(std::abs(r.cl) < 0.5 * r.cd);
    // Force and coefficient are the same number in SI.
    const double q = conditions.dynamic_pressure();
    CHECK(r.force.x == Approx(r.cd * q * geometry.reference.area).epsilon(1e-9));
    CHECK(solver.instantaneous().drag > 0.0);
    CHECK(solver.history().size() == 200);

    // Wall pressure: the front stagnation point is near Cp = 1.
    REQUIRE_FALSE(r.surface.cp.empty());
    CHECK(r.surface.points.size() == r.surface.cp.size());
    CHECK(r.surface.normals.size() == r.surface.cp.size());
    const auto front = std::max_element(r.surface.cp.begin(), r.surface.cp.end());
    CHECK(*front == Approx(1.0).margin(0.25));
    const auto at = static_cast<std::size_t>(front - r.surface.cp.begin());
    CHECK(r.surface.points[at].x < 0.0);
    CHECK(r.surface.normals[at].x < -0.5);

    REQUIRE(r.velocity);
    CHECK(r.velocity->dims[1] == 1);
    CHECK(r.velocity->values.size() == r.velocity->dims[0] * r.velocity->dims[2]);
    CHECK(r.velocity->spacing == Approx(0.002));
}

TEST_CASE("A pitched NACA section produces lift and a pitching moment", "[aero][lbm][solver]") {
    AeroGeometry geometry;
    const geom::Naca4 airfoil = *geom::Naca4::from_designation("0012");
    geometry.section = airfoil.section(60);
    geometry.reference.length = 1.0;
    geometry.reference.area = 1.0;
    geometry.reference.moment_centre = geom::Vector3{0.25, 0.0, 0.0};
    FlowConditions conditions = flow_at(200.0, 1.0);
    conditions.angle_of_attack = 8.0 * std::numbers::pi / 180.0;

    LbmSolverOptions options;
    options.resolution = LatticeResolution{16.0, 0.05};
    options.upstream = 4.0;
    options.downstream = 6.0;
    options.height = 8.0;
    options.duration = 12.0;
    options.average_window = 4.0;
    options.steps_per_call = 4000;
    LbmAeroSolver2D solver(options);
    solver.setup(geometry, conditions);
    while (!solver.step()) {
    }
    const AeroResult& r = solver.result();
    INFO("cl " << r.cl << " cd " << r.cd << " cm " << r.cm);
    CHECK(r.cl > 0.2);
    CHECK(r.cd > 0.0);
    CHECK(std::isfinite(r.cm));
    CHECK_FALSE(r.velocity); // the lattice is aligned with the wind, not the body
    // The lift direction is tilted back by the angle of attack in body axes.
    CHECK(r.force.z > 0.0);
}

TEST_CASE("The LBM solver warns outside its envelope", "[aero][lbm][solver]") {
    const AeroGeometry geometry = cylinder_geometry(1.0);
    LbmSolverOptions options;
    options.resolution = LatticeResolution{12.0, 0.05};
    options.height = 8.0;
    options.downstream = 4.0;
    options.upstream = 2.0;

    // Re 5e4: automatic Smagorinsky closure, BGK relaxation time near 1/2.
    LbmAeroSolver2D les(options);
    les.setup(geometry, flow_at(5e4, 1.0));
    CHECK(les.result().has_warning("lbm.les"));
    CHECK(les.result().has_warning("lbm.tau"));
    REQUIRE(les.lattice() != nullptr);
    CHECK(les.lattice()->config().smagorinsky.enabled);

    options.les = LesMode::off;
    LbmAeroSolver2D bare(options);
    bare.setup(geometry, flow_at(5e4, 1.0));
    CHECK(bare.result().has_warning("lbm.les_off"));
    CHECK_FALSE(bare.lattice()->config().smagorinsky.enabled);

    // Air at Mach 0.45 and a sideslip angle: the method's envelope warnings.
    FlowConditions fast;
    fast.speed = 150.0;
    fast.sideslip = 0.1;
    LbmAeroSolver2D compressible(options);
    compressible.setup(geometry, fast);
    CHECK(compressible.result().has_warning("lbm.mach"));
    CHECK(compressible.result().has_warning("lbm.sideslip"));
}

TEST_CASE("The LBM solver rejects unusable problems", "[aero][lbm][solver]") {
    LbmAeroSolver2D solver;
    AeroGeometry empty;
    CHECK_THROWS_AS(solver.setup(empty, flow_at(100.0, 1.0)), std::invalid_argument);
    const AeroGeometry geometry = cylinder_geometry(1.0);
    FlowConditions still = flow_at(100.0, 1.0);
    still.speed = 0.0;
    CHECK_THROWS_AS(solver.setup(geometry, still), std::invalid_argument);
    AeroGeometry no_reference = geometry;
    no_reference.reference.length = 0.0;
    CHECK_THROWS_AS(solver.setup(no_reference, flow_at(100.0, 1.0)), std::invalid_argument);

    // The Mach guard of the lattice units.
    LbmSolverOptions fast;
    fast.resolution = LatticeResolution{10.0, 0.08};
    LbmAeroSolver2D refused(fast);
    CHECK_THROWS_AS(refused.setup(geometry, flow_at(100.0, 1.0)), std::invalid_argument);
}
