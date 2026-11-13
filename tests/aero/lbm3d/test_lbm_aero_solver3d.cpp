// LbmAeroSolver3D: the D3Q19 solver behind IAeroSolver on the CPU, small
// lattices. Every validity warning of the adapter is triggered here.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include "orrery/aero/aero_solver.h"
#include "orrery/aero/compute/null_compute_backend.h"
#include "orrery/aero/lbm3d/lbm_aero_solver3d.h"
#include "orrery/geom/primitives.h"

using namespace orrery;
using namespace orrery::aero;
using namespace orrery::aero::lbm;
using Catch::Approx;

namespace {

// Sphere of diameter d (m) centred at the origin.
AeroGeometry sphere_geometry(double d) {
    AeroGeometry g;
    g.mesh = geom::sphere_profile(0.5 * d).mesh({24, 0.15});
    g.reference.length = d;
    g.reference.area = 0.25 * std::numbers::pi * d * d;
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

// A coarse channel: 8 cells per diameter, 2 diameters ahead, 3 behind, 3
// wide (a 49 x 25 x 25 lattice).
LbmSolver3DOptions coarse_options() {
    LbmSolver3DOptions options;
    options.resolution = LatticeResolution{8.0, 0.05};
    options.upstream = 2.0;
    options.downstream = 3.0;
    options.width = 3.0;
    options.duration = 6.0;
    options.average_window = 2.0;
    options.steps_per_call = 1000;
    return options;
}

void run(LbmAeroSolver3D& solver) {
    std::size_t calls = 0;
    while (!solver.step()) {
        ++calls;
        REQUIRE(calls < 100000);
    }
}

} // namespace

TEST_CASE("The 3D LBM solver runs a sphere through IAeroSolver", "[aero][lbm3d][solver]") {
    LbmAeroSolver3D solver(coarse_options());
    CHECK(solver.kind() == SolverKind::lattice_boltzmann);
    CHECK_THROWS_AS(solver.step(), std::logic_error);

    const AeroGeometry geometry = sphere_geometry(0.02);
    const FlowConditions conditions = flow_at(20.0, 0.02);
    solver.setup(geometry, conditions);
    REQUIRE(solver.units());
    CHECK(solver.units()->reynolds() == Approx(20.0));
    REQUIRE(solver.lattice() != nullptr);
    const std::size_t nx = solver.lattice()->config().nx;
    CHECK((nx == 49 || nx == 50)); // 6 diameters of 8 cells plus the end node
    CHECK(solver.lattice()->config().ny == 25);
    CHECK(solver.result().has_warning("lbm.resolution")); // 8 cells per diameter
    CHECK(solver.result().has_warning("lbm.blockage"));   // 3 diameters wide: 8.7 percent
    CHECK_FALSE(solver.result().has_warning("lbm.les"));
    CHECK_FALSE(solver.result().has_warning("lbm.mach"));

    run(solver);
    const AeroResult& r = solver.result();
    CHECK(r.iterations == 960); // 6 L / U at 8 cells and u_lat 0.05
    INFO("cd " << r.cd << " cl " << r.cl << " cy " << r.cy << " cm " << r.cm);
    CHECK(std::isfinite(r.cd));
    CHECK(r.cd > 1.5);
    CHECK(r.cd < 6.0);
    CHECK(std::abs(r.cl) < 0.05 * r.cd);
    CHECK(std::abs(r.cy) < 0.05 * r.cd);
    CHECK(std::abs(r.cm) < 0.05 * r.cd);
    // Force and coefficient are the same number in SI.
    const double q = conditions.dynamic_pressure();
    CHECK(r.force.x == Approx(r.cd * q * geometry.reference.area).epsilon(1e-9));
    CHECK(solver.instantaneous().drag > 0.0);
    CHECK(solver.history().size() == 96);
    CHECK(solver.side_history().size() == 96);

    // Wall pressure: at Re 20 viscous stresses lift the front stagnation
    // pressure above the inviscid Cp = 1.
    REQUIRE_FALSE(r.surface.cp.empty());
    CHECK(r.surface.points.size() == r.surface.cp.size());
    CHECK(r.surface.normals.size() == r.surface.cp.size());
    double area = 0.0;
    for (const double a : r.surface.areas) {
        area += a;
    }
    CHECK(area == Approx(geom::surface_area(geometry.mesh)).epsilon(1e-12));
    const auto front = std::max_element(r.surface.cp.begin(), r.surface.cp.end());
    CHECK(*front > 1.0);
    CHECK(*front < 2.0);
    const auto at = static_cast<std::size_t>(front - r.surface.cp.begin());
    CHECK(r.surface.points[at].x < 0.0);
    CHECK(r.surface.normals[at].x < -0.5);

    REQUIRE(r.velocity);
    CHECK(r.velocity->dims[0] == nx);
    CHECK(r.velocity->dims[2] == 25);
    CHECK(r.velocity->values.size() == nx * 25 * 25);
    CHECK(r.velocity->spacing == Approx(0.0025));
}

TEST_CASE("Angle of attack and sideslip rotate the wind frame", "[aero][lbm3d][solver]") {
    LbmSolver3DOptions options = coarse_options();
    options.duration = 3.0;
    LbmAeroSolver3D solver(options);
    FlowConditions conditions = flow_at(20.0, 0.02);
    conditions.angle_of_attack = 30.0 * std::numbers::pi / 180.0;
    conditions.sideslip = 20.0 * std::numbers::pi / 180.0;
    solver.setup(sphere_geometry(0.02), conditions);
    run(solver);
    const AeroResult& r = solver.result();
    // A sphere feels drag only: the force lies along the free stream.
    const geom::Vector3 d = conditions.drag_direction();
    const double along = r.force.x * d.x + r.force.y * d.y + r.force.z * d.z;
    const double total =
        std::sqrt(r.force.x * r.force.x + r.force.y * r.force.y + r.force.z * r.force.z);
    CHECK(along == Approx(total).epsilon(2e-3));
    CHECK(r.cd > 1.0);
    CHECK_FALSE(r.velocity); // the lattice is aligned with the wind, not the body
}

TEST_CASE("The 3D LBM solver warns outside its envelope", "[aero][lbm3d][solver]") {
    const AeroGeometry geometry = sphere_geometry(1.0);
    LbmSolver3DOptions options = coarse_options();
    options.width = 6.0;

    // Re 5000: automatic Smagorinsky closure, BGK relaxation time near 1/2.
    LbmAeroSolver3D les(options);
    les.setup(geometry, flow_at(5000.0, 1.0));
    CHECK(les.result().has_warning("lbm.les"));
    CHECK(les.result().has_warning("lbm.tau"));
    CHECK_FALSE(les.result().has_warning("lbm.blockage"));
    REQUIRE(les.lattice() != nullptr);
    CHECK(les.lattice()->config().smagorinsky.enabled);

    options.les = LesMode::off;
    LbmAeroSolver3D bare(options);
    bare.setup(geometry, flow_at(5000.0, 1.0));
    CHECK(bare.result().has_warning("lbm.les_off"));
    CHECK_FALSE(bare.lattice()->config().smagorinsky.enabled);

    // Air at Mach 0.45 (and Re 1e7): the method's envelope warnings.
    FlowConditions fast;
    fast.speed = 150.0;
    LbmAeroSolver3D compressible(options);
    compressible.setup(geometry, fast);
    CHECK(compressible.result().has_warning("lbm.mach"));
    CHECK(compressible.result().has_warning("lbm.reynolds"));

    // A plate 0.1 thick at 16 cells per length: 1.6 cells across.
    AeroGeometry plate;
    plate.mesh = geom::box_mesh(0.1, 1.0, 1.0);
    plate.reference.length = 1.0;
    plate.reference.area = 1.0;
    LbmSolver3DOptions fine = options;
    fine.resolution = LatticeResolution{16.0, 0.05};
    fine.width = 3.0;
    LbmAeroSolver3D thin(fine);
    thin.setup(plate, flow_at(20.0, 1.0));
    CHECK(thin.result().has_warning("lbm.resolution"));
    LbmAeroSolver3D sphere(fine);
    sphere.setup(geometry, flow_at(20.0, 1.0));
    CHECK_FALSE(sphere.result().has_warning("lbm.resolution"));
}

TEST_CASE("Fast local flow and a diverged lattice are reported", "[aero][lbm3d][solver]") {
    // A plate across most of a narrow channel: the flow squeezes through the
    // gaps at several times the inlet speed.
    AeroGeometry plate;
    plate.mesh = geom::box_mesh(0.25, 1.0, 1.0);
    plate.reference.length = 1.0;
    plate.reference.area = 1.0;
    LbmSolver3DOptions options = coarse_options();
    options.width = 1.3;
    options.duration = 3.0;
    LbmAeroSolver3D squeezed(options);
    squeezed.setup(plate, flow_at(20.0, 1.0));
    CHECK(squeezed.result().has_warning("lbm.blockage"));
    run(squeezed);
    CHECK(squeezed.result().has_warning("lbm.local_mach"));

    // Re 2e5 without the closure at 8 cells per diameter, started
    // impulsively: tau - 1/2 is 6e-6 and BGK blows up.
    LbmSolver3DOptions bare = coarse_options();
    bare.les = LesMode::off;
    bare.start_ramp = 0.0;
    bare.wall = SolidWall::halfway;
    LbmAeroSolver3D unstable(bare);
    unstable.setup(sphere_geometry(1.0), flow_at(2e5, 1.0));
    CHECK(unstable.result().has_warning("lbm.reynolds"));
    run(unstable);
    INFO("cd " << unstable.result().cd);
    CHECK(unstable.result().has_warning("lbm.diverged"));
    CHECK(unstable.result().has_warning("lbm.unconverged"));
    CHECK_FALSE(unstable.result().converged);
}

TEST_CASE("The 3D LBM solver rejects unusable problems", "[aero][lbm3d][solver]") {
    LbmAeroSolver3D solver(coarse_options());
    AeroGeometry empty;
    empty.reference.length = 1.0;
    CHECK_THROWS_AS(solver.setup(empty, flow_at(100.0, 1.0)), std::invalid_argument);
    const AeroGeometry geometry = sphere_geometry(1.0);
    FlowConditions still = flow_at(100.0, 1.0);
    still.speed = 0.0;
    CHECK_THROWS_AS(solver.setup(geometry, still), std::invalid_argument);
    AeroGeometry no_reference = geometry;
    no_reference.reference.area = 0.0;
    CHECK_THROWS_AS(solver.setup(no_reference, flow_at(100.0, 1.0)), std::invalid_argument);

    LbmSolver3DOptions walls = coarse_options();
    walls.sides = WallKind::no_slip;
    LbmAeroSolver3D walled(walls);
    CHECK_THROWS_AS(walled.setup(geometry, flow_at(100.0, 1.0)), std::invalid_argument);

    // The Mach guard of the lattice units.
    LbmSolver3DOptions fast = coarse_options();
    fast.resolution = LatticeResolution{8.0, 0.06};
    LbmAeroSolver3D refused(fast);
    CHECK_THROWS_AS(refused.setup(geometry, flow_at(100.0, 1.0)), std::invalid_argument);
}

TEST_CASE("The 3D LBM solver refuses a domain with fewer than three nodes along an axis",
          "[aero][lbm3d][solver]") {
    // A body 0.01 long at one cell per reference length, with 0.01 lengths
    // ahead and behind: a 2 x 6 x 6 lattice, refused before the body is
    // placed (placing it would clamp to an empty range of nodes).
    AeroGeometry sliver;
    sliver.mesh = geom::box_mesh(0.01, 0.5, 0.5);
    sliver.reference.length = 1.0;
    sliver.reference.area = 1.0;
    LbmSolver3DOptions options = coarse_options();
    options.resolution = LatticeResolution{1.0, 0.05};
    options.upstream = 0.01;
    options.downstream = 0.01;
    LbmAeroSolver3D solver(options);
    try {
        solver.setup(sliver, flow_at(20.0, 1.0));
        FAIL("a lattice with two nodes along x was accepted");
    } catch (const std::invalid_argument& error) {
        const std::string message = error.what();
        INFO(message);
        CHECK(message.find("2 x 6 x 6") != std::string::npos);
        CHECK(message.find("at least 3 nodes") != std::string::npos);
    }
    CHECK(solver.lattice() == nullptr);
}

TEST_CASE("The 3D LBM solver refuses lattices above its cell budget", "[aero][lbm3d][solver]") {
    const AeroGeometry geometry = sphere_geometry(1.0);
    LbmSolver3DOptions options = coarse_options();
    options.max_cells = 20000; // the coarse lattice has about 31000 nodes
    LbmAeroSolver3D capped(options);
    try {
        capped.setup(geometry, flow_at(20.0, 1.0));
        FAIL("a lattice above max_cells was accepted");
    } catch (const std::invalid_argument& error) {
        const std::string message = error.what();
        INFO(message);
        CHECK(message.find(" x 25 x 25") != std::string::npos);
        CHECK(message.find("max_cells") != std::string::npos);
        CHECK(message.find("cells per reference length") != std::string::npos);
    }
    CHECK(capped.lattice() == nullptr);

    // The default cap: 256^3 nodes when the device memory is unknown.
    LbmSolver3DOptions large = coarse_options();
    large.resolution = LatticeResolution{80.0, 0.05}; // 481 x 241 x 241
    LbmAeroSolver3D refused(large);
    CHECK_THROWS_AS(refused.setup(geometry, flow_at(20.0, 1.0)), std::invalid_argument);
    CHECK(refused.lattice() == nullptr);

    // On a backend that reports its free memory, the budget follows it.
    compute::NullComputeBackend backend;
    backend.set_memory({std::size_t{4} << 20U, std::nullopt}); // 4 MiB: about 24000 nodes
    LbmSolver3DOptions device = coarse_options();
    device.backend = &backend;
    LbmAeroSolver3D small(device);
    try {
        small.setup(geometry, flow_at(20.0, 1.0));
        FAIL("a lattice above the device memory was accepted");
    } catch (const std::invalid_argument& error) {
        const std::string message = error.what();
        INFO(message);
        CHECK(message.find("free device memory") != std::string::npos);
    }
    CHECK(backend.live_buffers() == 0);
    backend.set_memory({std::size_t{64} << 20U, std::nullopt});
    small.setup(geometry, flow_at(20.0, 1.0));
    CHECK(small.lattice() != nullptr);
}

TEST_CASE("The 3D LBM solver refuses step counts no integer can hold", "[aero][lbm3d][solver]") {
    const AeroGeometry geometry = sphere_geometry(1.0);
    LbmSolver3DOptions long_run = coarse_options();
    long_run.duration = 1e100;
    LbmAeroSolver3D endless(long_run);
    CHECK_THROWS_AS(endless.setup(geometry, flow_at(20.0, 1.0)), std::invalid_argument);
    CHECK(endless.lattice() == nullptr);
    LbmSolver3DOptions wide = coarse_options();
    wide.average_window = 1e100;
    LbmAeroSolver3D window(wide);
    CHECK_THROWS_AS(window.setup(geometry, flow_at(20.0, 1.0)), std::invalid_argument);
    CHECK(window.lattice() == nullptr);
}

namespace {

struct ShortRun {
    AeroResult result;
    std::vector<double> time;
    std::vector<double> drag;
    std::vector<double> side;
    Coefficients average;
};

// A short Re 20 sphere run (400 lattice steps) with the given call size and
// reference area.
ShortRun short_sphere_run(std::size_t steps_per_call, double area_scale) {
    LbmSolver3DOptions options = coarse_options();
    options.duration = 2.5;
    options.average_window = 1.0;
    options.steps_per_call = steps_per_call;
    LbmAeroSolver3D solver(options);
    AeroGeometry geometry = sphere_geometry(0.02);
    geometry.reference.area *= area_scale;
    solver.setup(geometry, flow_at(20.0, 0.02));
    run(solver);
    ShortRun out;
    out.result = solver.result();
    out.time.assign(solver.history().time().begin(), solver.history().time().end());
    out.drag.assign(solver.history().drag().begin(), solver.history().drag().end());
    out.side = solver.side_history();
    out.average = solver.history().moving_average();
    return out;
}

} // namespace

TEST_CASE("3D averages do not depend on how the steps are split into calls",
          "[aero][lbm3d][solver]") {
    const ShortRun reference = short_sphere_run(400, 1.0);
    REQUIRE(reference.result.iterations == 400);
    REQUIRE(reference.time.size() == 40);
    for (const std::size_t call : {std::size_t{1}, std::size_t{7}, std::size_t{25}}) {
        const ShortRun run = short_sphere_run(call, 1.0);
        INFO("steps_per_call " << call);
        CHECK(run.result.iterations == 400);
        CHECK(run.time == reference.time);
        CHECK(run.drag == reference.drag);
        CHECK(run.side == reference.side);
        CHECK(run.result.cd == reference.result.cd);
        CHECK(run.average.drag == reference.average.drag);
    }
}

TEST_CASE("3D history and result coefficients share the reference area", "[aero][lbm3d][solver]") {
    const ShortRun unit = short_sphere_run(400, 1.0);
    const ShortRun doubled = short_sphere_run(400, 2.0);
    CHECK(doubled.result.cd == Approx(0.5 * unit.result.cd).epsilon(1e-12));
    CHECK(doubled.average.drag == Approx(doubled.result.cd).epsilon(1e-12));
    CHECK(unit.average.drag == Approx(unit.result.cd).epsilon(1e-12));
    REQUIRE(doubled.drag.size() == unit.drag.size());
    double worst = 0.0;
    for (std::size_t k = 0; k < unit.drag.size(); ++k) {
        worst = std::max(worst, std::abs(doubled.drag[k] / (0.5 * unit.drag[k]) - 1.0));
    }
    CHECK(worst < 1e-12);
}

namespace {

// Synthetic force histories for forces_settled_3d: 1000 samples every 10
// steps, analysed over the second half.
enum class Signal : std::uint8_t { steady, periodic, drifting };

std::vector<double> signal_of(Signal kind, const std::vector<double>& time, double mean,
                              double period) {
    std::vector<double> values;
    values.reserve(time.size());
    for (const double t : time) {
        switch (kind) {
            case Signal::steady:
                values.push_back(mean);
                break;
            case Signal::periodic:
                values.push_back(mean + 0.05 * std::sin(2.0 * std::numbers::pi * t / period));
                break;
            case Signal::drifting:
                values.push_back(mean + 2e-6 * t); // 0.01 over the analysed half
                break;
        }
    }
    return values;
}

bool settled(Signal drag, Signal lift, Signal side) {
    std::vector<double> time;
    time.reserve(1000);
    for (int k = 0; k < 1000; ++k) {
        time.push_back(10.0 * k);
    }
    // The drag oscillates at twice the shedding frequency, as behind a body
    // shedding in one plane.
    return forces_settled_3d(time, signal_of(drag, time, 1.0, 200.0),
                             signal_of(lift, time, 0.0, 400.0), signal_of(side, time, 0.0, 400.0),
                             5000.0, SettledCriteria{1e-3, 0.05, 0.02});
}

} // namespace

TEST_CASE("3D convergence needs every force component stationary", "[aero][lbm3d][solver]") {
    using enum Signal;
    // Settled: each component steady or periodic.
    CHECK(settled(steady, steady, steady));
    CHECK(settled(steady, periodic, steady));
    CHECK(settled(steady, steady, periodic));
    CHECK(settled(periodic, periodic, periodic));
    CHECK(settled(periodic, steady, steady));
    // One periodic component does not excuse a drifting one.
    CHECK_FALSE(settled(steady, periodic, drifting));
    CHECK_FALSE(settled(steady, drifting, periodic));
    CHECK_FALSE(settled(periodic, drifting, steady));
    CHECK_FALSE(settled(periodic, steady, drifting));
    CHECK_FALSE(settled(drifting, steady, steady));
    CHECK_FALSE(settled(drifting, periodic, periodic));
}

TEST_CASE("3D convergence: a start-up transient is not converged, a steady wake is",
          "[aero][lbm3d][solver]") {
    // 2.5 convective times from the free stream: the drag is still falling.
    const ShortRun transient = short_sphere_run(400, 1.0);
    CHECK_FALSE(transient.result.converged);
    CHECK(transient.result.has_warning("lbm.unconverged"));

    // Re 10 for 40 convective times: a steady wake.
    LbmSolver3DOptions options = coarse_options();
    options.duration = 40.0;
    options.average_window = 5.0;
    options.steps_per_call = 2000;
    LbmAeroSolver3D solver(options);
    solver.setup(sphere_geometry(0.02), flow_at(10.0, 0.02));
    run(solver);
    INFO("cd " << solver.result().cd);
    CHECK(solver.result().converged);
    CHECK_FALSE(solver.result().has_warning("lbm.unconverged"));
}
