// The CPU D3Q19 solver on flows with exact solutions: plane Poiseuille,
// square duct, Couette, uniform translation, mass and momentum balances, and
// the configuration checks.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <numbers>
#include <vector>

#include "lbm3d_test_support.h"
#include "orrery/aero/lbm3d/lbm3d.h"

using namespace orrery::aero::lbm;
using namespace orrery::aero::lbm::test;
using Catch::Approx;

namespace {

// Relative L2 error of the x velocity against `exact(x, y, z)` over fluid
// nodes.
template <typename Exact> double profile_error(const MacroFields3D& fields, const Exact& exact) {
    double diff = 0.0;
    double norm = 0.0;
    for (std::size_t z = 0; z < fields.nz; ++z) {
        for (std::size_t y = 0; y < fields.ny; ++y) {
            for (std::size_t x = 0; x < fields.nx; ++x) {
                const std::size_t c = x + fields.nx * (y + fields.ny * z);
                if (fields.solid[c] != 0) {
                    continue;
                }
                const double e = exact(x, y, z);
                diff += (fields.velocity_x[c] - e) * (fields.velocity_x[c] - e);
                norm += e * e;
            }
        }
    }
    return std::sqrt(diff / norm);
}

double run_poiseuille(double tau, std::size_t steps) {
    const Lbm3DConfig config = poiseuille3d_config(4, 24, 4, tau, 1e-6);
    auto solver = take(make_cpu_solver(config, fluid_mask_3d(4, 24, 4)));
    require_ok(solver->step(steps));
    const MacroFields3D fields = take(solver->macroscopic());
    return profile_error(fields, [&](std::size_t, std::size_t y, std::size_t) {
        return poiseuille3d_velocity(config, static_cast<double>(y));
    });
}

} // namespace

TEST_CASE("D3Q19 plane Poiseuille flow matches the parabola", "[aero][lbm3d]") {
    // With halfway bounce-back the BGK wall sits exactly halfway for
    // tau = 1/2 + sqrt(3/16) (the parabola is then exact); other tau values
    // move it by a viscosity-dependent slip (Kruger et al. 2017, 5.3.3).
    const double magic = 0.5 + std::sqrt(3.0 / 16.0);
    const double exact = run_poiseuille(magic, 14000);
    const double slip = run_poiseuille(0.8, 14000);
    std::cout << "measured: 3D Poiseuille relative L2 at tau " << magic << ": " << exact
              << ", at tau 0.8: " << slip << "\n";
    CHECK(exact < 1e-9);
    CHECK(slip < 2e-3);
}

TEST_CASE("D3Q19 square duct flow matches the series solution", "[aero][lbm3d]") {
    const double tau = 0.5 + std::sqrt(3.0 / 16.0);
    const Lbm3DConfig config = duct_config(4, 20, 20, tau, 1e-6);
    auto solver = take(make_cpu_solver(config, fluid_mask_3d(4, 20, 20)));
    require_ok(solver->step(8000));
    const MacroFields3D fields = take(solver->macroscopic());
    const double error = profile_error(fields, [&](std::size_t, std::size_t y, std::size_t z) {
        return duct_velocity(config, y, z);
    });
    std::cout << "measured: square duct 20 x 20 relative L2 " << error << "\n";
    CHECK(error < 1e-3);
}

TEST_CASE("D3Q19 Couette flow between a resting and a moving wall", "[aero][lbm3d]") {
    const Lbm3DConfig config = couette3d_config(4, 4, 16, 0.8, 0.02);
    auto solver = take(make_cpu_solver(config, fluid_mask_3d(4, 4, 16)));
    require_ok(solver->step(8000));
    const MacroFields3D fields = take(solver->macroscopic());
    const double error = profile_error(fields, [&](std::size_t, std::size_t, std::size_t z) {
        return 0.02 * (static_cast<double>(z) + 0.5) / 16.0;
    });
    std::cout << "measured: 3D Couette relative L2 " << error << "\n";
    CHECK(error < 1e-9);
}

TEST_CASE("D3Q19 uniform translation between co-moving walls at any density", "[aero][lbm3d]") {
    // Walls on y and z moving with the fluid: the uniform state is exact; a
    // moving-wall term without the wall density would push it off.
    for (const double rho : {0.7, 1.0, 2.0}) {
        Lbm3DConfig config = periodic_box(4, 8, 8, 0.7);
        const double u = 0.03;
        config.y_low = wall(WallKind::no_slip, u);
        config.y_high = wall(WallKind::no_slip, u);
        config.z_low = wall(WallKind::no_slip, u);
        config.z_high = wall(WallKind::no_slip, u);
        auto solver = take(make_cpu_solver(config, fluid_mask_3d(4, 8, 8)));
        require_ok(solver->initialize(uniform_state3d(rho, u)));
        require_ok(solver->step(300));
        const MacroFields3D fields = take(solver->macroscopic());
        INFO("density " << rho);
        CHECK(uniform_state_error(fields, rho, {u, 0.0, 0.0}, u) < 1e-12);
    }
}

TEST_CASE("The uniform-state check fails on non-finite fields", "[aero][lbm3d]") {
    MacroFields3D fields;
    fields.nx = 2;
    fields.ny = 1;
    fields.nz = 1;
    fields.density = {1.0, 1.0};
    fields.velocity_x = {0.03, 0.03};
    fields.velocity_y = {0.0, 0.0};
    fields.velocity_z = {0.0, 0.0};
    fields.solid = {0, 0};
    CHECK(uniform_state_error(fields, 1.0, {0.03, 0.0, 0.0}, 0.03) == 0.0);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const auto spoilt = [&](std::vector<double> MacroFields3D::* field, double value) {
        MacroFields3D f = fields;
        (f.*field)[1] = value;
        return uniform_state_error(f, 1.0, {0.03, 0.0, 0.0}, 0.03);
    };
    for (std::vector<double> MacroFields3D::* field :
         {&MacroFields3D::density, &MacroFields3D::velocity_x, &MacroFields3D::velocity_y,
          &MacroFields3D::velocity_z}) {
        CHECK_FALSE(spoilt(field, nan) < 1e-12);
        CHECK_FALSE(spoilt(field, std::numeric_limits<double>::infinity()) < 1e-12);
    }
    // An entirely NaN field.
    MacroFields3D all = fields;
    for (std::vector<double>* v :
         {&all.density, &all.velocity_x, &all.velocity_y, &all.velocity_z}) {
        v->assign(v->size(), nan);
    }
    CHECK_FALSE(uniform_state_error(all, 1.0, {0.03, 0.0, 0.0}, 0.03) < 1e-12);
}

TEST_CASE("D3Q19 conserves mass with periodic, slip and moving walls", "[aero][lbm3d]") {
    Lbm3DConfig config = periodic_box(12, 10, 9, 0.62);
    config.y_low = wall(WallKind::slip);
    config.y_high = wall(WallKind::slip);
    config.z_low = wall(WallKind::no_slip, 0.02);
    config.z_high = wall(WallKind::no_slip, -0.01);
    config.smagorinsky = Smagorinsky{true, 0.15};
    SolidMask3D mask = sphere_mask(12, 10, 9, {6.0, 4.6, 4.2}, 2.4);
    auto solver = take(make_cpu_solver(config, mask));
    const double pi = std::numbers::pi;
    require_ok(solver->initialize([pi](std::size_t x, std::size_t y, std::size_t z) {
        const double a = 2.0 * pi * static_cast<double>(x) / 12.0;
        return d3q19::Moments{1.0 + 0.01 * std::sin(a), 0.03 * std::cos(a),
                              0.01 * std::sin(0.7 * static_cast<double>(y)),
                              0.01 * std::cos(0.5 * static_cast<double>(z))};
    }));
    const double before = take(solver->mass());
    require_ok(solver->step(1500));
    const double after = take(solver->mass());
    std::cout << "measured: 3D mass drift over 1500 steps " << std::abs(after - before) / before
              << "\n";
    CHECK(std::abs(after - before) / before < 1e-12);
}

TEST_CASE("D3Q19 momentum exchange balances the body force", "[aero][lbm3d]") {
    // Fully periodic box with a solid layer at y = 0: at steady state the
    // force on the layer equals the body force on all fluid nodes.
    const Lbm3DConfig config = [] {
        Lbm3DConfig c = periodic_box(4, 16, 4, 0.9);
        c.body_force_x = 2e-6;
        return c;
    }();
    const SolidMask3D mask = box_mask(4, 16, 4, {0.0, 0.0, 0.0}, {3.0, 0.0, 3.0});
    auto solver = take(make_cpu_solver(config, mask));
    require_ok(solver->step(6000));
    const Force3D f = take(solver->force());
    const double expected = config.body_force_x * static_cast<double>(4 * 15 * 4);
    std::cout << "measured: momentum exchange " << f.force[0] << " vs body force " << expected
              << "\n";
    CHECK(f.force[0] == Approx(expected).epsilon(1e-6));
    CHECK(std::abs(f.force[1]) < 1e-12);
    CHECK(std::abs(f.force[2]) < 1e-12);
}

TEST_CASE("A uniform stream through the inlet and outlet stays uniform", "[aero][lbm3d]") {
    for (const Outlet outlet : {Outlet::convective, Outlet::extrapolation}) {
        Lbm3DConfig config;
        config.nx = 16;
        config.ny = 6;
        config.nz = 5;
        config.tau = 0.7;
        config.outlet = outlet;
        config.y_low = wall(WallKind::slip);
        config.y_high = wall(WallKind::slip);
        config.z_low = wall(WallKind::periodic);
        config.z_high = wall(WallKind::periodic);
        config.inlet_ux = 0.05;
        config.inlet_uy = 0.0;
        auto solver = take(make_cpu_solver(config, fluid_mask_3d(16, 6, 5)));
        require_ok(solver->step(400));
        const MacroFields3D fields = take(solver->macroscopic());
        CHECK(uniform_state_error(fields, 1.0, {0.05, 0.0, 0.0}, 1.0) < 1e-13);
    }
}

TEST_CASE("The inlet imposes its velocity next to a sphere", "[aero][lbm3d]") {
    SphereSetup setup = sphere_channel(40, 20, 20, 8.0, 2.0, 0.05, 20.0);
    setup.config.inlet_uy = 0.004;
    setup.config.inlet_uz = -0.003;
    auto solver = take(make_cpu_solver(setup.config, setup.mask));
    require_ok(solver->step(600));
    const MacroFields3D fields = take(solver->macroscopic());
    double worst = 0.0;
    for (std::size_t z = 0; z < 20; ++z) {
        for (std::size_t y = 0; y < 20; ++y) {
            const std::size_t c = 40 * (y + 20 * z);
            worst = worse(worst, std::abs(fields.velocity_x[c] - 0.05));
            worst = worse(worst, std::abs(fields.velocity_y[c] - 0.004));
            worst = worse(worst, std::abs(fields.velocity_z[c] + 0.003));
        }
    }
    CHECK(worst < 1e-13);
    const Force3D f = take(solver->force());
    CHECK(f.force[0] > 0.0);
}

TEST_CASE("The CPU D3Q19 solver is deterministic", "[aero][lbm3d]") {
    const SphereSetup setup = sphere_channel(32, 16, 16, 6.0, 2.0, 0.05, 50.0);
    auto a = take(make_cpu_solver(setup.config, setup.mask));
    auto b = take(make_cpu_solver(setup.config, setup.mask));
    require_ok(a->step(150));
    require_ok(b->step(100));
    require_ok(b->step(50));
    CHECK(take(a->populations()) == take(b->populations()));
}

namespace {

// Wall fraction q of the link from the fluid node (3, 4, 4) to the solid
// node (2, 4, 4) (direction 1, c = (1, 0, 0)) of a periodic 8^3 box with the
// interpolated wall, for the distances `fluid` and `solid` at those nodes
// (1 at the other nodes). `second_solid` makes the node behind the link,
// (4, 4, 4), solid as well (distance -1).
double fraction_of(double fluid, double solid, bool second_solid) {
    Lbm3DConfig config = periodic_box(8, 8, 8, 0.7);
    config.solid_wall = SolidWall::interpolated;
    SolidMask3D mask = fluid_mask_3d(8, 8, 8);
    mask.distance.assign(mask.solid.size(), 1.0);
    mask.solid[mask.index(2, 4, 4)] = 1;
    mask.distance[mask.index(2, 4, 4)] = solid;
    mask.distance[mask.index(3, 4, 4)] = fluid;
    if (second_solid) {
        mask.solid[mask.index(4, 4, 4)] = 1;
        mask.distance[mask.index(4, 4, 4)] = -1.0;
    }
    REQUIRE_FALSE(validate(config, mask));
    for (const BoundaryLink3D& link : boundary_links(config, mask)) {
        if (link.cell == mask.index(3, 4, 4) && link.direction == 1) {
            return link.q;
        }
    }
    FAIL("no boundary link from (3, 4, 4) in direction 1");
    return -1.0;
}

} // namespace

TEST_CASE("Wall fractions of the interpolated wall from the signed distance", "[aero][lbm3d]") {
    const double inf = std::numeric_limits<double>::infinity();
    const double big = std::numeric_limits<double>::max();
    // The zero of the linear interpolation between the two nodes.
    CHECK(fraction_of(0.25, -0.75, false) == 0.25);
    CHECK(fraction_of(0.75, -0.25, false) == 0.75);
    CHECK(fraction_of(0.0, -1.0, false) == 0.0); // the wall on the fluid node
    CHECK(fraction_of(1.0, 0.0, false) == 1.0);  // the wall on the solid node
    CHECK(fraction_of(std::numeric_limits<double>::denorm_min(), 0.0, false) == 1.0);
    // q < 1/2 needs a fluid node behind the link; without one, halfway.
    CHECK(fraction_of(0.25, -0.75, true) == 0.5);
    CHECK(fraction_of(0.0, -1.0, true) == 0.5);
    // Distances that place no wall on the link: halfway.
    CHECK(fraction_of(inf, -1.0, false) == 0.5);
    CHECK(fraction_of(1.0, -inf, false) == 0.5);
    CHECK(fraction_of(inf, -inf, false) == 0.5);
    CHECK(fraction_of(2.0, 1.0, false) == 0.5);   // solid node outside the body
    CHECK(fraction_of(-0.5, -1.0, false) == 0.5); // fluid node inside the body
    CHECK(fraction_of(0.0, 0.0, false) == 0.5);
    // Finite distances whose difference overflows: still the linear zero.
    CHECK(fraction_of(big, -big, false) == 0.5);
    CHECK(fraction_of(big, -0.5 * big, false) == Approx(2.0 / 3.0).epsilon(1e-15));
}

TEST_CASE("Links with unusable wall distances reflect like the halfway wall", "[aero][lbm3d]") {
    const double inf = std::numeric_limits<double>::infinity();
    const double big = std::numeric_limits<double>::max();
    struct Distances {
        double fluid;
        double solid;
        const char* what;
    };
    for (const Distances& d : {Distances{inf, -1.0, "infinite fluid distance"},
                               Distances{1.0, -inf, "infinite solid distance"},
                               Distances{2.0, 1.0, "positive solid distance"},
                               Distances{big, -big, "overflowing difference"}}) {
        INFO(d.what);
        Lbm3DConfig halfway = periodic_box(8, 8, 8, 0.7);
        Lbm3DConfig interpolated = halfway;
        interpolated.solid_wall = SolidWall::interpolated;
        SolidMask3D mask = box_mask(8, 8, 8, {3.0, 3.0, 3.0}, {4.0, 4.0, 4.0});
        for (std::size_t c = 0; c < mask.solid.size(); ++c) {
            mask.distance[c] = mask.solid[c] != 0 ? d.solid : d.fluid;
        }
        const std::vector<BoundaryLink3D> links = boundary_links(interpolated, mask);
        REQUIRE_FALSE(links.empty());
        std::size_t off_halfway = 0;
        for (const BoundaryLink3D& link : links) {
            off_halfway += link.q == 0.5 ? 0 : 1;
        }
        CHECK(off_halfway == 0);
        auto a = take(make_cpu_solver(halfway, mask));
        auto b = take(make_cpu_solver(interpolated, mask));
        const auto shear = [](std::size_t, std::size_t y, std::size_t z) {
            return d3q19::Moments{1.0, 0.03 + 0.01 * std::sin(0.8 * static_cast<double>(y)),
                                  0.01 * std::cos(0.6 * static_cast<double>(z)), 0.0};
        };
        require_ok(a->initialize(shear));
        require_ok(b->initialize(shear));
        require_ok(a->step(20));
        require_ok(b->step(20));
        const std::vector<double> fb = take(b->populations());
        CHECK(std::all_of(fb.begin(), fb.end(), [](double v) { return std::isfinite(v); }));
        const bool same = fb == take(a->populations());
        CHECK(same);
        const Force3D fa = take(a->force());
        const Force3D ff = take(b->force());
        CHECK(std::isfinite(ff.force[0]));
        CHECK(ff.force == fa.force);
        CHECK(ff.moment == fa.moment);
    }
}

TEST_CASE("The Smagorinsky closure relaxes a manufactured shear at the predicted rate",
          "[aero][lbm3d]") {
    // Every ordered pair of axes, so each off-diagonal stress (xy, xz, yz)
    // drives the closure on its own.
    const double tau = 0.52;
    const double gamma = 0.01;
    for (std::size_t a = 0; a < 3; ++a) {
        for (std::size_t b = 0; b < 3; ++b) {
            if (a == b) {
                continue;
            }
            for (const bool closure : {true, false}) {
                Lbm3DConfig config = shear_box(tau, 0.4);
                config.smagorinsky.enabled = closure;
                auto solver = take(make_cpu_solver(config, fluid_mask_3d(8, 8, 8)));
                require_ok(solver->initialize(linear_shear(a, b, gamma)));
                require_ok(solver->step(1));
                const std::vector<double> after = take(solver->populations());
                for (const std::size_t k : {std::size_t{1}, std::size_t{3}, std::size_t{6}}) {
                    std::array<std::size_t, 3> node{2, 5, 4};
                    node[b] = k;
                    const ClosureProbe probe = probe_closure(config, a, b, gamma, after, node);
                    INFO("u along axis " << a << ", varying along " << b << ", closure " << closure
                                         << ", node " << node[0] << " " << node[1] << " " << node[2]
                                         << ": tau_eff " << probe.measured << ", closure "
                                         << probe.predicted << ", continuum " << probe.continuum);
                    if (!closure) {
                        CHECK(probe.measured == Approx(tau).epsilon(1e-10));
                        continue;
                    }
                    CHECK(probe.measured == Approx(probe.predicted).epsilon(1e-10));
                    CHECK(std::abs(probe.measured - probe.continuum) <
                          0.02 * (probe.continuum - tau));
                    CHECK(probe.measured - tau > 0.005); // tau_eff - tau about 0.009
                }
            }
        }
    }
}

TEST_CASE("D3Q19 configurations are validated", "[aero][lbm3d]") {
    const SolidMask3D mask = fluid_mask_3d(8, 8, 8);
    Lbm3DConfig ok = periodic_box(8, 8, 8, 0.6);
    CHECK_FALSE(validate(ok, mask));

    Lbm3DConfig small = ok;
    small.nz = 2;
    CHECK(validate(small, fluid_mask_3d(8, 8, 2)));
    Lbm3DConfig mismatch = ok;
    mismatch.nx = 9;
    CHECK(validate(mismatch, mask));
    Lbm3DConfig tau = ok;
    tau.tau = 0.5;
    CHECK(validate(tau, mask));
    Lbm3DConfig infinite = ok;
    infinite.body_force_z = std::numeric_limits<double>::infinity();
    CHECK(validate(infinite, mask));
    Lbm3DConfig half_periodic = ok;
    half_periodic.z_low = wall(WallKind::slip);
    CHECK(validate(half_periodic, mask));
    Lbm3DConfig fast_wall = ok;
    fast_wall.y_low = wall(WallKind::no_slip, 0.06);
    fast_wall.y_high = wall(WallKind::no_slip, 0.0);
    CHECK(validate(fast_wall, mask));
    Lbm3DConfig huge;
    huge.nx = 4;
    huge.ny = 4;
    huge.nz = (std::size_t{1} << 60U) + 1;
    CHECK(validate(huge, mask));

    Lbm3DConfig channel = ok;
    channel.x_boundary = XBoundary::inflow_outflow;
    channel.inlet_ux = 0.05;
    CHECK_FALSE(validate(channel, mask));
    channel.inlet_uz = 0.04; // |u| 0.064: lattice Mach 0.11
    CHECK(validate(channel, mask));
    channel.inlet_uz = 0.0;
    const SolidMask3D blocked = box_mask(8, 8, 8, {7.0, 3.0, 3.0}, {7.0, 3.0, 3.0});
    CHECK(validate(channel, blocked)); // solid node on the outlet face
    channel.inlet_ux = 0.0;
    CHECK(validate(channel, mask)); // convective outlet without inflow
    channel.outlet = Outlet::extrapolation;
    CHECK_FALSE(validate(channel, mask));
    CHECK_FALSE(make_cpu_solver(tau, mask));
}

TEST_CASE("Memory estimates of the 3D solvers", "[aero][lbm3d]") {
    Lbm3DConfig config = periodic_box(128, 128, 128, 0.6);
    const std::size_t cells = std::size_t{128} * 128 * 128;
    CHECK(gpu_population_buffer_bytes(config) == cells * 19 * 4);
    CHECK(gpu_memory_bytes(config) ==
          cells * std::size_t{2 * 19 * 4 + 4 + 16} + std::size_t{128} * 128 * 5 * 4);
    CHECK(cpu_memory_bytes(config) == cells * std::size_t{2 * 19 * 8 + 2});
    config.nz = (std::size_t{1} << 60U) + 1;
    CHECK_FALSE(gpu_memory_bytes(config));
    CHECK_FALSE(cpu_memory_bytes(config));
}
