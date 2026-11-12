#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <numbers>
#include <vector>

#include "lbm_test_support.h"
#include "orrery/aero/lbm/lbm2d.h"

using namespace orrery::aero::lbm;
using namespace orrery::aero::lbm::test;
using Catch::Approx;

namespace {

// Steps until the largest change of u_x over `chunk` steps is below
// `tolerance` times its largest value, or `limit` steps.
MacroFields2D run_to_steady(Lbm2DSolver& solver, std::size_t chunk, std::size_t limit,
                            double tolerance) {
    MacroFields2D previous = take(solver.macroscopic());
    while (solver.steps() < limit) {
        require_ok(solver.step(chunk));
        MacroFields2D now = take(solver.macroscopic());
        double change = 0.0;
        double peak = 0.0;
        for (std::size_t c = 0; c < now.velocity_x.size(); ++c) {
            change = std::max(change, std::abs(now.velocity_x[c] - previous.velocity_x[c]));
            peak = std::max(peak, std::abs(now.velocity_x[c]));
        }
        previous = std::move(now);
        if (change <= tolerance * peak) {
            break;
        }
    }
    return previous;
}

// Relative L2 error of the column-averaged u_x(y) against `exact`.
template <typename Exact> double profile_error(const MacroFields2D& fields, const Exact& exact) {
    double diff = 0.0;
    double norm = 0.0;
    for (std::size_t y = 0; y < fields.ny; ++y) {
        const double e = exact(static_cast<double>(y));
        for (std::size_t x = 0; x < fields.nx; ++x) {
            const double u = fields.velocity_x[x + fields.nx * y];
            diff += (u - e) * (u - e);
            norm += e * e;
        }
    }
    return std::sqrt(diff / norm);
}

Lbm2DConfig periodic_box(std::size_t nx, std::size_t ny, double tau) {
    Lbm2DConfig config;
    config.nx = nx;
    config.ny = ny;
    config.tau = tau;
    config.x_boundary = XBoundary::periodic;
    config.bottom = Wall{WallKind::periodic, 0.0};
    config.top = Wall{WallKind::periodic, 0.0};
    return config;
}

// Taylor-Green vortex of amplitude u0 and wave number k = 2 pi / n with its
// pressure field p = -u0^2 / 4 (cos 2kx + cos 2ky).
InitialState taylor_green(std::size_t n, double u0) {
    const double k = 2.0 * std::numbers::pi / static_cast<double>(n);
    return [k, u0](std::size_t xi, std::size_t yi) {
        const double x = static_cast<double>(xi);
        const double y = static_cast<double>(yi);
        const double rho = 1.0 - 0.75 * u0 * u0 * (std::cos(2.0 * k * x) + std::cos(2.0 * k * y));
        return orrery::aero::lbm::d2q9::Moments{rho, -u0 * std::cos(k * x) * std::sin(k * y),
                                                u0 * std::sin(k * x) * std::cos(k * y)};
    };
}

double kinetic_energy(const MacroFields2D& f) {
    double e = 0.0;
    for (std::size_t c = 0; c < f.velocity_x.size(); ++c) {
        e += 0.5 * f.density[c] *
             (f.velocity_x[c] * f.velocity_x[c] + f.velocity_y[c] * f.velocity_y[c]);
    }
    return e;
}

} // namespace

TEST_CASE("Plane Poiseuille flow matches the parabolic profile", "[aero][lbm][flow]") {
    // tau = 1/2 + sqrt(3/16): BGK with halfway bounce-back places the wall
    // exactly halfway, so the parabola is reproduced to round-off; other tau
    // values give a small tau-dependent wall slip (He et al. 1997).
    struct Case {
        double tau;
        double tolerance;
    };
    for (const Case c :
         {Case{0.5 + std::sqrt(3.0 / 16.0), 1e-9}, Case{0.8, 2e-3}, Case{0.6, 2e-3}}) {
        const Lbm2DConfig config = poiseuille_config(4, 32, c.tau, 1e-6);
        auto solver = take(make_cpu_solver(config, fluid_mask(4, 32)));
        const MacroFields2D fields = run_to_steady(*solver, 2000, 400000, 1e-12);
        const double error =
            profile_error(fields, [&](double y) { return poiseuille_velocity(config, y); });
        std::cout << "measured: Poiseuille tau " << c.tau << " L2 relative " << error << " after "
                  << solver->steps() << " steps\n";
        INFO("tau " << c.tau << " error " << error);
        CHECK(error < c.tolerance);
        double max_uy = 0.0;
        for (const double v : fields.velocity_y) {
            max_uy = std::max(max_uy, std::abs(v));
        }
        // No transverse flow beyond round-off.
        CHECK(max_uy < 1e-9 * poiseuille_velocity(config, 15.5));
    }
}

TEST_CASE("Plane Couette flow matches the linear profile", "[aero][lbm][flow]") {
    const Lbm2DConfig config = couette_config(4, 16, 0.9, 0.02);
    auto solver = take(make_cpu_solver(config, fluid_mask(4, 16)));
    const MacroFields2D fields = run_to_steady(*solver, 1000, 200000, 1e-13);
    const double error = profile_error(fields, [](double y) { return 0.02 * (y + 0.5) / 16.0; });
    std::cout << "measured: Couette L2 relative " << error << "\n";
    CHECK(error < 1e-9);

    // The Smagorinsky closure only raises the viscosity of a uniform shear,
    // so the steady profile stays linear.
    Lbm2DConfig les = config;
    les.smagorinsky = Smagorinsky{true, 0.17};
    auto les_solver = take(make_cpu_solver(les, fluid_mask(4, 16)));
    const MacroFields2D les_fields = run_to_steady(*les_solver, 1000, 200000, 1e-13);
    CHECK(profile_error(les_fields, [](double y) { return 0.02 * (y + 0.5) / 16.0; }) < 1e-7);
}

TEST_CASE("Momentum exchange balances the body force on a channel", "[aero][lbm][flow]") {
    // Periodic box with one solid row: at steady state the force on the
    // solid equals the body force on all fluid nodes.
    Lbm2DConfig config = periodic_box(6, 24, 0.9);
    config.body_force_x = 2e-6;
    SolidMask2D mask = fluid_mask(6, 24);
    for (std::size_t x = 0; x < 6; ++x) {
        mask.solid[x] = 1;
    }
    auto solver = take(make_cpu_solver(config, mask));
    (void)run_to_steady(*solver, 2000, 300000, 1e-12);
    const Force2D f = take(solver->force());
    const double expected = 2e-6 * static_cast<double>(6 * 23);
    std::cout << "measured: momentum exchange " << f.x << " expected " << expected << "\n";
    CHECK(f.x == Approx(expected).epsilon(1e-6));
    CHECK(std::abs(f.y) < 1e-9 * f.x);
}

TEST_CASE("Mass is conserved by streaming, collision and every closed boundary",
          "[aero][lbm][flow]") {
    struct Case {
        const char* name;
        WallKind walls;
        double wall_speed;
        bool smagorinsky;
    };
    for (const Case c :
         {Case{"periodic", WallKind::periodic, 0.0, false}, Case{"slip", WallKind::slip, 0.0, true},
          Case{"moving no-slip", WallKind::no_slip, 0.03, false}}) {
        Lbm2DConfig config = periodic_box(48, 40, 0.56);
        config.bottom = Wall{c.walls, 0.0};
        config.top = Wall{c.walls, c.wall_speed};
        config.smagorinsky = Smagorinsky{c.smagorinsky, 0.15};
        auto solver = take(make_cpu_solver(config, circle_mask(48, 40, 20.0, 19.0, 6.0)));
        require_ok(solver->initialize(taylor_green(40, 0.04)));
        const double m0 = take(solver->mass());
        require_ok(solver->step(1500));
        const double m1 = take(solver->mass());
        INFO(c.name << ": m0 " << m0 << " m1 " << m1);
        CHECK(std::abs(m1 - m0) / m0 < 1e-12);
    }
}

TEST_CASE("Taylor-Green vortex decays at the lattice viscosity", "[aero][lbm][flow]") {
    const std::size_t n = 64;
    const double u0 = 0.02;
    const double tau = 0.65;
    auto solver = take(make_cpu_solver(periodic_box(n, n, tau), fluid_mask(n, n)));
    require_ok(solver->initialize(taylor_green(n, u0)));
    const MacroFields2D start = take(solver->macroscopic());

    // Vorticity output against 2 u0 k cos(kx) cos(ky).
    const double k = 2.0 * std::numbers::pi / static_cast<double>(n);
    const std::vector<double> omega = vorticity(start, solver->config());
    std::vector<double> exact(omega.size());
    for (std::size_t y = 0; y < n; ++y) {
        for (std::size_t x = 0; x < n; ++x) {
            exact[x + n * y] = 2.0 * u0 * k * std::cos(k * static_cast<double>(x)) *
                               std::cos(k * static_cast<double>(y));
        }
    }
    CHECK(l2_relative(omega, exact) < 2e-3);
    const std::vector<double> p = pressure(start);
    CHECK(p[0] == Approx(-0.25 * u0 * u0 * 2.0).epsilon(1e-12));

    const std::size_t steps = 2000;
    require_ok(solver->step(steps));
    const MacroFields2D end = take(solver->macroscopic());
    // E(t) = E0 exp(-4 nu k^2 t).
    const double rate =
        -std::log(kinetic_energy(end) / kinetic_energy(start)) / static_cast<double>(steps);
    const double expected = 4.0 * viscosity_of_tau(tau) * k * k;
    std::cout << "measured: Taylor-Green decay rate " << rate << " expected " << expected << "\n";
    CHECK(rate == Approx(expected).epsilon(0.01));

    // The Smagorinsky closure adds eddy viscosity: faster decay.
    Lbm2DConfig les = periodic_box(n, n, tau);
    les.smagorinsky = Smagorinsky{true, 0.3};
    auto les_solver = take(make_cpu_solver(les, fluid_mask(n, n)));
    require_ok(les_solver->initialize(taylor_green(n, u0)));
    require_ok(les_solver->step(steps));
    CHECK(kinetic_energy(take(les_solver->macroscopic())) < kinetic_energy(end));
}

TEST_CASE("Fluid at rest stays at rest and exerts no force", "[aero][lbm][flow]") {
    Lbm2DConfig config = periodic_box(32, 24, 0.7);
    config.bottom = Wall{WallKind::no_slip, 0.0};
    config.top = Wall{WallKind::slip, 0.0};
    auto solver = take(make_cpu_solver(config, circle_mask(32, 24, 15.0, 12.0, 5.0)));
    require_ok(solver->step(200));
    const Force2D f = take(solver->force());
    CHECK(std::abs(f.x) < 1e-15);
    CHECK(std::abs(f.y) < 1e-15);
    CHECK(max_speed(take(solver->macroscopic())) < 1e-16);
}

TEST_CASE("The CPU solver is deterministic", "[aero][lbm][flow]") {
    const CylinderSetup setup = cylinder_setup(120, 60, 10.0, 30.0, 30.5, 0.05, 100.0);
    auto a = take(make_cpu_solver(setup.config, setup.mask));
    auto b = take(make_cpu_solver(setup.config, setup.mask));
    require_ok(a->step(300));
    require_ok(b->step(300));
    CHECK(take(a->populations()) == take(b->populations()));
    const Force2D fa = take(a->force());
    const Force2D fb = take(b->force());
    CHECK(fa.x == fb.x);
    CHECK(fa.y == fb.y);
    CHECK(fa.x > 0.0); // drag along the stream
}

TEST_CASE("Invalid configurations are rejected", "[aero][lbm]") {
    const SolidMask2D mask = fluid_mask(16, 8);
    Lbm2DConfig config;
    config.nx = 16;
    config.ny = 8;
    config.inlet_ux = 0.05;
    REQUIRE_FALSE(validate(config, mask));

    Lbm2DConfig bad = config;
    bad.tau = 0.5;
    CHECK(validate(bad, mask));
    bad = config;
    bad.nx = 15;
    CHECK(validate(bad, mask));
    bad = config;
    bad.top = Wall{WallKind::periodic, 0.0};
    CHECK(validate(bad, mask));
    bad = config;
    bad.body_force_x = std::numeric_limits<double>::infinity();
    CHECK(validate(bad, mask));
    bad = config;
    bad.smagorinsky = Smagorinsky{true, -0.1};
    CHECK(validate(bad, mask));
    SolidMask2D blocked = mask;
    blocked.solid[0] = 1; // inlet column
    CHECK(validate(config, blocked));
    CHECK_FALSE(make_cpu_solver(bad, mask));
    bad = config;
    bad.nx = 2;
    CHECK(validate(bad, fluid_mask(2, 8)));
}

TEST_CASE("CPU D2Q9 throughput", "[.benchmark][aero][lbm]") {
    const CylinderSetup setup = cylinder_setup(600, 400, 20.0, 160.0, 200.5, 0.05, 100.0);
    auto solver = take(make_cpu_solver(setup.config, setup.mask));
    require_ok(solver->step(20));
    const auto start = std::chrono::steady_clock::now();
    const std::size_t steps = 500;
    require_ok(solver->step(steps));
    const double seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    const double mlups = static_cast<double>(steps) * 600.0 * 400.0 / seconds / 1e6;
    std::cout << "measured: CPU D2Q9 600x400 cylinder " << mlups << " MLUPS ("
              << static_cast<double>(steps) / seconds << " steps/s)\n";
    CHECK(mlups > 0.0);
}
