#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <string>

#include "orrery/aero/lbm/force_history.h"
#include "orrery/aero/lbm/lattice_units.h"
#include "orrery/aero/lbm/lbm2d.h"

using namespace orrery::aero::lbm;
using Catch::Approx;

namespace {

// Air past a 2 cm cylinder at 0.075 m/s: Re = 100.
SiFlow cylinder_in_air() {
    return SiFlow{0.02, 0.075, 1.5e-5, 1.225};
}

} // namespace

TEST_CASE("Lattice units follow from resolution and lattice velocity", "[aero][lbm][units]") {
    auto made = LatticeUnits::create(cylinder_in_air(), LatticeResolution{40.0, 0.05});
    REQUIRE(made);
    const LatticeUnits& u = made.value();
    CHECK(u.reynolds() == Approx(100.0).epsilon(1e-14));
    CHECK(u.dx() == Approx(0.02 / 40.0).epsilon(1e-15));
    CHECK(u.dt() == Approx(0.05 * u.dx() / 0.075).epsilon(1e-15));
    // Re is the same in lattice units: u_lat N / nu_lat.
    CHECK(0.05 * 40.0 / u.lattice_viscosity() == Approx(100.0).epsilon(1e-13));
    CHECK(u.tau() == Approx(3.0 * 0.05 * 40.0 / 100.0 + 0.5).epsilon(1e-14));
    CHECK(u.tau() == Approx(tau_of_viscosity(u.lattice_viscosity())).epsilon(1e-15));
    CHECK(viscosity_of_tau(u.tau()) == Approx(u.lattice_viscosity()).epsilon(1e-14));
    CHECK(u.mach() == Approx(0.05 * std::sqrt(3.0)).epsilon(1e-15));
    CHECK(u.cells_per_length() == 40.0);
}

TEST_CASE("Lattice to SI conversions round-trip", "[aero][lbm][units]") {
    const LatticeUnits u =
        LatticeUnits::create(cylinder_in_air(), LatticeResolution{25.0, 0.04}).value();
    CHECK(u.velocity_to_si(u.lattice_velocity()) == Approx(0.075).epsilon(1e-14));
    for (const double v : {1e-3, 0.3, 12.5}) {
        CHECK(u.velocity_to_si(u.velocity_to_lattice(v)) == Approx(v).epsilon(1e-14));
        CHECK(u.length_to_si(u.length_to_lattice(v)) == Approx(v).epsilon(1e-14));
        CHECK(u.time_to_si(u.time_to_lattice(v)) == Approx(v).epsilon(1e-14));
    }
    CHECK(u.viscosity_to_si(u.lattice_viscosity()) == Approx(1.5e-5).epsilon(1e-13));
    CHECK(u.pressure_to_si(1.0) == 0.0);
    // Gauge pressure: dp = c_s^2 drho in lattice units, scaled by rho (dx/dt)^2.
    const double scale = 1.225 * std::pow(u.dx() / u.dt(), 2);
    CHECK(u.pressure_to_si(1.003) == Approx(0.003 / 3.0 * scale).epsilon(1e-12));
    CHECK(u.vorticity_to_si(0.01) == Approx(0.01 / u.dt()).epsilon(1e-15));
}

TEST_CASE("Force coefficients are the same in lattice and SI units", "[aero][lbm][units]") {
    const LatticeUnits u =
        LatticeUnits::create(cylinder_in_air(), LatticeResolution{30.0, 0.05}).value();
    const double fx = 0.0021;
    const double fy = -0.0004;
    const Coefficients lattice = force_coefficients(fx, fy, u.lattice_velocity(), 30.0);
    const double q_si = 0.5 * 1.225 * 0.075 * 0.075 * 0.02; // N/m per unit coefficient
    CHECK(u.force_per_span_to_si(fx) / q_si == Approx(lattice.drag).epsilon(1e-12));
    CHECK(u.force_per_span_to_si(fy) / q_si == Approx(lattice.lift).epsilon(1e-12));
}

TEST_CASE("The Mach guard refuses lattice Mach numbers of 0.1 or more", "[aero][lbm][units]") {
    const double limit = 0.1 / std::sqrt(3.0); // u_lat at Ma = 0.1
    CHECK(LatticeUnits::create(cylinder_in_air(), LatticeResolution{20.0, 0.99 * limit}));
    auto refused = LatticeUnits::create(cylinder_in_air(), LatticeResolution{20.0, limit});
    REQUIRE_FALSE(refused);
    CHECK(refused.error().find("Mach") != std::string::npos);
    CHECK_FALSE(LatticeUnits::create(cylinder_in_air(), LatticeResolution{20.0, 0.1}));
    CHECK(lattice_mach(-0.05) == Approx(0.05 * std::sqrt(3.0)));

    // The solver applies the same guard to the inlet and the wall speeds.
    Lbm2DConfig config;
    config.nx = 16;
    config.ny = 8;
    config.inlet_ux = 0.06;
    const SolidMask2D mask = fluid_mask(16, 8);
    REQUIRE(validate(config, mask));
    CHECK(validate(config, mask)->find("Mach") != std::string::npos);
    config.inlet_ux = 0.05;
    CHECK_FALSE(validate(config, mask));
    config.top = Wall{WallKind::no_slip, 0.07};
    CHECK(validate(config, mask));
}

TEST_CASE("Lattice units reject invalid input", "[aero][lbm][units]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(LatticeUnits::create(SiFlow{0.0, 1.0, 1e-5, 1.2}, {}));
    CHECK_FALSE(LatticeUnits::create(SiFlow{1.0, -1.0, 1e-5, 1.2}, {}));
    CHECK_FALSE(LatticeUnits::create(SiFlow{1.0, 1.0, nan, 1.2}, {}));
    CHECK_FALSE(LatticeUnits::create(SiFlow{1.0, 1.0, 1e-5, 0.0}, {}));
    CHECK_FALSE(LatticeUnits::create(cylinder_in_air(), LatticeResolution{0.0, 0.05}));
    CHECK_FALSE(LatticeUnits::create(cylinder_in_air(), LatticeResolution{20.0, -0.01}));
    CHECK_FALSE(
        LatticeUnits::create(SiFlow{1e300, 1e-300, 1e-300, 1.0}, LatticeResolution{1e-300, 0.05}));
}
