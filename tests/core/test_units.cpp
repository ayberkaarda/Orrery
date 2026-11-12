#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>
#include <cmath>
#include <limits>

#include "orrery/core/units.h"

using namespace orrery::core;
using Catch::Matchers::WithinRel;

static_assert(constants::astronomical_unit.value() == 149597870700.0);
static_assert(constants::day.value() == 86400.0);
static_assert(constants::gauss_constant_value == 0.01720209895);
static_assert(SpaceUnits::id == UnitSystemId::space);
static_assert(SiUnits::id == UnitSystemId::si);
static_assert(space_to_si_factor<dim::Dimensionless>() == 1.0);
static_assert(space_to_si_factor<dim::Length>() == 149597870700.0);
static_assert(space_to_si_factor<dim::Time>() == 86400.0);
// Constant evaluation diagnoses undefined behaviour, so these also prove the
// minimum exponent is handled without signed overflow.
static_assert(detail::integer_power(2.0, std::numeric_limits<int>::min()) == 0.0);
static_assert(detail::integer_power(2.0, -1074) == std::numeric_limits<double>::denorm_min());
static_assert(detail::integer_power(3.0, 5) == 243.0);
static_assert(detail::integer_power(7.0, 0) == 1.0);

TEST_CASE("base unit conversions use the defining constants", "[core][units]") {
    CHECK(to_si(space::Length(1.0)).value() == 149597870700.0);
    CHECK(to_si(space::Time(1.0)).value() == 86400.0);
    CHECK(to_space(si::Length(149597870700.0)).value() == 1.0);
    CHECK(to_space(si::Time(86400.0)).value() == 1.0);
    CHECK_THAT(to_si(space::Velocity(1.0)).value(), WithinRel(149597870700.0 / 86400.0, 1e-15));
    CHECK_THAT(to_si(space::Acceleration(1.0)).value(),
               WithinRel(149597870700.0 / (86400.0 * 86400.0), 1e-15));
}

TEST_CASE("gravitational parameter of the Sun", "[core][units]") {
    // k^2 in AU^3 / day^2.
    CHECK(constants::gm_sun_space.value() == 0.01720209895 * 0.01720209895);
    // Converted to SI it reproduces the measured heliocentric gravitational
    // constant (1.32712440041e20 m^3 s^-2) to better than 1e-10.
    CHECK_THAT(to_si(constants::gm_sun_space).value(), WithinRel(1.32712440041e20, 1e-10));
    CHECK_THAT(to_si(constants::gm_sun_space).value(),
               WithinRel(constants::gm_sun_si.value(), 1e-15));
    // G in space units is k^2 by construction; in SI it maps to CODATA G.
    CHECK_THAT(to_si(constants::gravitational_constant_space).value(),
               WithinRel(constants::gravitational_constant_si.value(), 1e-14));
    // One solar mass is about 1.98841e30 kg.
    CHECK_THAT(constants::solar_mass.value(), WithinRel(1.98841e30, 1e-5));
    CHECK_THAT(to_si(space::Mass(1.0)).value(), WithinRel(constants::solar_mass.value(), 1e-15));
}

TEST_CASE("SI <-> space round trips", "[core][units]") {
    const std::array<double, 7> values{0.0, 1e-30, 1e-6, 1.0, 3.75, 1e12, -42.5};
    for (const double x : values) {
        CHECK_THAT(to_si(to_space(si::Length(x))).value(), WithinRel(x, 1e-15));
        CHECK_THAT(to_space(to_si(space::Length(x))).value(), WithinRel(x, 1e-15));
        CHECK_THAT(to_si(to_space(si::Mass(x))).value(), WithinRel(x, 1e-15));
        CHECK_THAT(to_si(to_space(si::Time(x))).value(), WithinRel(x, 1e-15));
        CHECK_THAT(to_si(to_space(si::Velocity(x))).value(), WithinRel(x, 1e-15));
        CHECK_THAT(to_space(to_si(space::Acceleration(x))).value(), WithinRel(x, 1e-15));
        CHECK_THAT(to_si(to_space(si::Pressure(x))).value(), WithinRel(x, 1e-15));
        CHECK_THAT(to_space(to_si(space::GravitationalParameter(x))).value(), WithinRel(x, 1e-15));
    }
}

TEST_CASE("conversion commutes with dimension algebra", "[core][units]") {
    const space::Length distance(2.5);
    const space::Time duration(7.0);
    const si::Velocity a = to_si(distance / duration);
    const si::Velocity b = to_si(distance) / to_si(duration);
    CHECK_THAT(a.value(), WithinRel(b.value(), 1e-15));

    const space::Mass mass(3.0e-6);
    const space::Pressure p = mass * space::Acceleration(1.0) / (distance * distance);
    const si::Pressure p_si =
        to_si(mass) * to_si(space::Acceleration(1.0)) / (to_si(distance) * to_si(distance));
    CHECK_THAT(to_si(p).value(), WithinRel(p_si.value(), 1e-14));
}

TEST_CASE("documented GM agreement bound", "[core][units]") {
    // k^2 AU^3 / day^2 differs from the measured 1.32712440041e20
    // by about 7.08e-12 relative; docs/physics.md and units.h state 7.1e-12.
    const double measured = 1.32712440041e20;
    const double relative = std::abs(constants::gm_sun_si.value() - measured) / measured;
    CHECK(relative < 7.1e-12);
    CHECK(relative > 7.0e-12);
}

TEST_CASE("integer powers are defined over the whole exponent range", "[core][units]") {
    // the minimum exponent must not negate INT_MIN. The exponents
    // are read through volatile so the calls are evaluated at run time.
    volatile int min_exponent = std::numeric_limits<int>::min();
    volatile int max_exponent = std::numeric_limits<int>::max();
    const double inf = std::numeric_limits<double>::infinity();
    CHECK(detail::integer_power(2.0, min_exponent) == 0.0);
    CHECK(detail::integer_power(0.5, min_exponent) == inf);
    CHECK(detail::integer_power(2.0, max_exponent) == inf);
    CHECK(detail::integer_power(1.0, min_exponent) == 1.0);
    volatile int subnormal_exponent = -1074;
    CHECK(detail::integer_power(2.0, subnormal_exponent) ==
          std::numeric_limits<double>::denorm_min());
    CHECK(detail::integer_power(10.0, 3) == 1000.0);
    CHECK_THAT(detail::integer_power(10.0, -3), WithinRel(1e-3, 1e-15));

    using Extreme = Quantity<Dimension<std::numeric_limits<int>::min(), 0, 0>, SpaceUnits>;
    volatile double one = 1.0;
    CHECK(to_si(Extreme(one)).value() == 0.0);
}

TEST_CASE("named unit helpers", "[core][units]") {
    CHECK(si_units::kilometres(1.5) == si::Length(1500.0));
    CHECK(si_units::pascals(101325.0).value() == 101325.0);
    CHECK(space_units::days(365.25) == space::Time(365.25));
    CHECK(to_si(space_units::astronomical_units(1.0)) == constants::astronomical_unit);
}
