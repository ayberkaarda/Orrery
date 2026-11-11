#pragma once

#include "orrery/core/quantity.h"

namespace orrery::core {

namespace constants {

// Astronomical unit in metres, exact by definition (IAU 2012 Resolution B2).
inline constexpr si::Length astronomical_unit{149597870700.0};

// Day in seconds, exact by definition (86400 SI seconds).
inline constexpr si::Time day{86400.0};

// Numerical value of the Gauss gravitational constant k (IAU 1976).
// k has fractional dimensions (AU^(3/2) Msun^(-1/2) day^(-1)), which Quantity
// cannot express, so only its square is exposed as a typed value below.
inline constexpr double gauss_constant_value = 0.01720209895;

// Newton's constant in space units: G = k^2 AU^3 Msun^-1 day^-2.
inline constexpr space::GravitationalConstant gravitational_constant_space{gauss_constant_value *
                                                                           gauss_constant_value};

// Heliocentric gravitational parameter in space units: GM_sun = k^2 AU^3 day^-2.
inline constexpr space::GravitationalParameter gm_sun_space{gauss_constant_value *
                                                            gauss_constant_value};

// Heliocentric gravitational parameter in SI, derived exactly from k, the AU
// and the day: k^2 AU^3 / day^2 = 1.3271244004193...e20 m^3 s^-2. It agrees
// with the measured TDB-compatible value (1.32712440041e20) to about 7.1e-12
// relative (7.08e-12).
inline constexpr si::GravitationalParameter gm_sun_si{
    gauss_constant_value * gauss_constant_value * astronomical_unit.value() *
    astronomical_unit.value() * astronomical_unit.value() / (day.value() * day.value())};

// Newtonian constant of gravitation, CODATA 2018 (relative uncertainty 2.2e-5).
inline constexpr si::GravitationalConstant gravitational_constant_si{6.67430e-11};

// Solar mass in kilograms.
// APPROXIMATION: solar mass in kg derived as GM_sun / G - G is known only to
// about 2.2e-5 relative while GM_sun is known to about 1e-10 - every mass
// converted between space and SI units carries this 2.2e-5 relative
// uncertainty; gravitational parameters (G*M) convert exactly.
inline constexpr si::Mass solar_mass{gm_sun_si.value() / gravitational_constant_si.value()};

} // namespace constants

namespace detail {

// base^exponent by squaring, defined for every int exponent: the magnitude is
// taken in unsigned arithmetic (no negation of INT_MIN), negative exponents
// use the reciprocal base, and at most 32 squarings are performed. Results
// beyond the double range saturate to 0 or infinity.
constexpr double integer_power(double base, int exponent) noexcept {
    const auto magnitude_bits = static_cast<unsigned int>(exponent);
    unsigned int magnitude = exponent >= 0 ? magnitude_bits : 0U - magnitude_bits;
    double factor = exponent >= 0 ? base : 1.0 / base;
    double result = 1.0;
    while (magnitude != 0U) {
        if ((magnitude & 1U) != 0U) {
            result *= factor;
        }
        magnitude >>= 1U;
        if (magnitude != 0U) {
            factor *= factor;
        }
    }
    return result;
}

} // namespace detail

// Number of SI base units in one space base unit of dimension D:
// AU^L * Msun^M * day^T expressed in m^L kg^M s^T.
template <DimensionType D> constexpr double space_to_si_factor() noexcept {
    return detail::integer_power(constants::astronomical_unit.value(), D::length) *
           detail::integer_power(constants::solar_mass.value(), D::mass) *
           detail::integer_power(constants::day.value(), D::time);
}

template <DimensionType D>
[[nodiscard]] constexpr Quantity<D, SiUnits> to_si(Quantity<D, SpaceUnits> q) noexcept {
    return Quantity<D, SiUnits>(q.value() * space_to_si_factor<D>());
}

template <DimensionType D>
[[nodiscard]] constexpr Quantity<D, SpaceUnits> to_space(Quantity<D, SiUnits> q) noexcept {
    return Quantity<D, SpaceUnits>(q.value() / space_to_si_factor<D>());
}

// Identity conversions, so generic boundary code can call to_si / to_space
// regardless of the source system.
template <DimensionType D>
[[nodiscard]] constexpr Quantity<D, SiUnits> to_si(Quantity<D, SiUnits> q) noexcept {
    return q;
}

template <DimensionType D>
[[nodiscard]] constexpr Quantity<D, SpaceUnits> to_space(Quantity<D, SpaceUnits> q) noexcept {
    return q;
}

// Explicit construction from common non-base units.
namespace si_units {
[[nodiscard]] constexpr si::Length metres(double v) noexcept {
    return si::Length(v);
}
[[nodiscard]] constexpr si::Length kilometres(double v) noexcept {
    return si::Length(v * 1.0e3);
}
[[nodiscard]] constexpr si::Mass kilograms(double v) noexcept {
    return si::Mass(v);
}
[[nodiscard]] constexpr si::Time seconds(double v) noexcept {
    return si::Time(v);
}
[[nodiscard]] constexpr si::Velocity metres_per_second(double v) noexcept {
    return si::Velocity(v);
}
[[nodiscard]] constexpr si::Acceleration metres_per_second_squared(double v) noexcept {
    return si::Acceleration(v);
}
[[nodiscard]] constexpr si::Pressure pascals(double v) noexcept {
    return si::Pressure(v);
}
} // namespace si_units

namespace space_units {
[[nodiscard]] constexpr space::Length astronomical_units(double v) noexcept {
    return space::Length(v);
}
[[nodiscard]] constexpr space::Mass solar_masses(double v) noexcept {
    return space::Mass(v);
}
[[nodiscard]] constexpr space::Time days(double v) noexcept {
    return space::Time(v);
}
[[nodiscard]] constexpr space::Velocity au_per_day(double v) noexcept {
    return space::Velocity(v);
}
} // namespace space_units

} // namespace orrery::core
