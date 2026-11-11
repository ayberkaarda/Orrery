#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <type_traits>

#include "orrery/core/angle.h"
#include "orrery/core/dimension.h"
#include "orrery/core/quantity.h"
#include "orrery/core/vec3.h"

using namespace orrery::core;
using Catch::Matchers::WithinRel;

// Dimension algebra, checked at compile time.
static_assert(std::is_same_v<DimensionProduct<dim::Velocity, dim::Time>, dim::Length>);
static_assert(std::is_same_v<DimensionQuotient<dim::Length, dim::Time>, dim::Velocity>);
static_assert(std::is_same_v<DimensionQuotient<dim::Velocity, dim::Time>, dim::Acceleration>);
static_assert(std::is_same_v<DimensionProduct<dim::Mass, dim::Acceleration>, dim::Force>);
static_assert(std::is_same_v<DimensionQuotient<dim::Force, dim::Area>, dim::Pressure>);
static_assert(std::is_same_v<DimensionPower<dim::Length, 3>, dim::Volume>);
static_assert(std::is_same_v<DimensionSquareRoot<dim::Area>, dim::Length>);
static_assert(std::is_same_v<DimensionQuotient<dim::GravitationalParameter, dim::Mass>,
                             dim::GravitationalConstant>);
static_assert(has_integer_square_root<dim::SpecificEnergy>);
static_assert(!has_integer_square_root<dim::Length>);
static_assert(std::is_same_v<DimensionQuotient<dim::Length, dim::Length>, dim::Dimensionless>);

// Quantity operators produce the expected types.
static_assert(std::is_same_v<decltype(si::Velocity{} * si::Time{}), si::Length>);
static_assert(std::is_same_v<decltype(si::Length{} / si::Time{}), si::Velocity>);
static_assert(std::is_same_v<decltype(si::Velocity{} / si::Time{}), si::Acceleration>);
static_assert(std::is_same_v<decltype(si::Mass{} * si::Acceleration{}), si::Force>);
static_assert(std::is_same_v<decltype(si::Force{} / si::Area{}), si::Pressure>);
static_assert(std::is_same_v<decltype(sqrt(space::SpecificEnergy{})), space::Velocity>);
static_assert(std::is_same_v<decltype(1.0 / space::Time{}), space::Frequency>);
static_assert(std::is_same_v<decltype(cross(Vec3<space::Length>{}, Vec3<space::Velocity>{})),
                             Vec3<space::SpecificAngularMomentum>>);

// No implicit conversions from or to double, and no mixing of unit systems.
static_assert(!std::is_convertible_v<double, si::Length>);
static_assert(!std::is_convertible_v<si::Length, double>);
static_assert(std::is_constructible_v<si::Length, double>);
static_assert(!std::is_convertible_v<si::Length, space::Length>);
static_assert(!std::is_constructible_v<space::Length, si::Length>);
template <typename A, typename B>
concept Addable = requires(A a, B b) { a + b; };
static_assert(Addable<si::Length, si::Length>);
static_assert(!Addable<si::Length, si::Time>);
static_assert(!Addable<si::Length, space::Length>);
static_assert(!Addable<si::Length, double>);
template <typename A, typename B>
concept Multipliable = requires(A a, B b) { a * b; };
static_assert(!Multipliable<si::Length, space::Length>);

// A quantity is exactly one double, so SoA columns of quantities are dense.
static_assert(sizeof(si::Length) == sizeof(double));
static_assert(std::is_trivially_copyable_v<space::Velocity>);

// Constexpr arithmetic.
static_assert((si::Length(3.0) + si::Length(4.0)).value() == 7.0);
static_assert((si::Length(6.0) / si::Time(2.0)).value() == 3.0);
static_assert(si::Length(1.0) < si::Length(2.0));
static_assert(-si::Length(1.0) == si::Length(-1.0));

TEST_CASE("quantity arithmetic", "[core][quantity]") {
    const si::Length distance(100.0);
    const si::Time duration(4.0);
    const si::Velocity speed = distance / duration;
    CHECK(speed.value() == 25.0);

    si::Length accumulated(1.0);
    accumulated += si::Length(2.0);
    accumulated -= si::Length(0.5);
    accumulated *= 2.0;
    accumulated /= 5.0;
    CHECK_THAT(accumulated.value(), WithinRel(1.0, 1e-15));

    const si::Pressure pressure = si::Force(10.0) / si::Area(4.0);
    CHECK(pressure.value() == 2.5);

    CHECK(abs(si::Length(-3.0)) == si::Length(3.0));
    CHECK(sqrt(si::Area(9.0)) == si::Length(3.0));
    CHECK(isfinite(si::Length(1.0)));
}

TEST_CASE("vector quantities", "[core][quantity]") {
    const Vec3<space::Length> r{space::Length(1.0), space::Length(2.0), space::Length(2.0)};
    const Vec3<space::Velocity> v{space::Velocity(0.0), space::Velocity(1.0), space::Velocity(0.0)};
    CHECK(norm(r) == space::Length(3.0));

    // The norm neither underflows nor overflows for representable lengths.
    const auto length = [](double x, double y, double z) {
        return norm(Vec3<space::Length>{space::Length(x), space::Length(y), space::Length(z)})
            .value();
    };
    CHECK_THAT(length(3.0e-200, 0.0, 4.0e-200), WithinRel(5.0e-200, 1e-15));
    CHECK_THAT(length(0.0, -3.0e200, 4.0e200), WithinRel(5.0e200, 1e-15));
    CHECK_THAT(length(1.0e308, 1.0e308, 0.0), WithinRel(1.4142135623730951e308, 1e-15));
    CHECK(length(5.0e-324, 0.0, 0.0) == 5.0e-324);
    CHECK(length(0.0, 0.0, 0.0) == 0.0);
    CHECK(length(-0.0, 0.0, -0.0) == 0.0);

    const auto h = cross(r, v);
    CHECK(h.x.value() == -2.0);
    CHECK(h.y.value() == 0.0);
    CHECK(h.z.value() == 1.0);

    const Vec3<space::Length> moved = r + v * space::Time(2.0);
    CHECK(moved.y == space::Length(4.0));
    CHECK(dot(r, r).value() == 9.0);
}

TEST_CASE("angles", "[core][quantity]") {
    const Angle right = Angle::from_degrees(90.0);
    CHECK_THAT(right.radians(), WithinRel(1.5707963267948966, 1e-15));
    CHECK_THAT(right.degrees(), WithinRel(90.0, 1e-15));
    CHECK_THAT(wrap_two_pi(Angle::from_degrees(-90.0)).degrees(), WithinRel(270.0, 1e-14));
    CHECK_THAT(wrap_pi(Angle::from_degrees(270.0)).degrees(), WithinRel(-90.0, 1e-14));
    CHECK(wrap_two_pi(Angle::from_radians(-1e-300)).radians() < 6.2831853071795865);
}
