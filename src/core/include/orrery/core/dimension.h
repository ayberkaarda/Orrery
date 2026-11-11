#pragma once

#include <type_traits>

namespace orrery::core {

// Physical dimension as integer exponents of length, mass and time.
// Angles are dimensionless and are represented by the separate Angle type.
template <int L, int M, int T> struct Dimension {
    static constexpr int length = L;
    static constexpr int mass = M;
    static constexpr int time = T;
};

template <typename D> struct is_dimension : std::false_type {};
template <int L, int M, int T> struct is_dimension<Dimension<L, M, T>> : std::true_type {};

template <typename D>
concept DimensionType = is_dimension<D>::value;

template <DimensionType A, DimensionType B>
using DimensionProduct = Dimension<A::length + B::length, A::mass + B::mass, A::time + B::time>;

template <DimensionType A, DimensionType B>
using DimensionQuotient = Dimension<A::length - B::length, A::mass - B::mass, A::time - B::time>;

template <DimensionType A, int N>
using DimensionPower = Dimension<A::length * N, A::mass * N, A::time * N>;

// True when every exponent is even, i.e. the square root has integer exponents.
template <DimensionType A>
inline constexpr bool has_integer_square_root =
    A::length % 2 == 0 && A::mass % 2 == 0 && A::time % 2 == 0;

template <DimensionType A>
    requires has_integer_square_root<A>
using DimensionSquareRoot = Dimension<A::length / 2, A::mass / 2, A::time / 2>;

namespace dim {

using Dimensionless = Dimension<0, 0, 0>;
using Length = Dimension<1, 0, 0>;
using Mass = Dimension<0, 1, 0>;
using Time = Dimension<0, 0, 1>;
using Area = Dimension<2, 0, 0>;
using Volume = Dimension<3, 0, 0>;
using Frequency = Dimension<0, 0, -1>;
using Velocity = Dimension<1, 0, -1>;
using Acceleration = Dimension<1, 0, -2>;
using Force = Dimension<1, 1, -2>;
using Pressure = Dimension<-1, 1, -2>;
using Density = Dimension<-3, 1, 0>;
using Energy = Dimension<2, 1, -2>;
using SpecificEnergy = Dimension<2, 0, -2>;
using SpecificAngularMomentum = Dimension<2, 0, -1>;
// G * M of a body.
using GravitationalParameter = Dimension<3, 0, -2>;
// Newton's constant G.
using GravitationalConstant = Dimension<3, -1, -2>;

} // namespace dim

} // namespace orrery::core
