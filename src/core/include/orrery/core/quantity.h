#pragma once

#include <cmath>
#include <compare>
#include <string_view>
#include <type_traits>

#include "orrery/core/dimension.h"

namespace orrery::core {

// Runtime identifier of a unit system (ADR-001).
enum class UnitSystemId {
    // SI: metre, kilogram, second. Aero domain.
    si,
    // Astronomical unit, solar mass, day. Space domain.
    space,
};

// Compile-time unit system tags. A Quantity carries its unit system in its
// type, so values from different domains cannot be mixed without an explicit
// conversion (see units.h).
struct SiUnits {
    static constexpr UnitSystemId id = UnitSystemId::si;
    static constexpr std::string_view name = "SI (m, kg, s)";
};

struct SpaceUnits {
    static constexpr UnitSystemId id = UnitSystemId::space;
    static constexpr std::string_view name = "space (AU, solar mass, day)";
};

template <typename U>
concept UnitSystem = std::is_same_v<U, SiUnits> || std::is_same_v<U, SpaceUnits>;

// A dimensional value: a double expressed in the base units of unit system U.
// Construction from and extraction to double are explicit.
template <DimensionType D, UnitSystem U> class Quantity {
public:
    using dimension = D;
    using unit_system = U;

    constexpr Quantity() noexcept = default;
    constexpr explicit Quantity(double value) noexcept : value_(value) {}

    // Numerical value in the base units of U.
    [[nodiscard]] constexpr double value() const noexcept { return value_; }

    constexpr Quantity operator+() const noexcept { return *this; }
    constexpr Quantity operator-() const noexcept { return Quantity(-value_); }

    constexpr Quantity& operator+=(Quantity rhs) noexcept {
        value_ += rhs.value_;
        return *this;
    }
    constexpr Quantity& operator-=(Quantity rhs) noexcept {
        value_ -= rhs.value_;
        return *this;
    }
    constexpr Quantity& operator*=(double factor) noexcept {
        value_ *= factor;
        return *this;
    }
    constexpr Quantity& operator/=(double divisor) noexcept {
        value_ /= divisor;
        return *this;
    }

    friend constexpr Quantity operator+(Quantity lhs, Quantity rhs) noexcept {
        return Quantity(lhs.value_ + rhs.value_);
    }
    friend constexpr Quantity operator-(Quantity lhs, Quantity rhs) noexcept {
        return Quantity(lhs.value_ - rhs.value_);
    }
    friend constexpr Quantity operator*(Quantity lhs, double factor) noexcept {
        return Quantity(lhs.value_ * factor);
    }
    friend constexpr Quantity operator*(double factor, Quantity rhs) noexcept {
        return Quantity(factor * rhs.value_);
    }
    friend constexpr Quantity operator/(Quantity lhs, double divisor) noexcept {
        return Quantity(lhs.value_ / divisor);
    }

    friend constexpr bool operator==(const Quantity& lhs, const Quantity& rhs) noexcept = default;
    friend constexpr std::partial_ordering operator<=>(Quantity lhs, Quantity rhs) noexcept {
        return lhs.value_ <=> rhs.value_;
    }

private:
    double value_ = 0.0;
};

template <typename Q> struct is_quantity : std::false_type {};
template <DimensionType D, UnitSystem U> struct is_quantity<Quantity<D, U>> : std::true_type {};

template <typename Q>
concept QuantityType = is_quantity<Q>::value;

template <DimensionType A, DimensionType B, UnitSystem U>
constexpr Quantity<DimensionProduct<A, B>, U> operator*(Quantity<A, U> lhs,
                                                        Quantity<B, U> rhs) noexcept {
    return Quantity<DimensionProduct<A, B>, U>(lhs.value() * rhs.value());
}

template <DimensionType A, DimensionType B, UnitSystem U>
constexpr Quantity<DimensionQuotient<A, B>, U> operator/(Quantity<A, U> lhs,
                                                         Quantity<B, U> rhs) noexcept {
    return Quantity<DimensionQuotient<A, B>, U>(lhs.value() / rhs.value());
}

template <DimensionType A, UnitSystem U>
constexpr Quantity<DimensionQuotient<dim::Dimensionless, A>, U>
operator/(double numerator, Quantity<A, U> rhs) noexcept {
    return Quantity<DimensionQuotient<dim::Dimensionless, A>, U>(numerator / rhs.value());
}

template <DimensionType A, UnitSystem U>
    requires has_integer_square_root<A>
[[nodiscard]] inline Quantity<DimensionSquareRoot<A>, U> sqrt(Quantity<A, U> q) noexcept {
    return Quantity<DimensionSquareRoot<A>, U>(std::sqrt(q.value()));
}

template <DimensionType A, UnitSystem U>
[[nodiscard]] constexpr Quantity<A, U> abs(Quantity<A, U> q) noexcept {
    return q.value() < 0.0 ? -q : q;
}

template <DimensionType A, UnitSystem U>
[[nodiscard]] inline bool isfinite(Quantity<A, U> q) noexcept {
    return std::isfinite(q.value());
}

// Named quantities, one set per unit system.
template <UnitSystem U> struct QuantitiesOf {
    using Dimensionless = Quantity<dim::Dimensionless, U>;
    using Length = Quantity<dim::Length, U>;
    using Mass = Quantity<dim::Mass, U>;
    using Time = Quantity<dim::Time, U>;
    using Area = Quantity<dim::Area, U>;
    using Volume = Quantity<dim::Volume, U>;
    using Frequency = Quantity<dim::Frequency, U>;
    using Velocity = Quantity<dim::Velocity, U>;
    using Acceleration = Quantity<dim::Acceleration, U>;
    using Force = Quantity<dim::Force, U>;
    using Pressure = Quantity<dim::Pressure, U>;
    using Density = Quantity<dim::Density, U>;
    using Energy = Quantity<dim::Energy, U>;
    using SpecificEnergy = Quantity<dim::SpecificEnergy, U>;
    using SpecificAngularMomentum = Quantity<dim::SpecificAngularMomentum, U>;
    using GravitationalParameter = Quantity<dim::GravitationalParameter, U>;
    using GravitationalConstant = Quantity<dim::GravitationalConstant, U>;
};

namespace si {
using Q = QuantitiesOf<SiUnits>;
using Dimensionless = Q::Dimensionless;
using Length = Q::Length;
using Mass = Q::Mass;
using Time = Q::Time;
using Area = Q::Area;
using Volume = Q::Volume;
using Frequency = Q::Frequency;
using Velocity = Q::Velocity;
using Acceleration = Q::Acceleration;
using Force = Q::Force;
using Pressure = Q::Pressure;
using Density = Q::Density;
using Energy = Q::Energy;
using SpecificEnergy = Q::SpecificEnergy;
using SpecificAngularMomentum = Q::SpecificAngularMomentum;
using GravitationalParameter = Q::GravitationalParameter;
using GravitationalConstant = Q::GravitationalConstant;
} // namespace si

namespace space {
using Q = QuantitiesOf<SpaceUnits>;
using Dimensionless = Q::Dimensionless;
using Length = Q::Length;
using Mass = Q::Mass;
using Time = Q::Time;
using Area = Q::Area;
using Volume = Q::Volume;
using Frequency = Q::Frequency;
using Velocity = Q::Velocity;
using Acceleration = Q::Acceleration;
using Force = Q::Force;
using Pressure = Q::Pressure;
using Density = Q::Density;
using Energy = Q::Energy;
using SpecificEnergy = Q::SpecificEnergy;
using SpecificAngularMomentum = Q::SpecificAngularMomentum;
using GravitationalParameter = Q::GravitationalParameter;
using GravitationalConstant = Q::GravitationalConstant;
} // namespace space

} // namespace orrery::core
