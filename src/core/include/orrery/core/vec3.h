#pragma once

#include "orrery/core/quantity.h"

namespace orrery::core {

// Cartesian 3-vector whose components share one quantity type.
template <QuantityType Q> struct Vec3 {
    using quantity = Q;

    Q x{};
    Q y{};
    Q z{};

    constexpr Vec3 operator-() const noexcept { return Vec3{-x, -y, -z}; }
    constexpr Vec3& operator+=(const Vec3& rhs) noexcept {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }
    constexpr Vec3& operator-=(const Vec3& rhs) noexcept {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    friend constexpr Vec3 operator+(const Vec3& lhs, const Vec3& rhs) noexcept {
        return Vec3{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
    }
    friend constexpr Vec3 operator-(const Vec3& lhs, const Vec3& rhs) noexcept {
        return Vec3{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
    }
    friend constexpr Vec3 operator*(const Vec3& lhs, double factor) noexcept {
        return Vec3{lhs.x * factor, lhs.y * factor, lhs.z * factor};
    }
    friend constexpr Vec3 operator*(double factor, const Vec3& rhs) noexcept {
        return Vec3{factor * rhs.x, factor * rhs.y, factor * rhs.z};
    }
    friend constexpr Vec3 operator/(const Vec3& lhs, double divisor) noexcept {
        return Vec3{lhs.x / divisor, lhs.y / divisor, lhs.z / divisor};
    }

    friend constexpr bool operator==(const Vec3& lhs, const Vec3& rhs) noexcept = default;
};

// Component-wise scaling by a quantity, e.g. velocity * time -> length.
template <QuantityType Q, QuantityType S>
constexpr Vec3<decltype(Q{} * S{})> operator*(const Vec3<Q>& v, S s) noexcept {
    return {v.x * s, v.y * s, v.z * s};
}

template <QuantityType Q, QuantityType S>
constexpr Vec3<decltype(S{} * Q{})> operator*(S s, const Vec3<Q>& v) noexcept {
    return {s * v.x, s * v.y, s * v.z};
}

template <QuantityType Q, QuantityType S>
constexpr Vec3<decltype(Q{} / S{})> operator/(const Vec3<Q>& v, S s) noexcept {
    return {v.x / s, v.y / s, v.z / s};
}

template <QuantityType A, QuantityType B>
[[nodiscard]] constexpr auto dot(const Vec3<A>& a, const Vec3<B>& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

template <QuantityType A, QuantityType B>
[[nodiscard]] constexpr auto cross(const Vec3<A>& a, const Vec3<B>& b) noexcept {
    using R = decltype(A{} * B{});
    return Vec3<R>{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

template <QuantityType Q> [[nodiscard]] inline Q norm(const Vec3<Q>& v) noexcept {
    return sqrt(dot(v, v));
}

template <QuantityType Q> [[nodiscard]] inline bool isfinite(const Vec3<Q>& v) noexcept {
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

} // namespace orrery::core
