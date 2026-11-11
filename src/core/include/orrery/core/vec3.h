#pragma once

#include <algorithm>
#include <cmath>

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

namespace detail {

// Euclidean length with the components scaled by an exact power of two, so
// that no square overflows or underflows significantly: accurate for every
// finite vector. Non-finite components give the unscaled result (inf or NaN).
[[nodiscard]] inline double scaled_norm(double x, double y, double z) noexcept {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        return std::sqrt(x * x + y * y + z * z);
    }
    const double largest = std::max(std::abs(x), std::max(std::abs(y), std::abs(z)));
    if (largest == 0.0) {
        return 0.0;
    }
    int e = 0;
    static_cast<void>(std::frexp(largest, &e));
    const double xs = std::ldexp(x, -e);
    const double ys = std::ldexp(y, -e);
    const double zs = std::ldexp(z, -e);
    return std::ldexp(std::sqrt(xs * xs + ys * ys + zs * zs), e);
}

} // namespace detail

// Euclidean length. Ordinary magnitudes (squared length in [2^-900, 2^1000]:
// nothing overflowed, and a component whose square underflowed is below the
// rounding of the sum) take the direct sqrt(x^2 + y^2 + z^2); every other
// vector is scaled first, so tiny and huge lengths are not lost to underflow
// or overflow.
template <QuantityType Q> [[nodiscard]] inline Q norm(const Vec3<Q>& v) noexcept {
    const double x = v.x.value();
    const double y = v.y.value();
    const double z = v.z.value();
    const double squared = x * x + y * y + z * z;
    if (squared >= 0x1p-900 && squared <= 0x1p+1000) {
        return Q(std::sqrt(squared));
    }
    return Q(detail::scaled_norm(x, y, z));
}

template <QuantityType Q> [[nodiscard]] inline bool isfinite(const Vec3<Q>& v) noexcept {
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

} // namespace orrery::core
