#pragma once

#include <cmath>

namespace orrery::geom {

// Plain double-precision 3-vector for geometry in metres (or in the raw units
// of an imported file before the unit scale is confirmed). Geometry is not
// tied to a quantity type: a mesh is a shape, its unit is a property of the
// mesh, not of each coordinate.
struct Vector3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    constexpr Vector3 operator-() const noexcept { return {-x, -y, -z}; }
    constexpr Vector3& operator+=(const Vector3& rhs) noexcept {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }
    constexpr Vector3& operator-=(const Vector3& rhs) noexcept {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    friend constexpr Vector3 operator+(const Vector3& a, const Vector3& b) noexcept {
        return {a.x + b.x, a.y + b.y, a.z + b.z};
    }
    friend constexpr Vector3 operator-(const Vector3& a, const Vector3& b) noexcept {
        return {a.x - b.x, a.y - b.y, a.z - b.z};
    }
    friend constexpr Vector3 operator*(const Vector3& a, double s) noexcept {
        return {a.x * s, a.y * s, a.z * s};
    }
    friend constexpr Vector3 operator*(double s, const Vector3& a) noexcept {
        return {s * a.x, s * a.y, s * a.z};
    }
    friend constexpr Vector3 operator/(const Vector3& a, double s) noexcept {
        return {a.x / s, a.y / s, a.z / s};
    }
    friend constexpr bool operator==(const Vector3& a, const Vector3& b) noexcept = default;
};

[[nodiscard]] constexpr double dot(const Vector3& a, const Vector3& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

[[nodiscard]] constexpr Vector3 cross(const Vector3& a, const Vector3& b) noexcept {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

[[nodiscard]] inline double norm(const Vector3& a) noexcept {
    return std::hypot(a.x, a.y, a.z);
}

[[nodiscard]] constexpr double squared_norm(const Vector3& a) noexcept {
    return dot(a, a);
}

[[nodiscard]] inline bool isfinite(const Vector3& a) noexcept {
    return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z);
}

// Unit vector along a; the zero vector stays zero.
[[nodiscard]] inline Vector3 normalized(const Vector3& a) noexcept {
    const double n = norm(a);
    return n > 0.0 ? a / n : Vector3{};
}

} // namespace orrery::geom
