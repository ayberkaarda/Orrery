#pragma once

#include <cmath>
#include <compare>
#include <numbers>

namespace orrery::core {

// Plane angle. Stored in radians; construction and extraction are explicit.
class Angle {
public:
    constexpr Angle() noexcept = default;

    [[nodiscard]] static constexpr Angle from_radians(double radians) noexcept {
        return Angle(radians);
    }
    [[nodiscard]] static constexpr Angle from_degrees(double degrees) noexcept {
        return Angle(degrees * (std::numbers::pi / 180.0));
    }

    [[nodiscard]] constexpr double radians() const noexcept { return radians_; }
    [[nodiscard]] constexpr double degrees() const noexcept {
        return radians_ * (180.0 / std::numbers::pi);
    }

    constexpr Angle operator-() const noexcept { return Angle(-radians_); }
    friend constexpr Angle operator+(Angle lhs, Angle rhs) noexcept {
        return Angle(lhs.radians_ + rhs.radians_);
    }
    friend constexpr Angle operator-(Angle lhs, Angle rhs) noexcept {
        return Angle(lhs.radians_ - rhs.radians_);
    }
    friend constexpr Angle operator*(Angle lhs, double factor) noexcept {
        return Angle(lhs.radians_ * factor);
    }
    friend constexpr Angle operator*(double factor, Angle rhs) noexcept {
        return Angle(factor * rhs.radians_);
    }

    friend constexpr bool operator==(const Angle& lhs, const Angle& rhs) noexcept = default;
    friend constexpr std::partial_ordering operator<=>(Angle lhs, Angle rhs) noexcept {
        return lhs.radians_ <=> rhs.radians_;
    }

private:
    constexpr explicit Angle(double radians) noexcept : radians_(radians) {}

    double radians_ = 0.0;
};

// Equivalent angle in [0, 2 pi).
[[nodiscard]] inline Angle wrap_two_pi(Angle a) noexcept {
    constexpr double two_pi = 2.0 * std::numbers::pi;
    double r = std::fmod(a.radians(), two_pi);
    if (r < 0.0) {
        r += two_pi;
    }
    // fmod of a tiny negative value plus 2 pi can round up to exactly 2 pi.
    if (r >= two_pi) {
        r = 0.0;
    }
    return Angle::from_radians(r);
}

// Equivalent angle in [-pi, pi). The signed reduction is exact (IEEE
// remainder) and leaves inputs already in range unchanged, so small negative
// angles keep their full precision.
[[nodiscard]] inline Angle wrap_pi(Angle a) noexcept {
    constexpr double two_pi = 2.0 * std::numbers::pi;
    double r = std::remainder(a.radians(), two_pi);
    // remainder returns [-pi, pi]; map the upper end onto the half-open range.
    if (r >= std::numbers::pi) {
        r -= two_pi;
    }
    return Angle::from_radians(r);
}

} // namespace orrery::core
