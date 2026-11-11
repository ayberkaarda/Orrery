#include "orrery/core/black_hole.h"

#include <cmath>
#include <numbers>

namespace orrery::core {

std::optional<space::Length> tidal_radius(space::Length body_radius, space::Mass body_mass,
                                          space::Mass black_hole_mass) {
    const double r = body_radius.value();
    const double m = body_mass.value();
    const double big_m = black_hole_mass.value();
    if (!(r > 0.0) || !(m > 0.0) || !(big_m > 0.0)) {
        return std::nullopt;
    }
    const double rt = r * std::cbrt(big_m / m);
    if (!std::isfinite(rt)) {
        return std::nullopt;
    }
    return space::Length(rt);
}

std::optional<space::Velocity> pw_circular_speed(space::Mass black_hole_mass,
                                                 space::Length radius) {
    const double big_m = black_hole_mass.value();
    const double r = radius.value();
    const double rs = schwarzschild_radius(black_hole_mass).value();
    if (!(big_m > 0.0) || !(r > rs) || !std::isfinite(r)) {
        return std::nullopt;
    }
    const double gm = constants::gravitational_constant_space.value() * big_m;
    return space::Velocity(std::sqrt(gm * r) / (r - rs));
}

std::optional<space::Time> pw_circular_period(space::Mass black_hole_mass, space::Length radius) {
    const auto speed = pw_circular_speed(black_hole_mass, radius);
    if (!speed) {
        return std::nullopt;
    }
    return space::Time(2.0 * std::numbers::pi * radius.value() / speed->value());
}

} // namespace orrery::core
