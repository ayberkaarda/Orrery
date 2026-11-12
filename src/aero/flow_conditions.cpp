#include "orrery/aero/flow_conditions.h"

#include <cmath>

namespace orrery::aero {

double FlowConditions::speed_of_sound() const noexcept {
    return std::sqrt(gamma * gas_constant * temperature);
}

double FlowConditions::mach() const noexcept {
    return speed / speed_of_sound();
}

double FlowConditions::reynolds(double length) const noexcept {
    return density * speed * length / viscosity;
}

double FlowConditions::dynamic_pressure() const noexcept {
    return 0.5 * density * speed * speed;
}

geom::Vector3 FlowConditions::drag_direction() const noexcept {
    const double cb = std::cos(sideslip);
    return {std::cos(angle_of_attack) * cb, std::sin(sideslip), std::sin(angle_of_attack) * cb};
}

geom::Vector3 FlowConditions::lift_direction() const noexcept {
    // Perpendicular to the free stream and to the span axis y.
    return geom::normalized(geom::cross(drag_direction(), geom::Vector3{0.0, 1.0, 0.0}));
}

geom::Vector3 FlowConditions::side_direction() const noexcept {
    return geom::cross(lift_direction(), drag_direction());
}

geom::Vector3 FlowConditions::velocity() const noexcept {
    return drag_direction() * speed;
}

} // namespace orrery::aero
