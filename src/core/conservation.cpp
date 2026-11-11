#include "orrery/core/conservation.h"

#include <cstddef>
#include <limits>
#include <vector>

namespace orrery::core {

namespace {

// diff / scale with the zero handling documented in conservation.h.
double normalised(double diff, double scale) noexcept {
    if (scale > 0.0) {
        return diff / scale;
    }
    return diff == 0.0 ? 0.0 : std::numeric_limits<double>::infinity();
}

} // namespace

ConservationDiagnostics::ConservationDiagnostics(const SimState& initial, const IForceModel& forces)
    : baseline_(measure(initial, forces)) {}

ConservedQuantities ConservationDiagnostics::measure(const SimState& state,
                                                     const IForceModel& forces) {
    const BodyStore& bodies = state.bodies();
    const std::size_t n = bodies.size();
    std::vector<Vec3<space::Length>> positions(n);
    for (std::size_t i = 0; i < n; ++i) {
        positions[i] = bodies.position(i);
    }

    ConservedQuantities q;
    Vec3<decltype(space::Mass{} * space::Length{})> mass_moment{};
    for (std::size_t i = 0; i < n; ++i) {
        if (!bodies.exerts_gravity(i)) {
            continue;
        }
        const space::Mass m = bodies.mass(i);
        const Vec3<space::Velocity> v = bodies.velocity(i);
        const Vec3<space::Momentum> p = m * v;
        const Vec3<space::AngularMomentum> l = m * cross(positions[i], v);
        q.total_mass += m;
        q.kinetic_energy += 0.5 * (m * dot(v, v));
        q.linear_momentum += p;
        q.angular_momentum += l;
        mass_moment += m * positions[i];
        q.momentum_scale += norm(p);
        q.angular_momentum_scale += norm(l);
    }
    q.potential_energy = forces.potential_energy(bodies, positions);
    q.total_energy = q.kinetic_energy + q.potential_energy;
    if (q.total_mass.value() > 0.0) {
        q.center_of_mass = mass_moment / q.total_mass;
        q.center_of_mass_velocity = q.linear_momentum / q.total_mass;
    }
    return q;
}

ConservationDrift
ConservationDiagnostics::drift(const ConservedQuantities& current) const noexcept {
    const ConservedQuantities& b = baseline_;
    ConservationDrift d;

    const double de = abs(current.total_energy - b.total_energy).value();
    const double e0 = abs(b.total_energy).value();
    d.relative_energy =
        e0 > 0.0 ? de / e0 : normalised(de, (b.kinetic_energy + abs(b.potential_energy)).value());

    d.linear_momentum = normalised(norm(current.linear_momentum - b.linear_momentum).value(),
                                   b.momentum_scale.value());
    d.angular_momentum = normalised(norm(current.angular_momentum - b.angular_momentum).value(),
                                    b.angular_momentum_scale.value());
    const double velocity_scale =
        b.total_mass.value() > 0.0 ? b.momentum_scale.value() / b.total_mass.value() : 0.0;
    d.center_of_mass_velocity = normalised(
        norm(current.center_of_mass_velocity - b.center_of_mass_velocity).value(), velocity_scale);
    return d;
}

ConservationDrift ConservationDiagnostics::drift(const SimState& state,
                                                 const IForceModel& forces) const {
    return drift(measure(state, forces));
}

} // namespace orrery::core
