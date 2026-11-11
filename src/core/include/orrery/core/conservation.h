#pragma once

#include "orrery/core/dimension.h"
#include "orrery/core/force_model.h"
#include "orrery/core/quantity.h"
#include "orrery/core/sim_state.h"
#include "orrery/core/vec3.h"

namespace orrery::core {

namespace space {
// Linear momentum, Msun AU day^-1.
using Momentum = Quantity<Dimension<1, 1, -1>, SpaceUnits>;
// Angular momentum, Msun AU^2 day^-1.
using AngularMomentum = Quantity<Dimension<2, 1, -1>, SpaceUnits>;
} // namespace space

// Integrals of motion of the gravitating bodies of a state.
//
// Only bodies that are sources of gravity enter the totals. Probes are test
// particles: they are accelerated by the sources but exert no reaction, so
// including them would make momentum and energy non-conserved by
// construction. All sums run in body order (deterministic).
struct ConservedQuantities {
    space::Mass total_mass{};
    space::Energy kinetic_energy{};
    // From IForceModel::potential_energy, so it includes the same softening
    // as the accelerations and kinetic + potential is the conserved
    // Hamiltonian of the force model.
    space::Energy potential_energy{};
    space::Energy total_energy{};
    Vec3<space::Momentum> linear_momentum{};
    // About the origin of the frame.
    Vec3<space::AngularMomentum> angular_momentum{};
    // Zero when the total mass is zero.
    Vec3<space::Length> center_of_mass{};
    Vec3<space::Velocity> center_of_mass_velocity{};
    // Scales used to normalise drifts: sum of m |v| and sum of m |r x v|.
    // They stay meaningful when the totals themselves vanish (for example
    // zero linear momentum in the barycentric frame).
    space::Momentum momentum_scale{};
    space::AngularMomentum angular_momentum_scale{};
};

// Dimensionless drifts of a state with respect to a baseline.
struct ConservationDrift {
    // |E - E0| / |E0|. If E0 is exactly zero, |E - E0| / (K0 + |U0|) is used;
    // if that is also zero the drift is 0 for E == E0 and +infinity otherwise.
    double relative_energy = 0.0;
    // |P - P0| / sum(m |v|) at the baseline (same zero handling).
    double linear_momentum = 0.0;
    // |L - L0| / sum(m |r x v|) at the baseline (same zero handling).
    double angular_momentum = 0.0;
    // |V_cm - V_cm0| / (sum(m |v|) / M) at the baseline (same zero handling).
    double center_of_mass_velocity = 0.0;
};

// Measures the integrals of motion of a state and their drift from a
// baseline captured at construction.
class ConservationDiagnostics {
public:
    // Records the baseline quantities of `initial` under `forces`.
    ConservationDiagnostics(const SimState& initial, const IForceModel& forces);

    [[nodiscard]] static ConservedQuantities measure(const SimState& state,
                                                     const IForceModel& forces);

    [[nodiscard]] const ConservedQuantities& baseline() const noexcept { return baseline_; }

    [[nodiscard]] ConservationDrift drift(const ConservedQuantities& current) const noexcept;
    [[nodiscard]] ConservationDrift drift(const SimState& state, const IForceModel& forces) const;

private:
    ConservedQuantities baseline_;
};

} // namespace orrery::core
