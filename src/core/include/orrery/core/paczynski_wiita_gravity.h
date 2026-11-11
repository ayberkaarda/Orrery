#pragma once

#include <span>
#include <string_view>

#include "orrery/core/body_store.h"
#include "orrery/core/force_model.h"
#include "orrery/core/quantity.h"
#include "orrery/core/result.h"
#include "orrery/core/vec3.h"

namespace orrery::core {

// Pseudo-Newtonian gravity with black holes (ADR-004), direct O(N^2)
// summation in Gauss-constant units (G = k^2 AU^3 Msun^-1 day^-2).
//
// Every pair of sources interacts through a central pair potential
//   U_ij = -G m_i m_j / (r - R_ij),
// where R_ij is the sum of the Schwarzschild radii r_s = 2 G m / c^2 of the
// black holes of the pair (0 if neither is a black hole, so the pair is
// Newtonian). For a black hole of mass M and any other body this is the
// Paczynski-Wiita potential Phi = -G M / (r - r_s); for two black holes it is
// the Paczynski-Wiita potential of the total mass, since r_s is linear in
// mass. The pair force is equal and opposite and central, so linear and
// angular momentum are conserved exactly by the force law, and
// potential_energy() is the matching Hamiltonian term.
//
// Softening policy: Plummer softening eps applies to Newtonian pairs only,
// exactly as in NewtonianGravity (the ordinary path, the power-of-two scaled
// path, their selection and the order of operations are the same, so a system
// without black holes gives accelerations and potential bit-identical to
// NewtonianGravity for every separation and mass).
// Black-hole pairs are never softened: their regularisation is the horizon
// floor below.
//
// Horizon floor (defined behaviour at and inside the horizon): let
// x = r - R_ij and h = horizon_floor_fraction * R_ij. For x >= h the force
// is the Paczynski-Wiita force G m_i m_j / x^2. For x < h, including every
// r <= R_ij (inside the horizon), the force magnitude is held at its value at
// x = h, G m_i m_j / h^2, still directed along the separation, and the pair
// potential continues linearly, U = -G m_i m_j (2h - x) / h^2, so it stays the
// exact potential of the force and is finite everywhere. At r = 0 the
// direction is undefined and the pair force is zero. No NaN is produced for
// any finite separation. A body there is absorbed by the encounter resolver
// at the end of the step (HorizonCrossing); the floor only keeps the force
// evaluations of the step itself finite.
//
// APPROXIMATION: horizon floor - the Paczynski-Wiita force diverges at
// r = r_s, so it is capped at its value at r = (1 + 1e-3) R_ij - the dynamics
// are exact Paczynski-Wiita for r >= (1 + 1e-3) R_ij; inside that shell the
// body is about to be absorbed and its trajectory there carries no meaning.
//
// Probes are test particles, as in NewtonianGravity: they feel every source
// (including the Paczynski-Wiita field of black holes) but exert no force.
//
// Determinism: the same pair order as NewtonianGravity (i = 0..N-1,
// j = i+1..N-1), so each acceleration accumulates in ascending source index.
//
// Range: black-hole pairs never form r^2 or (r - R_ij)^2 r. The separation is
// scaled by a power of two to obtain r and the unit direction, and the field
// is (G m / g^2) u 2^(-2k) with r - R_ij (or the floor) = g 2^k, g in
// [0.5, 1). Forces and potentials are therefore accurate to rounding for
// every finite separation and for black-hole masses within [1e-30, 1e30] Msun
// (where G M and the floor gap stay normal doubles); a field below the
// smallest normal double is rounded as a subnormal. Newtonian pairs share the
// range of NewtonianGravity (accurate for every representable separation).
class PaczynskiWiitaGravity final : public IForceModel {
public:
    // Fraction of R_ij below which the force is held constant (see above).
    static constexpr double horizon_floor_fraction = 1.0e-3;

    // Unsoftened Newtonian pairs.
    PaczynskiWiitaGravity() = default;

    // eps must be finite and non-negative; it applies to Newtonian pairs only.
    [[nodiscard]] static Result<PaczynskiWiitaGravity, GravityError>
    create(space::Length softening);

    [[nodiscard]] std::string_view name() const noexcept override { return "paczynski_wiita"; }

    [[nodiscard]] space::Length softening() const noexcept { return softening_; }

    void accelerations(const BodyStore& bodies, std::span<const Vec3<space::Length>> positions,
                       std::span<Vec3<space::Acceleration>> out) const override;

    [[nodiscard]] space::Energy
    potential_energy(const BodyStore& bodies,
                     std::span<const Vec3<space::Length>> positions) const override;

private:
    explicit PaczynskiWiitaGravity(space::Length softening) noexcept : softening_(softening) {}

    space::Length softening_{};
};

} // namespace orrery::core
