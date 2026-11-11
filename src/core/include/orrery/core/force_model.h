#pragma once

#include <cstdint>
#include <span>
#include <string_view>

#include "orrery/core/body_store.h"
#include "orrery/core/quantity.h"
#include "orrery/core/result.h"
#include "orrery/core/vec3.h"

namespace orrery::core {

// Source of accelerations for the space integrators (space units throughout).
//
// A force model is a pure function of the masses and kinds held by a
// BodyStore and of a set of trial positions. Positions are passed separately
// so that integrators can evaluate accelerations at intermediate stages
// without writing them into the store. Implementations must be deterministic:
// identical inputs give bit-identical outputs (fixed summation order, no
// unordered containers, no threading whose reduction order depends on
// scheduling).
class IForceModel {
public:
    virtual ~IForceModel() = default;

    // Short identifier of the model, e.g. "newtonian".
    [[nodiscard]] virtual std::string_view name() const noexcept = 0;

    // Writes the acceleration of every body of `bodies`, evaluated at
    // `positions`, into `out` (body order). positions.size() and out.size()
    // must equal bodies.size(); otherwise std::invalid_argument is thrown and
    // `out` is left unchanged. Coincident sources without softening give
    // non-finite accelerations; callers detect them (see IIntegrator).
    virtual void accelerations(const BodyStore& bodies,
                               std::span<const Vec3<space::Length>> positions,
                               std::span<Vec3<space::Acceleration>> out) const = 0;

    // Potential energy of the bodies at `positions`, consistent with
    // accelerations(): for every body that is a source of gravity, the
    // acceleration equals -grad(U) / m. Test particles (probes) neither
    // contribute nor appear in U. Same size contract as accelerations().
    [[nodiscard]] virtual space::Energy
    potential_energy(const BodyStore& bodies,
                     std::span<const Vec3<space::Length>> positions) const = 0;

protected:
    IForceModel() = default;
    IForceModel(const IForceModel&) = default;
    IForceModel(IForceModel&&) = default;
    IForceModel& operator=(const IForceModel&) = default;
    IForceModel& operator=(IForceModel&&) = default;
};

enum class GravityError : std::uint8_t {
    // Softening length is NaN, infinite or negative.
    invalid_softening,
};

[[nodiscard]] std::string_view to_string(GravityError error) noexcept;

// Newtonian point-mass gravity by direct O(N^2) summation with Plummer
// softening, in Gauss-constant units: G = k^2 AU^3 Msun^-1 day^-2, so
// G * M is in AU^3 day^-2 for M in solar masses.
//
//   a_i = sum_{j != i, j source} G m_j (r_j - r_i) / (|r_j - r_i|^2 + eps^2)^(3/2)
//   U   = - sum_{i < j, both sources} G m_i m_j / (|r_j - r_i|^2 + eps^2)^(1/2)
//
// This is the reference force model. Its contract is N <= 256 bodies: the
// cost grows as N^2 and larger systems are the domain of a tree code. It
// remains correct above that size, only slower.
//
// Determinism: pairs are visited in the fixed order i = 0..N-1, j = i+1..N-1
// (body order of the store), each acceleration therefore accumulates its
// contributions in ascending source index, and the potential sum is reduced in
// the same pair order. No other order is ever used.
//
// Probes are test particles: they feel the field of every source but exert no
// force (BodyStore::exerts_gravity is false for them).
//
// APPROXIMATION: Newtonian point masses - bodies are treated as spherically
// symmetric points in flat space-time (no general relativity, oblateness,
// tides or radiation forces) - valid for planetary dynamics where the leading
// neglected terms are relativistic (Mercury's perihelion advances by 43
// arcsec per century, a relative force correction of order 3e-8) and for
// separations far above the bodies' radii.
class NewtonianGravity final : public IForceModel {
public:
    // Exact Newtonian gravity (eps = 0).
    NewtonianGravity() = default;

    // Plummer-softened gravity. eps must be finite and non-negative; eps = 0
    // is exact Newtonian gravity.
    [[nodiscard]] static Result<NewtonianGravity, GravityError> create(space::Length softening);

    [[nodiscard]] std::string_view name() const noexcept override { return "newtonian"; }

    [[nodiscard]] space::Length softening() const noexcept { return softening_; }

    void accelerations(const BodyStore& bodies, std::span<const Vec3<space::Length>> positions,
                       std::span<Vec3<space::Acceleration>> out) const override;

    [[nodiscard]] space::Energy
    potential_energy(const BodyStore& bodies,
                     std::span<const Vec3<space::Length>> positions) const override;

private:
    explicit NewtonianGravity(space::Length softening) noexcept : softening_(softening) {}

    space::Length softening_{};
};

} // namespace orrery::core
