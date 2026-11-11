#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "orrery/core/body_store.h"
#include "orrery/core/force_model.h"
#include "orrery/core/quantity.h"
#include "orrery/core/result.h"
#include "orrery/core/vec3.h"

namespace orrery::core {

struct BarnesHutOptions {
    // Plummer softening length; same kernel as NewtonianGravity. Finite, >= 0.
    space::Length softening{};
    // Opening angle theta in [0, 1]. A node is used as a multipole when
    // theta (d - offset) > side, i.e. d > side / theta + offset (Barnes 1994),
    // with d the distance from the receiver to the node's centre of mass,
    // side the Morton cell side and offset the distance between the cell
    // centre and the centre of mass. theta = 0 never accepts a node, which
    // reduces the model to direct summation.
    double opening_angle = 0.5;
    // Maximum number of sources in a leaf (>= 1); leaves are summed directly.
    std::size_t leaf_capacity = 8;
    // Upper bound on the worker threads used by one evaluation; 0 uses the
    // oneTBB default. Results never depend on it.
    int max_threads = 0;
};

enum class BarnesHutError : std::uint8_t {
    invalid_softening,
    invalid_opening_angle,
    invalid_leaf_capacity,
    invalid_thread_count,
};

[[nodiscard]] std::string_view to_string(BarnesHutError error) noexcept;

// Barnes-Hut tree gravity (Barnes and Hut 1986) with monopole and quadrupole
// moments and Plummer softening, in Gauss-constant units like
// NewtonianGravity.
//
// Each evaluation builds an Octree over the sources (bodies that exert gravity
// and have a positive mass), then computes the acceleration of every body
// independently by a stackless pre-order walk of the tree: an accepted node
// contributes its softened monopole and quadrupole field, a leaf that is not
// accepted is summed directly in sorted order (skipping the receiver itself),
// and any other node is opened. The receivers are distributed over threads
// with oneTBB parallel_for, but no floating-point value is ever combined
// across receivers or threads, so the result is bit-identical for any thread
// count and any partitioning. The tree itself is deterministic (see Octree).
//
// Probes and zero-mass bodies are receivers only, as in NewtonianGravity.
//
// The softened multipole expansion about the centre of mass c, with
// r = x - c, D^2 = |r|^2 + eps^2, M the total G m and Q_ij the G-scaled
// second moments, is the exact Taylor expansion of the Plummer kernel
// 1 / D to second order:
//   a   = -M r / D^3 + (3 Q r + 1.5 tr(Q) r) / D^5 - 7.5 (r.Q.r) r / D^7
//   phi = -M / D - 1.5 (r.Q.r) / D^5 + 0.5 tr(Q) / D^3
// (the dipole vanishes about the centre of mass; derivation in
// docs/physics.md, section 3a).
//
// potential_energy() evaluates U = 1/2 sum_i m_i phi_i with each phi_i from
// the same tree walk, reduced in body order; it is consistent with the
// accelerations to the accuracy of the expansion, not exactly.
//
// APPROXIMATION: Barnes-Hut multipole truncation (A15) - far groups of
// sources are replaced by their monopole and quadrupole - the relative
// acceleration error is controlled by theta (measured median 1e-3 or below at
// theta = 0.5, see docs/physics.md); linear momentum is not exactly conserved
// because the interaction is not pairwise symmetric. Valid for collisionless
// or large-N systems; the direct sum (NewtonianGravity) remains the reference
// for few-body and planetary dynamics.
class BarnesHutGravity final : public IForceModel {
public:
    [[nodiscard]] static Result<BarnesHutGravity, BarnesHutError>
    create(const BarnesHutOptions& options);

    [[nodiscard]] std::string_view name() const noexcept override { return "barnes_hut"; }

    [[nodiscard]] const BarnesHutOptions& options() const noexcept { return options_; }

    void accelerations(const BodyStore& bodies, std::span<const Vec3<space::Length>> positions,
                       std::span<Vec3<space::Acceleration>> out) const override;

    [[nodiscard]] space::Energy
    potential_energy(const BodyStore& bodies,
                     std::span<const Vec3<space::Length>> positions) const override;

private:
    explicit BarnesHutGravity(const BarnesHutOptions& options) noexcept : options_(options) {}

    BarnesHutOptions options_;
};

} // namespace orrery::core
