#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include "orrery/core/body_store.h"
#include "orrery/core/event.h"
#include "orrery/core/quantity.h"
#include "orrery/core/vec3.h"

namespace orrery::core {

// Horizon phase of a step: absorbs every body that crosses the horizon of a
// black hole and appends one HorizonCrossingEvent per absorption.
//
// A body is absorbed when the centre of its swept path relative to a black
// hole (linear relative motion, see collision.h) comes within r_s of the
// black hole; for two black holes the threshold is r_s1 + r_s2 (the r_s of
// the total mass). Every kind is absorbed, probes included (a probe's mass
// and momentum are added to the black hole, so totals over all bodies are
// conserved). A black hole always survives against another kind; between
// two black holes the heavier survives (the lower id on equal masses). The
// merge is merge_bodies(..., RadiusRule::keep_survivor): mass, linear
// momentum and centre of mass are conserved to rounding; the relative
// angular momentum is reported in the event. The black hole's spin is left
// unchanged (ADR-003: spin is stored only).
//
// Order: the earliest crossing is resolved first, ties in ascending (lower
// id, higher id). After each absorption the surviving hole's start position
// becomes the start of the merged centre-of-mass path
// (merged_start_position), its pending crossings are dropped, and it is swept
// again - new position, velocity and Schwarzschild radius - against every
// other body over the remaining fraction [tau, 1] of the step; this repeats
// until no crossing is left. A hole may therefore absorb several bodies in
// one step, including bodies that reach only the horizon enlarged by an
// earlier absorption or merger. `start_positions` holds the position of each
// body at t_start (same order as `bodies`); entries of absorbed bodies are
// erased. Returns the number of absorptions.
//
// APPROXIMATION: absorption at the horizon of the point mass - a body is
// swallowed when its centre crosses r_s; its finite size, the plunge from
// the ISCO and the radiated energy are not modelled - valid for the
// bookkeeping of mass and momentum, not for the accretion luminosity.
std::size_t absorb_horizon_crossings(BodyStore& bodies,
                                     std::vector<Vec3<space::Length>>& start_positions,
                                     space::Time t_start, space::Time dt, EventLog& log);

// Earliest crossing step fraction of the horizon phase for bodies moving from
// `start_positions` to `end_positions` (one each per body of `bodies`, same
// order), without changing anything; empty if absorb_horizon_crossings would
// absorb nothing. Throws std::invalid_argument on a size mismatch.
[[nodiscard]] std::optional<double>
earliest_horizon_fraction(const BodyStore& bodies,
                          std::span<const Vec3<space::Length>> start_positions,
                          std::span<const Vec3<space::Length>> end_positions);

} // namespace orrery::core
