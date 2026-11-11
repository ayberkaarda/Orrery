#pragma once

#include <optional>
#include <span>

#include "orrery/core/body_store.h"
#include "orrery/core/event.h"
#include "orrery/core/quantity.h"
#include "orrery/core/vec3.h"

namespace orrery::core {

// Tidal phase of a step (detection only): appends a DisruptionEvent for
// every body that enters the tidal radius r_t = R (M / m)^(1/3) of a black
// hole during the step.
//
// Candidates are bodies with a positive radius and mass that are neither
// black holes nor probes. A body is reported when it is outside r_t at the
// start of the step and its swept path (linear relative motion, see
// collision.h) comes within r_t. Black holes with r_t <= r_s swallow the
// body whole and report no disruption. The body is not modified: debris
// physics is outside this phase. Events are appended in ascending
// (step_fraction, body id, black hole id). `start_positions` holds the
// position of each body at t_start (same order as `bodies`).
void detect_tidal_disruptions(const BodyStore& bodies,
                              std::span<const Vec3<space::Length>> start_positions,
                              space::Time t_start, space::Time dt, EventLog& log);

// Earliest tidal-entry step fraction for bodies moving from `start_positions`
// to `end_positions` (one each per body of `bodies`, same order); empty if
// detect_tidal_disruptions would report nothing. Throws std::invalid_argument
// on a size mismatch.
[[nodiscard]] std::optional<double>
earliest_tidal_fraction(const BodyStore& bodies,
                        std::span<const Vec3<space::Length>> start_positions,
                        std::span<const Vec3<space::Length>> end_positions);

} // namespace orrery::core
