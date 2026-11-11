#pragma once

#include <span>
#include <vector>

#include "orrery/core/event.h"
#include "orrery/core/force_model.h"
#include "orrery/core/integrator.h"
#include "orrery/core/quantity.h"
#include "orrery/core/result.h"
#include "orrery/core/sim_state.h"
#include "orrery/core/vec3.h"

namespace orrery::core {

// Which encounter phases run after each step.
struct EncounterOptions {
    bool tidal_disruption = true;
    bool horizon_absorption = true;
    bool collisions = true;
};

// Encounter phase of a step: given the positions at the start of the step
// and the state after it, runs (in this fixed order)
//   1. detect_tidal_disruptions  (tidal.h)  - Disruption events, no state change
//   2. absorb_horizon_crossings  (horizon.h) - HorizonCrossing events
//   3. resolve_collisions        (collision.h) - Collision events
// and appends the events to `log` in that order. Within each phase the order
// is documented by the phase. The result depends only on the inputs:
// deterministic, single threaded, no callbacks.
//
// `start_positions` must hold one position per body of `state` (same order);
// otherwise std::invalid_argument is thrown and nothing changes.
void resolve_encounters(SimState& state, std::span<const Vec3<space::Length>> start_positions,
                        space::Time t_start, space::Time dt, const EncounterOptions& options,
                        EventLog& log);

// True when any enabled phase of resolve_encounters would emit an event for
// bodies moving from `start` to `end` (one position each per body of
// `bodies`, same order). Changes nothing.
[[nodiscard]] bool encounter_pending(const BodyStore& bodies,
                                     std::span<const Vec3<space::Length>> start,
                                     std::span<const Vec3<space::Length>> end,
                                     const EncounterOptions& options);

// Step loop driver: integrates dt and runs the encounter phase on every
// accepted integration segment in which an encounter occurs.
//
//   EventStepper stepper;
//   EventLog log;
//   stepper.step(integrator, state, dt, forces, log);
//   for (const Event& e : log.drain()) { ... }
//
// The integrator is driven through IIntegrator::step_observed. For a
// fixed-step integrator the only segment is the whole step, as before. An
// adaptive integrator (IAS15) reports each accepted internal step; the first
// one in which encounter_pending holds ends the integrator call, the
// encounter phase runs on that internal step (t_start and dt of the internal
// step), and integration resumes from the resulting state for the rest of
// dt (IAS15 restarts its history when bodies were merged or removed). The
// chord of the linear swept motion is thus never longer than one accepted
// internal step, whose length the IAS15 error control bounds by the local
// dynamical time - a pericentre passage through a body is seen however long
// the requested dt is, and the events do not depend on how a run is split
// into output intervals beyond the internal steps those split points shorten.
//
// On success the state's time is t_start + dt. If an integrator call fails,
// the state is restored to the state before step() and no event is
// appended. Not thread-safe.
class EventStepper {
public:
    EventStepper() = default;
    explicit EventStepper(EncounterOptions options) : options_(options) {}

    [[nodiscard]] const EncounterOptions& options() const noexcept { return options_; }

    [[nodiscard]] Result<space::Time, IntegrationError> step(IIntegrator& integrator,
                                                             SimState& state, space::Time dt,
                                                             const IForceModel& forces,
                                                             EventLog& log);

private:
    EncounterOptions options_;
};

} // namespace orrery::core
