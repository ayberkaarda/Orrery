#include "orrery/core/event_step.h"

#include <optional>
#include <stdexcept>
#include <utility>

#include "orrery/core/collision.h"
#include "orrery/core/horizon.h"
#include "orrery/core/profile.h"
#include "orrery/core/tidal.h"

namespace orrery::core {

namespace {

// Stops the integrator at the first accepted segment in which an encounter
// phase would act, and remembers that segment.
class EncounterWatch final : public IStepObserver {
public:
    EncounterWatch(const EncounterOptions& options, SimState& state,
                   std::optional<SimState>& snapshot, space::Time requested)
        : options_(&options), state_(&state), snapshot_(&snapshot), requested_(requested) {}

    bool on_segment(const BodyStore& bodies, std::span<const Vec3<space::Length>> start,
                    std::span<const Vec3<space::Length>> end, space::Time begin,
                    space::Time end_offset) override {
        if (!encounter_pending(bodies, start, end, *options_)) {
            return false;
        }
        stopped_ = true;
        start_.assign(start.begin(), start.end());
        begin_ = begin;
        end_ = end_offset;
        // A stop before the end of the request: the integrator has not
        // committed anything yet, so this is the state before the call.
        if (end_offset != requested_ && !snapshot_->has_value()) {
            snapshot_->emplace(*state_);
        }
        return true;
    }

    [[nodiscard]] bool stopped() const noexcept { return stopped_; }
    [[nodiscard]] std::span<const Vec3<space::Length>> start() const noexcept { return start_; }
    [[nodiscard]] space::Time begin() const noexcept { return begin_; }
    [[nodiscard]] space::Time end() const noexcept { return end_; }

private:
    const EncounterOptions* options_;
    SimState* state_;
    std::optional<SimState>* snapshot_;
    space::Time requested_;
    bool stopped_ = false;
    std::vector<Vec3<space::Length>> start_;
    space::Time begin_{};
    space::Time end_{};
};

} // namespace

bool encounter_pending(const BodyStore& bodies, std::span<const Vec3<space::Length>> start,
                       std::span<const Vec3<space::Length>> end, const EncounterOptions& options) {
    return (options.tidal_disruption && earliest_tidal_fraction(bodies, start, end)) ||
           (options.horizon_absorption && earliest_horizon_fraction(bodies, start, end)) ||
           (options.collisions && earliest_collision_fraction(bodies, start, end));
}

void resolve_encounters(SimState& state, std::span<const Vec3<space::Length>> start_positions,
                        space::Time t_start, space::Time dt, const EncounterOptions& options,
                        EventLog& log) {
    BodyStore& bodies = state.bodies();
    if (start_positions.size() != bodies.size()) {
        throw std::invalid_argument("resolve_encounters: one start position per body is required");
    }
    std::vector<Vec3<space::Length>> start(start_positions.begin(), start_positions.end());
    if (options.tidal_disruption) {
        detect_tidal_disruptions(bodies, start, t_start, dt, log);
    }
    if (options.horizon_absorption) {
        static_cast<void>(absorb_horizon_crossings(bodies, start, t_start, dt, log));
    }
    if (options.collisions) {
        static_cast<void>(resolve_collisions(bodies, start, t_start, dt, log));
    }
}

Result<space::Time, IntegrationError> EventStepper::step(IIntegrator& integrator, SimState& state,
                                                         space::Time dt, const IForceModel& forces,
                                                         EventLog& log) {
    ORRERY_ZONE("physics_step");
    const space::Time t_start = state.time();
    // State before the call, taken only when a stop leaves a remainder: a
    // later integrator failure restores it (strong guarantee).
    std::optional<SimState> snapshot;
    EventLog pending;
    space::Time remaining = dt;
    while (true) {
        const space::Time call_start = state.time();
        EncounterWatch watch(options_, state, snapshot, remaining);
        auto result = integrator.step_observed(state, remaining, forces, watch);
        if (!result) {
            if (snapshot) {
                state = std::move(*snapshot);
            }
            return result;
        }
        if (!watch.stopped()) {
            break;
        }
        resolve_encounters(state, watch.start(), call_start + watch.begin(),
                           watch.end() - watch.begin(), options_, pending);
        if (watch.end() == remaining) {
            break;
        }
        remaining = remaining - watch.end();
    }
    state.set_time(t_start + dt);
    for (Event& e : pending.drain()) {
        log.append(e);
    }
    return state.time();
}

} // namespace orrery::core
