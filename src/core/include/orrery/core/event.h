#pragma once

#include <cstddef>
#include <span>
#include <utility>
#include <variant>
#include <vector>

#include "orrery/core/body.h"
#include "orrery/core/conservation.h"
#include "orrery/core/quantity.h"
#include "orrery/core/vec3.h"

namespace orrery::core {

// Typed simulation events of the encounter phase (Phase 3b). Events are plain
// values: they are collected into an EventLog in a fixed, documented order
// (see EncounterResolver) and never trigger callbacks.
//
// Every event carries the estimated instant of the encounter,
//   time = t_start + step_fraction * dt,
// with step_fraction in [0, 1] taken from the linear swept motion of the step
// (0 when the bodies already overlapped at the start of the step). Under
// EventStepper the step is the integration segment in which the encounter
// was found (one accepted IAS15 internal step, or the whole fixed step), so
// t_start and dt are those of that segment.

// Two bodies touched and merged inelastically. The survivor keeps its id,
// name and kind (the heavier body; the lower id on equal masses); the
// absorbed body is removed from the store.
struct CollisionEvent {
    space::Time time{};
    double step_fraction = 0.0;
    BodyId survivor;
    BodyId absorbed;
    space::Mass survivor_mass_before{};
    space::Mass absorbed_mass{};
    // Merged centre of mass (end of step), where the survivor now is.
    Vec3<space::Length> position{};
    // |v_absorbed - v_survivor| at the end of the step.
    space::Velocity relative_speed{};
    // Kinetic energy of the relative motion, mu v_rel^2 / 2, removed by the
    // inelastic merge.
    space::Energy dissipated_energy{};
    // Angular momentum of the relative motion about the merged centre of
    // mass, mu (r_rel x v_rel). A point mass cannot carry it, so it leaves the
    // orbital total; orbital L after + this = orbital L before (see
    // docs/physics.md, 3b).
    Vec3<space::AngularMomentum> internal_angular_momentum{};

    friend bool operator==(const CollisionEvent&, const CollisionEvent&) = default;
};

// A body with a radius entered the tidal radius r_t = R (M/m)^(1/3) of a
// black hole during the step (outside r_t at the start of the step). Only
// detection: the body is not fragmented or otherwise modified.
struct DisruptionEvent {
    space::Time time{};
    double step_fraction = 0.0;
    BodyId body;
    BodyId black_hole;
    space::Length tidal_radius{};
    // Smallest separation reached on the swept path of the step.
    space::Length closest_approach{};

    friend bool operator==(const DisruptionEvent&, const DisruptionEvent&) = default;
};

// A body crossed the horizon of a black hole and was absorbed: the black hole
// takes its mass and momentum and moves to the merged centre of mass. For two
// black holes the heavier one survives (the lower id on equal masses).
struct HorizonCrossingEvent {
    space::Time time{};
    double step_fraction = 0.0;
    BodyId black_hole;
    BodyId absorbed;
    space::Mass black_hole_mass_before{};
    space::Mass absorbed_mass{};
    // Absorption threshold: r_s of the black hole, or the sum of both
    // Schwarzschild radii for two black holes.
    space::Length horizon_radius{};
    // Schwarzschild radius after the absorption.
    space::Length schwarzschild_radius_after{};
    Vec3<space::Length> position{};
    // Same meaning as CollisionEvent::internal_angular_momentum.
    Vec3<space::AngularMomentum> internal_angular_momentum{};

    friend bool operator==(const HorizonCrossingEvent&, const HorizonCrossingEvent&) = default;
};

using Event = std::variant<CollisionEvent, DisruptionEvent, HorizonCrossingEvent>;

// Ordered, append-only list of events that the step loop drains. Single
// threaded; the order of the entries is the order in which they were
// appended, which the encounter resolver keeps deterministic.
class EventLog {
public:
    void append(const Event& event) { events_.push_back(event); }

    [[nodiscard]] std::span<const Event> events() const noexcept { return events_; }
    [[nodiscard]] std::size_t size() const noexcept { return events_.size(); }
    [[nodiscard]] bool empty() const noexcept { return events_.empty(); }

    // Moves every event out (in order) and leaves the log empty.
    [[nodiscard]] std::vector<Event> drain() noexcept {
        std::vector<Event> out;
        out.swap(events_);
        return out;
    }

    void clear() noexcept { events_.clear(); }

private:
    std::vector<Event> events_;
};

} // namespace orrery::core
