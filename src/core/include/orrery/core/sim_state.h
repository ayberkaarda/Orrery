#pragma once

#include <utility>

#include "orrery/core/body_store.h"
#include "orrery/core/quantity.h"

namespace orrery::core {

// Complete physical state of a space simulation at one instant.
// All values are in space units (AU, solar mass, day); see ADR-001.
class SimState {
public:
    using unit_system = SpaceUnits;

    SimState() = default;
    SimState(space::Time time, BodyStore bodies) : time_(time), bodies_(std::move(bodies)) {}
    ~SimState() = default;
    SimState(const SimState&) = default;
    SimState(SimState&&) noexcept = default;
    // Strong exception guarantee: copy first, then commit with a noexcept move.
    SimState& operator=(const SimState& other) {
        SimState copy(other);
        *this = std::move(copy);
        return *this;
    }
    SimState& operator=(SimState&&) noexcept = default;

    // Unit system of every quantity held by this state.
    [[nodiscard]] static constexpr UnitSystemId units() noexcept { return unit_system::id; }

    // Simulation time, days since the scenario epoch.
    [[nodiscard]] space::Time time() const noexcept { return time_; }
    void set_time(space::Time time) noexcept { time_ = time; }

    [[nodiscard]] const BodyStore& bodies() const noexcept { return bodies_; }
    [[nodiscard]] BodyStore& bodies() noexcept { return bodies_; }

private:
    space::Time time_{};
    BodyStore bodies_;
};

} // namespace orrery::core
