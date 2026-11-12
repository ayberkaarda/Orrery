#include "orrery/core/sim_snapshot.h"

#include <algorithm>
#include <iterator>

namespace orrery::core {

std::shared_ptr<const SimSnapshot> SimSnapshot::capture(const SimState& state,
                                                        std::uint64_t sequence) {
    const BodyStore& bodies = state.bodies();
    const std::size_t count = bodies.size();
    // The constructor is private, so make_shared cannot reach it.
    std::shared_ptr<SimSnapshot> snapshot(new SimSnapshot());
    snapshot->time_ = state.time();
    snapshot->sequence_ = sequence;
    snapshot->ids_.assign(bodies.ids().begin(), bodies.ids().end());
    snapshot->masses_.assign(bodies.masses().begin(), bodies.masses().end());
    snapshot->radii_.assign(bodies.radii().begin(), bodies.radii().end());
    snapshot->names_.reserve(count);
    snapshot->kinds_.reserve(count);
    snapshot->positions_.reserve(count);
    snapshot->velocities_.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        snapshot->names_.push_back(bodies.name(i));
        snapshot->kinds_.push_back(bodies.kind(i));
        snapshot->positions_.push_back(
            {bodies.position_x()[i], bodies.position_y()[i], bodies.position_z()[i]});
        snapshot->velocities_.push_back(
            {bodies.velocity_x()[i], bodies.velocity_y()[i], bodies.velocity_z()[i]});
    }
    return snapshot;
}

std::optional<std::size_t> SimSnapshot::index_of(BodyId id) const noexcept {
    const auto it = std::lower_bound(ids_.begin(), ids_.end(), id);
    if (it == ids_.end() || *it != id) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(std::distance(ids_.begin(), it));
}

} // namespace orrery::core
