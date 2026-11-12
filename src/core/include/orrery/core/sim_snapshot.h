#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "orrery/core/body.h"
#include "orrery/core/quantity.h"
#include "orrery/core/sim_state.h"
#include "orrery/core/vec3.h"

namespace orrery::core {

// Immutable copy of the observable state of a space simulation at one
// instant: what a renderer or user interface may read (architecture rule:
// render reads physics state only through std::shared_ptr<const SimSnapshot>).
// Space units (AU, solar mass, day; ADR-001), body order and ids as in the
// BodyStore it was captured from, so ids() is sorted ascending.
//
// A snapshot owns all of its data and has no mutating member functions;
// shared between threads as shared_ptr<const SimSnapshot> it is safe to read
// concurrently. capture() is the only way to build one: copying, moving and
// assignment are deleted, so no caller can hold a writable SimSnapshot behind
// a published const pointer; share a snapshot by copying the shared_ptr.
// Capturing reads the state once and must not race with writes to it (the
// caller keeps the source SimState stable during capture). Spans returned by
// the accessors stay valid as long as the snapshot itself is alive, so a
// reader keeps its shared_ptr while it uses them. How snapshots are handed
// from the physics thread to the render thread (triple buffer) is not part of
// this type.
class SimSnapshot {
public:
    SimSnapshot(const SimSnapshot&) = delete;
    SimSnapshot(SimSnapshot&&) = delete;
    SimSnapshot& operator=(const SimSnapshot&) = delete;
    SimSnapshot& operator=(SimSnapshot&&) = delete;
    ~SimSnapshot() = default;

    // Copies time, ids, names, kinds, masses, radii, positions and velocities
    // of every body. `sequence` is a caller-defined counter (for example the
    // physics step number) that lets readers detect a new snapshot.
    // Throws std::bad_alloc if the copy cannot be allocated.
    [[nodiscard]] static std::shared_ptr<const SimSnapshot> capture(const SimState& state,
                                                                    std::uint64_t sequence = 0);

    [[nodiscard]] space::Time time() const noexcept { return time_; }
    [[nodiscard]] std::uint64_t sequence() const noexcept { return sequence_; }

    [[nodiscard]] std::size_t size() const noexcept { return ids_.size(); }
    [[nodiscard]] bool empty() const noexcept { return ids_.empty(); }

    // Binary search over the sorted id column.
    [[nodiscard]] std::optional<std::size_t> index_of(BodyId id) const noexcept;

    // Columns, in body order; every column has size() elements.
    [[nodiscard]] std::span<const BodyId> ids() const noexcept { return ids_; }
    [[nodiscard]] std::span<const std::string> names() const noexcept { return names_; }
    [[nodiscard]] std::span<const BodyKind> kinds() const noexcept { return kinds_; }
    [[nodiscard]] std::span<const space::Mass> masses() const noexcept { return masses_; }
    // Physical radius (0 for a point body, see BodyStore::radius).
    [[nodiscard]] std::span<const space::Length> radii() const noexcept { return radii_; }
    [[nodiscard]] std::span<const Vec3<space::Length>> positions() const noexcept {
        return positions_;
    }
    [[nodiscard]] std::span<const Vec3<space::Velocity>> velocities() const noexcept {
        return velocities_;
    }

    [[nodiscard]] BodyKindTag kind_tag(std::size_t index) const { return tag_of(kinds_.at(index)); }

private:
    SimSnapshot() = default;

    space::Time time_{};
    std::uint64_t sequence_ = 0;
    std::vector<BodyId> ids_;
    std::vector<std::string> names_;
    std::vector<BodyKind> kinds_;
    std::vector<space::Mass> masses_;
    std::vector<space::Length> radii_;
    std::vector<Vec3<space::Length>> positions_;
    std::vector<Vec3<space::Velocity>> velocities_;
};

} // namespace orrery::core
