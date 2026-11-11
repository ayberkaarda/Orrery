#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "orrery/core/body.h"
#include "orrery/core/quantity.h"
#include "orrery/core/result.h"
#include "orrery/core/vec3.h"

namespace orrery::core {

// Initial description of a body added to a BodyStore. Space units.
struct BodyInit {
    std::string name;
    BodyKind kind = Planet{};
    space::Mass mass{};
    Vec3<space::Length> position{};
    Vec3<space::Velocity> velocity{};
};

enum class BodyError : std::uint8_t {
    // Mass is NaN or infinite.
    non_finite_mass,
    // Mass is negative, or zero for a kind that requires a positive mass
    // (probe, black hole).
    invalid_mass,
    // A position or velocity component is NaN or infinite.
    non_finite_state,
    // Black hole spin outside [0, 1].
    invalid_spin,
    // All 2^32 ids of this store have been handed out.
    id_space_exhausted,
    // Radius is NaN, infinite or negative.
    invalid_radius,
};

[[nodiscard]] std::string_view to_string(BodyError error) noexcept;

// Structure-of-arrays store of bodies in space units (AU, solar mass, day).
//
// Determinism contract: bodies are kept in insertion order; removal preserves
// the relative order of the remaining bodies. Ids increase monotonically with
// insertion and are never reused, so the id column is always sorted and id
// lookup is a binary search (no hash containers).
class BodyStore {
public:
    BodyStore() = default;
    ~BodyStore() = default;
    BodyStore(const BodyStore&) = default;
    BodyStore(BodyStore&&) noexcept = default;
    // Strong exception guarantee: the columns are copied into a temporary
    // first, so a failed allocation leaves this store unchanged and every
    // column the same length.
    BodyStore& operator=(const BodyStore& other);
    BodyStore& operator=(BodyStore&&) noexcept = default;

    void swap(BodyStore& other) noexcept;
    friend void swap(BodyStore& a, BodyStore& b) noexcept { a.swap(b); }

    [[nodiscard]] Result<BodyId, BodyError> add(BodyInit init);

    // Removes the body with this id. Returns false if no such body exists.
    bool remove(BodyId id);

    [[nodiscard]] std::size_t size() const noexcept { return ids_.size(); }
    [[nodiscard]] bool empty() const noexcept { return ids_.empty(); }

    [[nodiscard]] std::optional<std::size_t> index_of(BodyId id) const noexcept;
    [[nodiscard]] bool contains(BodyId id) const noexcept { return index_of(id).has_value(); }

    // Per-body accessors. The index must be below size(); otherwise
    // std::out_of_range is thrown.
    [[nodiscard]] BodyId id(std::size_t index) const { return ids_.at(index); }
    [[nodiscard]] const std::string& name(std::size_t index) const { return names_.at(index); }
    [[nodiscard]] const BodyKind& kind(std::size_t index) const { return kinds_.at(index); }
    [[nodiscard]] space::Mass mass(std::size_t index) const { return masses_.at(index); }
    [[nodiscard]] Vec3<space::Length> position(std::size_t index) const;
    [[nodiscard]] Vec3<space::Velocity> velocity(std::size_t index) const;
    // False for probes: they carry a mass but are not a source of gravity.
    [[nodiscard]] bool exerts_gravity(std::size_t index) const {
        return core::exerts_gravity(kinds_.at(index));
    }

    // Returns non_finite_state (and leaves the body unchanged) for NaN or
    // infinite components.
    [[nodiscard]] Result<BodyId, BodyError> set_position(std::size_t index,
                                                         const Vec3<space::Length>& position);
    [[nodiscard]] Result<BodyId, BodyError> set_velocity(std::size_t index,
                                                         const Vec3<space::Velocity>& velocity);

    // Physical radius used by collision detection and the tidal-disruption
    // test (Phase 3b). Every body starts as a point (radius 0); a point never
    // collides with another point and is never tidally disrupted.
    [[nodiscard]] space::Length radius(std::size_t index) const { return radii_.at(index); }
    // Returns invalid_radius (body unchanged) for a NaN, infinite or negative radius.
    [[nodiscard]] Result<BodyId, BodyError> set_radius(std::size_t index, space::Length radius);
    // Same validation as add() for the body's kind; the body is unchanged on failure.
    [[nodiscard]] Result<BodyId, BodyError> set_mass(std::size_t index, space::Mass mass);

    // Contiguous columns, in body order.
    [[nodiscard]] std::span<const BodyId> ids() const noexcept { return ids_; }
    [[nodiscard]] std::span<const space::Mass> masses() const noexcept { return masses_; }
    [[nodiscard]] std::span<const space::Length> position_x() const noexcept { return px_; }
    [[nodiscard]] std::span<const space::Length> position_y() const noexcept { return py_; }
    [[nodiscard]] std::span<const space::Length> position_z() const noexcept { return pz_; }
    [[nodiscard]] std::span<const space::Velocity> velocity_x() const noexcept { return vx_; }
    [[nodiscard]] std::span<const space::Velocity> velocity_y() const noexcept { return vy_; }
    [[nodiscard]] std::span<const space::Velocity> velocity_z() const noexcept { return vz_; }
    [[nodiscard]] std::span<const space::Length> radii() const noexcept { return radii_; }

private:
    std::uint32_t next_id_ = 0;
    bool ids_exhausted_ = false;
    std::vector<BodyId> ids_;
    std::vector<std::string> names_;
    std::vector<BodyKind> kinds_;
    std::vector<space::Mass> masses_;
    std::vector<space::Length> px_;
    std::vector<space::Length> py_;
    std::vector<space::Length> pz_;
    std::vector<space::Velocity> vx_;
    std::vector<space::Velocity> vy_;
    std::vector<space::Velocity> vz_;
    std::vector<space::Length> radii_;
};

} // namespace orrery::core
