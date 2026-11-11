#include "orrery/core/body_store.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <utility>

namespace orrery::core {

namespace {

bool requires_positive_mass(const BodyKind& kind) noexcept {
    return std::holds_alternative<Probe>(kind) || std::holds_alternative<BlackHole>(kind);
}

std::optional<BodyError> validate_kind(const BodyKind& kind) noexcept {
    if (const auto* hole = std::get_if<BlackHole>(&kind)) {
        if (!(hole->spin >= 0.0 && hole->spin <= 1.0)) {
            return BodyError::invalid_spin;
        }
    }
    return std::nullopt;
}

std::optional<BodyError> validate_mass(const BodyKind& kind, space::Mass mass) noexcept {
    if (!isfinite(mass)) {
        return BodyError::non_finite_mass;
    }
    if (mass.value() < 0.0 || (requires_positive_mass(kind) && mass.value() == 0.0)) {
        return BodyError::invalid_mass;
    }
    return std::nullopt;
}

// Geometric growth, so repeated insertion stays amortised O(1).
template <typename T> void grow_for(std::vector<T>& column, std::size_t new_size) {
    if (column.capacity() < new_size) {
        column.reserve(std::max(new_size, 2 * column.capacity()));
    }
}

template <typename T> void erase_at(std::vector<T>& column, std::size_t index) {
    column.erase(std::next(column.begin(), static_cast<std::ptrdiff_t>(index)));
}

} // namespace

std::string_view to_string(BodyError error) noexcept {
    switch (error) {
        case BodyError::non_finite_mass:
            return "mass is not finite";
        case BodyError::invalid_mass:
            return "mass is negative, or zero for a kind that requires a positive mass";
        case BodyError::non_finite_state:
            return "position or velocity is not finite";
        case BodyError::invalid_spin:
            return "black hole spin is outside [0, 1]";
        case BodyError::id_space_exhausted:
            return "body id space exhausted";
        case BodyError::invalid_radius:
            return "radius is not finite or is negative";
    }
    return "unknown body error";
}

BodyStore& BodyStore::operator=(const BodyStore& other) {
    BodyStore copy(other);
    swap(copy);
    return *this;
}

void BodyStore::swap(BodyStore& other) noexcept {
    using std::swap;
    swap(next_id_, other.next_id_);
    swap(ids_exhausted_, other.ids_exhausted_);
    swap(ids_, other.ids_);
    swap(names_, other.names_);
    swap(kinds_, other.kinds_);
    swap(masses_, other.masses_);
    swap(px_, other.px_);
    swap(py_, other.py_);
    swap(pz_, other.pz_);
    swap(vx_, other.vx_);
    swap(vy_, other.vy_);
    swap(vz_, other.vz_);
    swap(radii_, other.radii_);
}

Result<BodyId, BodyError> BodyStore::add(BodyInit init) {
    if (const auto error = validate_kind(init.kind)) {
        return failure(*error);
    }
    if (const auto error = validate_mass(init.kind, init.mass)) {
        return failure(*error);
    }
    if (!isfinite(init.position) || !isfinite(init.velocity)) {
        return failure(BodyError::non_finite_state);
    }
    if (ids_exhausted_) {
        return failure(BodyError::id_space_exhausted);
    }

    // Reserve every column first so that a failed allocation leaves the store
    // unchanged and the push_backs below cannot throw.
    const std::size_t new_size = ids_.size() + 1;
    grow_for(ids_, new_size);
    grow_for(names_, new_size);
    grow_for(kinds_, new_size);
    grow_for(masses_, new_size);
    grow_for(px_, new_size);
    grow_for(py_, new_size);
    grow_for(pz_, new_size);
    grow_for(vx_, new_size);
    grow_for(vy_, new_size);
    grow_for(vz_, new_size);
    grow_for(radii_, new_size);

    const BodyId id{next_id_};
    if (next_id_ == std::numeric_limits<std::uint32_t>::max()) {
        ids_exhausted_ = true;
    } else {
        ++next_id_;
    }

    ids_.push_back(id);
    names_.push_back(std::move(init.name));
    kinds_.push_back(init.kind);
    masses_.push_back(init.mass);
    px_.push_back(init.position.x);
    py_.push_back(init.position.y);
    pz_.push_back(init.position.z);
    vx_.push_back(init.velocity.x);
    vy_.push_back(init.velocity.y);
    vz_.push_back(init.velocity.z);
    radii_.emplace_back(0.0);
    return id;
}

bool BodyStore::remove(BodyId id) {
    const auto index = index_of(id);
    if (!index) {
        return false;
    }
    const std::size_t i = *index;
    erase_at(ids_, i);
    erase_at(names_, i);
    erase_at(kinds_, i);
    erase_at(masses_, i);
    erase_at(px_, i);
    erase_at(py_, i);
    erase_at(pz_, i);
    erase_at(vx_, i);
    erase_at(vy_, i);
    erase_at(vz_, i);
    erase_at(radii_, i);
    return true;
}

std::optional<std::size_t> BodyStore::index_of(BodyId id) const noexcept {
    const auto it = std::ranges::lower_bound(ids_, id);
    if (it == ids_.end() || *it != id) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(std::distance(ids_.begin(), it));
}

Vec3<space::Length> BodyStore::position(std::size_t index) const {
    return {px_.at(index), py_.at(index), pz_.at(index)};
}

Vec3<space::Velocity> BodyStore::velocity(std::size_t index) const {
    return {vx_.at(index), vy_.at(index), vz_.at(index)};
}

Result<BodyId, BodyError> BodyStore::set_position(std::size_t index,
                                                  const Vec3<space::Length>& position) {
    const BodyId body = ids_.at(index);
    if (!isfinite(position)) {
        return failure(BodyError::non_finite_state);
    }
    px_[index] = position.x;
    py_[index] = position.y;
    pz_[index] = position.z;
    return body;
}

Result<BodyId, BodyError> BodyStore::set_velocity(std::size_t index,
                                                  const Vec3<space::Velocity>& velocity) {
    const BodyId body = ids_.at(index);
    if (!isfinite(velocity)) {
        return failure(BodyError::non_finite_state);
    }
    vx_[index] = velocity.x;
    vy_[index] = velocity.y;
    vz_[index] = velocity.z;
    return body;
}

Result<BodyId, BodyError> BodyStore::set_radius(std::size_t index, space::Length radius) {
    const BodyId body = ids_.at(index);
    if (!isfinite(radius) || radius.value() < 0.0) {
        return failure(BodyError::invalid_radius);
    }
    radii_[index] = radius;
    return body;
}

Result<BodyId, BodyError> BodyStore::set_mass(std::size_t index, space::Mass mass) {
    const BodyId body = ids_.at(index);
    if (const auto error = validate_mass(kinds_[index], mass)) {
        return failure(*error);
    }
    masses_[index] = mass;
    return body;
}

} // namespace orrery::core
