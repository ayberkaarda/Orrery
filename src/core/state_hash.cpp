#include "orrery/core/state_hash.h"

#include <bit>
#include <cstddef>
#include <variant>

#include "orrery/core/body.h"

namespace orrery::core {

namespace {

std::uint64_t bits(double value) noexcept {
    return std::bit_cast<std::uint64_t>(value);
}

// Kind payload word: black-hole spin bits, probe aero_ref, 0 otherwise.
std::uint64_t kind_payload(const BodyKind& kind) noexcept {
    if (const auto* hole = std::get_if<BlackHole>(&kind)) {
        return bits(hole->spin);
    }
    if (const auto* probe = std::get_if<Probe>(&kind)) {
        return probe->aero_ref.value;
    }
    return 0;
}

} // namespace

std::uint64_t state_hash(const SimState& state) noexcept {
    const BodyStore& bodies = state.bodies();
    std::uint64_t hash = fnv1a_offset_basis;
    hash = fnv1a_append(hash, bits(state.time().value()));
    const std::size_t n = bodies.size();
    for (std::size_t i = 0; i < n; ++i) {
        const BodyKind& kind = bodies.kind(i);
        hash = fnv1a_append(hash, bodies.ids()[i].value);
        hash = fnv1a_append(hash, static_cast<std::uint64_t>(tag_of(kind)));
        hash = fnv1a_append(hash, kind_payload(kind));
        hash = fnv1a_append(hash, bits(bodies.masses()[i].value()));
        hash = fnv1a_append(hash, bits(bodies.position_x()[i].value()));
        hash = fnv1a_append(hash, bits(bodies.position_y()[i].value()));
        hash = fnv1a_append(hash, bits(bodies.position_z()[i].value()));
        hash = fnv1a_append(hash, bits(bodies.velocity_x()[i].value()));
        hash = fnv1a_append(hash, bits(bodies.velocity_y()[i].value()));
        hash = fnv1a_append(hash, bits(bodies.velocity_z()[i].value()));
        hash = fnv1a_append(hash, bits(bodies.radii()[i].value()));
    }
    return hash;
}

} // namespace orrery::core
