#pragma once

#include <cstdint>

#include "orrery/core/sim_state.h"

namespace orrery::core {

// 64-bit FNV-1a offset basis and prime (Fowler, Noll, Vo).
inline constexpr std::uint64_t fnv1a_offset_basis = 14695981039346656037ULL;
inline constexpr std::uint64_t fnv1a_prime = 1099511628211ULL;

// Folds the 8 bytes of `word` into an FNV-1a hash, least significant byte
// first (independent of the host byte order).
[[nodiscard]] constexpr std::uint64_t fnv1a_append(std::uint64_t hash,
                                                   std::uint64_t word) noexcept {
    for (int byte = 0; byte < 8; ++byte) {
        hash ^= (word >> (8 * byte)) & 0xFFU;
        hash *= fnv1a_prime;
    }
    return hash;
}

// Fingerprint of the dynamical state for the determinism contract: FNV-1a
// over the time and, in body order, each body's id, kind tag (BodyKindTag
// value), kind payload (the IEEE-754 bits of a black hole's spin, a probe's
// aero_ref value, 0 for the other kinds), mass, position (x, y, z) and
// velocity (x, y, z); doubles contribute their IEEE-754 bit patterns. Each
// value is widened to 64 bits and fed least significant byte first. Two
// states hash equal when these values are bit-identical (so 0.0 and -0.0
// differ). Names are not hashed.
[[nodiscard]] std::uint64_t state_hash(const SimState& state) noexcept;

} // namespace orrery::core
