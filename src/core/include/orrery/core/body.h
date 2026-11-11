#pragma once

#include <compare>
#include <cstdint>
#include <string_view>
#include <variant>

namespace orrery::core {

// Stable identifier of a body. Ids are assigned in increasing order by the
// BodyStore that created the body and are never reused within that store.
struct BodyId {
    std::uint32_t value = 0;

    friend constexpr auto operator<=>(const BodyId&, const BodyId&) noexcept = default;
};

// Opaque handle to the aerodynamic coefficient table (Cd * A versus Mach) of a
// probe. The table itself lives outside core; core only carries the handle.
struct AeroRef {
    std::uint32_t value = 0;

    friend constexpr auto operator<=>(const AeroRef&, const AeroRef&) noexcept = default;
};

struct Star {
    friend constexpr bool operator==(const Star&, const Star&) noexcept = default;
};
struct Planet {
    friend constexpr bool operator==(const Planet&, const Planet&) noexcept = default;
};
struct Moon {
    friend constexpr bool operator==(const Moon&, const Moon&) noexcept = default;
};
struct Asteroid {
    friend constexpr bool operator==(const Asteroid&, const Asteroid&) noexcept = default;
};

struct BlackHole {
    // Dimensionless spin parameter a* = J c / (G M^2), in [0, 1]. Stored only;
    // the dynamics and lensing models are non-rotating (ADR-003, ADR-004).
    double spin = 0.0;

    friend constexpr bool operator==(const BlackHole&, const BlackHole&) noexcept = default;
};

// A test particle: it has a mass (used for drag and entry dynamics) but exerts
// no gravitational force on other bodies.
struct Probe {
    AeroRef aero_ref;

    friend constexpr bool operator==(const Probe&, const Probe&) noexcept = default;
};

using BodyKind = std::variant<Star, Planet, Moon, Asteroid, BlackHole, Probe>;

enum class BodyKindTag : std::uint8_t {
    star,
    planet,
    moon,
    asteroid,
    black_hole,
    probe,
};

[[nodiscard]] constexpr BodyKindTag tag_of(const BodyKind& kind) noexcept {
    return static_cast<BodyKindTag>(kind.index());
}

[[nodiscard]] constexpr std::string_view to_string(BodyKindTag tag) noexcept {
    switch (tag) {
        case BodyKindTag::star:
            return "star";
        case BodyKindTag::planet:
            return "planet";
        case BodyKindTag::moon:
            return "moon";
        case BodyKindTag::asteroid:
            return "asteroid";
        case BodyKindTag::black_hole:
            return "black_hole";
        case BodyKindTag::probe:
            return "probe";
    }
    return "unknown";
}

// APPROXIMATION: probes are test particles - their masses are many orders of
// magnitude below those of the bodies they orbit - valid while the probe mass
// is negligible against every body whose motion matters (relative effect on
// the primary of order m_probe / M_primary).
[[nodiscard]] constexpr bool exerts_gravity(const BodyKind& kind) noexcept {
    return !std::holds_alternative<Probe>(kind);
}

} // namespace orrery::core
