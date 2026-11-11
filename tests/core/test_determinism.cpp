#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>

#include "gravity_test_support.h"
#include "orrery/core/body.h"
#include "orrery/core/force_model.h"
#include "orrery/core/integrator.h"
#include "orrery/core/state_hash.h"

using namespace orrery::core;
using namespace orrery::core::test;

namespace {

std::string hex(std::uint64_t value) {
    std::array<char, 24> buffer{};
    std::snprintf(buffer.data(), buffer.size(), "0x%016llx",
                  static_cast<unsigned long long>(value));
    return std::string(buffer.data());
}

// Golden hashes of golden_initial_state() and of golden_run()
// (gravity_test_support.h: literal initial state, softening 1e-4 AU, 10000
// steps of 1 day), measured on MSVC x64 (/fp:precise). The integration loop
// uses only IEEE-754 +, -, *, / and sqrt, which are correctly rounded, and fp
// contraction is off project-wide, so the values are platform independent;
// they were reproduced bit for bit with GCC 15 -O3 on x86-64 Linux
// (docs/physics.md, determinism contract). The hash includes each body's kind
// tag and kind payload.
constexpr std::uint64_t golden_initial = 0xaf61005ef03001b2ULL;
constexpr std::uint64_t golden_leapfrog = 0xe0fa7b48f95f973eULL;
constexpr std::uint64_t golden_yoshida4 = 0x9edf045b7f8169adULL;
constexpr std::uint64_t golden_rk4 = 0xec2fbc333855be6dULL;

} // namespace

TEST_CASE("FNV-1a hash folds bytes least significant first", "[core][determinism]") {
    // FNV-1a 64 of the eight bytes "abcdefgh" (0x61 .. 0x68) computed bytewise.
    std::uint64_t reference = fnv1a_offset_basis;
    for (unsigned char c = 'a'; c <= 'h'; ++c) {
        reference ^= c;
        reference *= fnv1a_prime;
    }
    CHECK(fnv1a_append(fnv1a_offset_basis, 0x6867666564636261ULL) == reference);
    // Known FNV-1a 64 test vector for the single byte "a".
    std::uint64_t single = fnv1a_offset_basis;
    single ^= 0x61U;
    single *= fnv1a_prime;
    CHECK(single == 0xaf63dc4c8601ec8cULL);

    // The hash sees every hashed column, including the sign of zero.
    SimState a = solar_system();
    SimState b = a;
    CHECK(state_hash(a) == state_hash(b));
    REQUIRE(b.bodies().set_velocity(0, {space::Velocity(-0.0), {}, {}}));
    REQUIRE(a.bodies().set_velocity(0, {space::Velocity(0.0), {}, {}}));
    CHECK(state_hash(a) != state_hash(b));
    b.set_time(space::Time(1.0));
    CHECK(state_hash(b) != state_hash(SimState(space::Time(0.0), b.bodies())));
}

TEST_CASE("state hash covers the kind and every other hashed field", "[core][determinism]") {
    const Vec3<space::Length> origin{};
    const Vec3<space::Length> x1{space::Length(1.0), {}, {}};
    const auto pair = [&](const BodyKind& kind_b) {
        BodyStore store;
        add_or_throw(store, BodyInit{"A", Star{}, space::Mass(1.0), origin, {}});
        add_or_throw(store, BodyInit{"B", kind_b, space::Mass(1.0), x1, {}});
        return SimState(space::Time(0.0), std::move(store));
    };
    // Same columns, different source membership: A is accelerated only when B
    // is a planet.
    const SimState planet = pair(Planet{});
    const SimState probe = pair(Probe{});
    const NewtonianGravity gravity;
    std::array<Vec3<space::Length>, 2> positions{origin, x1};
    std::array<Vec3<space::Acceleration>, 2> a_planet{};
    std::array<Vec3<space::Acceleration>, 2> a_probe{};
    gravity.accelerations(planet.bodies(), positions, a_planet);
    gravity.accelerations(probe.bodies(), positions, a_probe);
    REQUIRE(a_planet[0] != a_probe[0]);
    CHECK(state_hash(planet) != state_hash(probe));

    // Every kind tag and the kind payloads.
    const std::array<BodyKind, 9> kinds = {Star{},     Planet{},          Moon{},
                                           Asteroid{}, BlackHole{0.25},   BlackHole{0.5},
                                           Probe{},    Probe{AeroRef{1}}, Probe{AeroRef{2}}};
    for (std::size_t p = 0; p < kinds.size(); ++p) {
        for (std::size_t q = p + 1; q < kinds.size(); ++q) {
            CHECK(state_hash(pair(kinds[p])) != state_hash(pair(kinds[q])));
        }
    }

    // Field mutations: time, id, mass, position and velocity components.
    const std::uint64_t base = state_hash(planet);
    CHECK(state_hash(SimState(space::Time(1.0), planet.bodies())) != base);
    {
        BodyStore shifted;
        const auto removed = shifted.add(BodyInit{"X", Planet{}, space::Mass(1.0), origin, {}});
        REQUIRE(removed);
        REQUIRE(shifted.remove(removed.value()));
        add_or_throw(shifted, BodyInit{"A", Star{}, space::Mass(1.0), origin, {}});
        add_or_throw(shifted, BodyInit{"B", Planet{}, space::Mass(1.0), x1, {}});
        CHECK(state_hash(SimState(space::Time(0.0), std::move(shifted))) != base);
    }
    {
        BodyStore heavier;
        add_or_throw(heavier, BodyInit{"A", Star{}, space::Mass(2.0), origin, {}});
        add_or_throw(heavier, BodyInit{"B", Planet{}, space::Mass(1.0), x1, {}});
        CHECK(state_hash(SimState(space::Time(0.0), std::move(heavier))) != base);
    }
    for (int axis = 0; axis < 3; ++axis) {
        Vec3<space::Length> r = x1;
        Vec3<space::Velocity> v{};
        (axis == 0 ? r.x : (axis == 1 ? r.y : r.z)) += space::Length(0.5);
        (axis == 0 ? v.x : (axis == 1 ? v.y : v.z)) = space::Velocity(0.5);
        SimState moved = planet;
        REQUIRE(moved.bodies().set_position(1, r));
        CHECK(state_hash(moved) != base);
        SimState kicked = planet;
        REQUIRE(kicked.bodies().set_velocity(1, v));
        CHECK(state_hash(kicked) != base);
    }
    // Names are not hashed.
    BodyStore renamed;
    add_or_throw(renamed, BodyInit{"C", Star{}, space::Mass(1.0), origin, {}});
    add_or_throw(renamed, BodyInit{"D", Planet{}, space::Mass(1.0), x1, {}});
    CHECK(state_hash(SimState(space::Time(0.0), std::move(renamed))) == base);
}

TEST_CASE("an isolated planet and probe hash differently after free motion",
          "[core][determinism]") {
    const auto run = [](const BodyKind& kind) {
        BodyStore store;
        add_or_throw(store, BodyInit{"B",
                                     kind,
                                     space::Mass(1.0),
                                     {space::Length(1.0), {}, {}},
                                     {{}, space::Velocity(0.01), {}}});
        SimState state(space::Time(0.0), std::move(store));
        LeapfrogKdk integrator;
        advance(integrator, state, space::Time(1.0), 10000, NewtonianGravity{});
        return state;
    };
    const SimState planet = run(Planet{});
    const SimState probe = run(Probe{});
    REQUIRE(planet.bodies().position(0) == probe.bodies().position(0));
    CHECK(state_hash(planet) != state_hash(probe));
}

TEST_CASE("same inputs give bit-identical states", "[core][determinism]") {
    for (const IntegratorKind kind :
         {IntegratorKind::leapfrog_kdk, IntegratorKind::yoshida4, IntegratorKind::rk4}) {
        const SimState first = golden_run(kind);
        const SimState second = golden_run(kind);
        CHECK(state_hash(first) == state_hash(second));
        for (std::size_t i = 0; i < first.bodies().size(); ++i) {
            CHECK(first.bodies().position(i) == second.bodies().position(i));
            CHECK(first.bodies().velocity(i) == second.bodies().velocity(i));
        }
    }
}

TEST_CASE("golden initial-state hash", "[core][determinism][acceptance]") {
    const std::uint64_t initial = state_hash(golden_initial_state());
    report("golden initial-state hash " + hex(initial));
    CHECK(initial == golden_initial);
}

TEST_CASE("golden hash after 10000 steps", "[core][determinism][acceptance]") {
    const std::uint64_t leapfrog = state_hash(golden_run(IntegratorKind::leapfrog_kdk));
    const std::uint64_t yoshida = state_hash(golden_run(IntegratorKind::yoshida4));
    const std::uint64_t rk4 = state_hash(golden_run(IntegratorKind::rk4));
    report("golden hash leapfrog_kdk " + hex(leapfrog) + ", yoshida4 " + hex(yoshida) + ", rk4 " +
           hex(rk4));
    CHECK(leapfrog == golden_leapfrog);
    CHECK(yoshida == golden_yoshida4);
    CHECK(rk4 == golden_rk4);
}
