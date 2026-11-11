#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <limits>
#include <new>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "allocation_failure_injection.h"
#include "orrery/core/body_store.h"

using namespace orrery::core;

namespace {

Vec3<space::Length> position(double x, double y, double z) {
    return {space::Length(x), space::Length(y), space::Length(z)};
}

Vec3<space::Velocity> velocity(double x, double y, double z) {
    return {space::Velocity(x), space::Velocity(y), space::Velocity(z)};
}

BodyInit body(std::string name, BodyKind kind, double mass, double x) {
    return BodyInit{std::move(name), kind, space::Mass(mass), position(x, 0.0, 0.0),
                    velocity(0.0, x, 0.0)};
}

BodyStore make_system() {
    BodyStore store;
    REQUIRE(store.add(body("Sun", Star{}, 1.0, 0.0)));
    REQUIRE(store.add(body("Earth", Planet{}, 3.0e-6, 1.0)));
    REQUIRE(store.add(body("Moon", Moon{}, 3.7e-8, 1.00257)));
    REQUIRE(store.add(body("Ceres", Asteroid{}, 4.7e-10, 2.77)));
    REQUIRE(store.add(body("Sgr A*", BlackHole{0.9}, 4.3e6, 1.0e9)));
    REQUIRE(store.add(body("Capsule", Probe{AeroRef{7}}, 5.0e-27, 1.0001)));
    return store;
}

} // namespace

static_assert(std::variant_size_v<BodyKind> == 6);
static_assert(tag_of(BodyKind{Star{}}) == BodyKindTag::star);
static_assert(tag_of(BodyKind{Planet{}}) == BodyKindTag::planet);
static_assert(tag_of(BodyKind{Moon{}}) == BodyKindTag::moon);
static_assert(tag_of(BodyKind{Asteroid{}}) == BodyKindTag::asteroid);
static_assert(tag_of(BodyKind{BlackHole{}}) == BodyKindTag::black_hole);
static_assert(tag_of(BodyKind{Probe{}}) == BodyKindTag::probe);
static_assert(exerts_gravity(BodyKind{BlackHole{0.5}}));
static_assert(!exerts_gravity(BodyKind{Probe{}}));

TEST_CASE("bodies keep insertion order and sequential ids", "[core][body_store]") {
    const BodyStore store = make_system();
    REQUIRE(store.size() == 6);
    const std::vector<std::string> expected{"Sun", "Earth", "Moon", "Ceres", "Sgr A*", "Capsule"};
    for (std::size_t i = 0; i < store.size(); ++i) {
        CHECK(store.name(i) == expected[i]);
        CHECK(store.id(i).value == i);
        CHECK(store.index_of(store.id(i)) == i);
    }
    CHECK(to_string(tag_of(store.kind(4))) == "black_hole");
    CHECK(std::get<BlackHole>(store.kind(4)).spin == 0.9);
    CHECK(std::get<Probe>(store.kind(5)).aero_ref == AeroRef{7});
}

TEST_CASE("probes have mass but are not gravity sources", "[core][body_store]") {
    const BodyStore store = make_system();
    for (std::size_t i = 0; i < store.size(); ++i) {
        const bool is_probe = std::holds_alternative<Probe>(store.kind(i));
        CHECK(store.exerts_gravity(i) == !is_probe);
    }
    CHECK(store.mass(5) == space::Mass(5.0e-27));
}

TEST_CASE("columns are contiguous and consistent with accessors", "[core][body_store]") {
    const BodyStore store = make_system();
    REQUIRE(store.position_x().size() == store.size());
    REQUIRE(store.velocity_z().size() == store.size());
    REQUIRE(store.masses().size() == store.size());
    for (std::size_t i = 0; i < store.size(); ++i) {
        CHECK(store.position(i).x == store.position_x()[i]);
        CHECK(store.velocity(i).y == store.velocity_y()[i]);
        CHECK(store.masses()[i] == store.mass(i));
        CHECK(store.ids()[i] == store.id(i));
    }
}

TEST_CASE("removal preserves order and ids are never reused", "[core][body_store]") {
    BodyStore store = make_system();
    const BodyId earth = store.id(1);
    const BodyId ceres = store.id(3);
    CHECK(store.remove(earth));
    CHECK_FALSE(store.remove(earth));
    CHECK(store.remove(ceres));
    REQUIRE(store.size() == 4);
    CHECK_FALSE(store.contains(earth));
    CHECK(store.name(0) == "Sun");
    CHECK(store.name(1) == "Moon");
    CHECK(store.name(2) == "Sgr A*");
    CHECK(store.name(3) == "Capsule");
    CHECK(store.position(1).x == space::Length(1.00257));

    const auto added = store.add(body("Mars", Planet{}, 3.2e-7, 1.52));
    REQUIRE(added);
    CHECK(added.value() == BodyId{6});
    CHECK(store.index_of(added.value()) == 4);
    CHECK(store.index_of(BodyId{1}) == std::nullopt);
}

TEST_CASE("identical operation sequences give identical stores", "[core][body_store]") {
    const auto run = [] {
        BodyStore store = make_system();
        CHECK(store.remove(BodyId{2}));
        CHECK(store.add(body("Vesta", Asteroid{}, 1.3e-10, 2.36)));
        CHECK(store.set_position(0, position(0.1, 0.2, 0.3)));
        return store;
    };
    const BodyStore a = run();
    const BodyStore b = run();
    REQUIRE(a.size() == b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        CHECK(a.id(i) == b.id(i));
        CHECK(a.name(i) == b.name(i));
        CHECK(a.kind(i) == b.kind(i));
        CHECK(a.mass(i) == b.mass(i));
        CHECK(a.position(i) == b.position(i));
        CHECK(a.velocity(i) == b.velocity(i));
    }
}

TEST_CASE("invalid bodies are rejected without changing the store", "[core][body_store]") {
    BodyStore store = make_system();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    auto r = store.add(body("neg", Planet{}, -1.0, 1.0));
    REQUIRE_FALSE(r);
    CHECK(r.error() == BodyError::invalid_mass);
    CHECK(store.add(body("nan", Planet{}, nan, 1.0)).error() == BodyError::non_finite_mass);
    CHECK(store.add(body("probe", Probe{}, 0.0, 1.0)).error() == BodyError::invalid_mass);
    CHECK(store.add(body("hole", BlackHole{}, 0.0, 1.0)).error() == BodyError::invalid_mass);
    CHECK(store.add(body("spin", BlackHole{1.5}, 1.0, 1.0)).error() == BodyError::invalid_spin);
    CHECK(store.add(body("spin", BlackHole{nan}, 1.0, 1.0)).error() == BodyError::invalid_spin);
    CHECK(store.add(body("far", Planet{}, 1.0, inf)).error() == BodyError::non_finite_state);
    // A massless asteroid (test particle) is allowed.
    CHECK(store.add(body("dust", Asteroid{}, 0.0, 3.0)));
    CHECK(store.size() == 7);

    const auto before = store.position(0);
    CHECK(store.set_position(0, position(nan, 0.0, 0.0)).error() == BodyError::non_finite_state);
    CHECK(store.set_velocity(0, velocity(0.0, inf, 0.0)).error() == BodyError::non_finite_state);
    CHECK(store.position(0) == before);
    CHECK_THROWS_AS(store.mass(store.size()), std::out_of_range);
    CHECK_FALSE(to_string(BodyError::invalid_spin).empty());
}

namespace {

// Every column has size() entries.
bool columns_consistent(const BodyStore& store) {
    const std::size_t n = store.size();
    if (store.masses().size() != n || store.position_x().size() != n ||
        store.position_y().size() != n || store.position_z().size() != n ||
        store.velocity_x().size() != n || store.velocity_y().size() != n ||
        store.velocity_z().size() != n) {
        return false;
    }
    for (std::size_t i = 0; i < n; ++i) {
        try {
            static_cast<void>(store.name(i));
            static_cast<void>(store.kind(i));
        } catch (const std::out_of_range&) {
            return false;
        }
    }
    return true;
}

bool same_bodies(const BodyStore& a, const BodyStore& b) {
    if (a.size() != b.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (a.id(i) != b.id(i) || a.name(i) != b.name(i) || a.kind(i) != b.kind(i) ||
            a.mass(i) != b.mass(i) || a.position(i) != b.position(i) ||
            a.velocity(i) != b.velocity(i)) {
            return false;
        }
    }
    return true;
}

} // namespace

static_assert(std::is_nothrow_move_constructible_v<BodyStore>);
static_assert(std::is_nothrow_move_assignable_v<BodyStore>);

TEST_CASE("copy assignment is all-or-nothing under allocation failure", "[core][body_store]") {
    // ASTRA-03: assign a larger store with long (heap-allocated) names to a
    // smaller one, failing each allocation in turn.
    const std::string padding(48, '#');
    BodyStore source;
    REQUIRE(source.add(body("Sun" + padding, Star{}, 1.0, 0.0)));
    REQUIRE(source.add(body("Jupiter" + padding, Planet{}, 9.5e-4, 5.2)));
    REQUIRE(source.add(body("Io" + padding, Moon{}, 4.5e-8, 5.203)));

    BodyStore original;
    REQUIRE(original.add(body("Probe" + padding, Probe{AeroRef{3}}, 1.0e-27, 7.0)));

    bool completed = false;
    int failures = 0;
    for (long long failure_at = 0; failure_at < 1000 && !completed; ++failure_at) {
        BodyStore destination = original;
        REQUIRE(destination.position_x().size() == 1);
        bool threw = false;
        bool fired = false;
        {
            orrery::test::AllocationFailureScope inject(failure_at);
            try {
                destination = source;
            } catch (const std::bad_alloc&) {
                threw = true;
            }
            inject.disarm();
            fired = inject.fired();
        }
        // Only the injected failure can make the assignment throw.
        REQUIRE((fired || !threw));

        INFO("allocation failure injected at " << failure_at);
        REQUIRE(columns_consistent(destination));
        if (threw) {
            ++failures;
            CHECK(same_bodies(destination, original));
        } else {
            completed = true;
            CHECK(same_bodies(destination, source));
        }
        // The store stays fully usable.
        REQUIRE(destination.set_position(0, position(0.5, 0.5, 0.5)));
        REQUIRE(destination.set_velocity(destination.size() - 1, velocity(0.0, 0.1, 0.0)));
        REQUIRE(destination.add(body("Extra" + padding, Asteroid{}, 0.0, 9.0)));
        REQUIRE(destination.remove(destination.id(0)));
        CHECK(columns_consistent(destination));
    }
    CHECK(completed);
    CHECK(failures > 1);

    // Self-assignment and large-to-small assignment keep the contents.
    BodyStore self = source;
    const BodyStore& alias = self;
    self = alias;
    CHECK(same_bodies(self, source));
    BodyStore small = original;
    small = source;
    CHECK(same_bodies(small, source));
    CHECK(columns_consistent(small));
    BodyStore moved = std::move(small);
    CHECK(same_bodies(moved, source));
}
