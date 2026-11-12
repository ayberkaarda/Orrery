#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <memory>
#include <optional>
#include <type_traits>
#include <utility>
#include <variant>

#include "orrery/core/sim_snapshot.h"

using namespace orrery::core;

namespace {

SimState three_body_state() {
    BodyStore bodies;
    REQUIRE(bodies.add(BodyInit{"Sun", Star{}, space::Mass(1.0), {}, {}}));
    REQUIRE(bodies.add(
        BodyInit{"Earth",
                 Planet{},
                 space::Mass(3.0e-6),
                 {space::Length(1.0), space::Length(0.0), space::Length(0.0)},
                 {space::Velocity(0.0), space::Velocity(0.0172), space::Velocity(0.0)}}));
    REQUIRE(bodies.add(BodyInit{"Gate",
                                BlackHole{0.4},
                                space::Mass(10.0),
                                {space::Length(-3.0), space::Length(2.0), space::Length(0.5)},
                                {}}));
    REQUIRE(bodies.set_radius(1, space::Length(4.26e-5)));
    return {space::Time(12.5), std::move(bodies)};
}

} // namespace

// The renderer receives snapshots only as shared_ptr<const SimSnapshot>.
static_assert(std::is_same_v<decltype(SimSnapshot::capture(std::declval<const SimState&>())),
                             std::shared_ptr<const SimSnapshot>>);
static_assert(!std::is_default_constructible_v<SimSnapshot>);
// No writable copy of a published snapshot can be made.
static_assert(!std::is_copy_constructible_v<SimSnapshot>);
static_assert(!std::is_move_constructible_v<SimSnapshot>);
static_assert(!std::is_copy_assignable_v<SimSnapshot>);
static_assert(!std::is_move_assignable_v<SimSnapshot>);

TEST_CASE("a snapshot copies every observable column of the state", "[core][snapshot]") {
    const SimState state = three_body_state();
    const std::shared_ptr<const SimSnapshot> snapshot = SimSnapshot::capture(state, 42);
    REQUIRE(snapshot != nullptr);
    CHECK(snapshot->time() == space::Time(12.5));
    CHECK(snapshot->sequence() == 42);
    REQUIRE(snapshot->size() == 3);
    CHECK(snapshot->names()[0] == "Sun");
    CHECK(snapshot->names()[2] == "Gate");
    CHECK(snapshot->kind_tag(0) == BodyKindTag::star);
    CHECK(snapshot->kind_tag(1) == BodyKindTag::planet);
    REQUIRE(snapshot->kind_tag(2) == BodyKindTag::black_hole);
    CHECK(std::get<BlackHole>(snapshot->kinds()[2]).spin == 0.4);
    CHECK(snapshot->masses()[1] == space::Mass(3.0e-6));
    CHECK(snapshot->radii()[0] == space::Length(0.0));
    CHECK(snapshot->radii()[1] == space::Length(4.26e-5));
    CHECK(snapshot->positions()[2].z == space::Length(0.5));
    CHECK(snapshot->positions()[1].x == space::Length(1.0));
    CHECK(snapshot->velocities()[1].y == space::Velocity(0.0172));
    for (std::size_t i = 0; i < snapshot->size(); ++i) {
        CHECK(snapshot->ids()[i] == state.bodies().id(i));
    }
}

TEST_CASE("a snapshot is independent of later changes to the state", "[core][snapshot]") {
    SimState state = three_body_state();
    const auto snapshot = SimSnapshot::capture(state);
    REQUIRE(state.bodies().set_position(
        1, {space::Length(9.0), space::Length(9.0), space::Length(9.0)}));
    state.set_time(space::Time(99.0));
    REQUIRE(state.bodies().remove(state.bodies().id(0)));
    CHECK(snapshot->size() == 3);
    CHECK(snapshot->time() == space::Time(12.5));
    CHECK(snapshot->positions()[1].x == space::Length(1.0));
}

TEST_CASE("snapshot id lookup finds every body and rejects removed ids", "[core][snapshot]") {
    SimState state = three_body_state();
    const BodyId removed = state.bodies().id(1);
    REQUIRE(state.bodies().remove(removed));
    const auto snapshot = SimSnapshot::capture(state);
    REQUIRE(snapshot->size() == 2);
    CHECK(snapshot->index_of(state.bodies().id(0)) == std::optional<std::size_t>(0));
    CHECK(snapshot->index_of(state.bodies().id(1)) == std::optional<std::size_t>(1));
    CHECK_FALSE(snapshot->index_of(removed).has_value());
    CHECK_FALSE(snapshot->index_of(BodyId{1000}).has_value());
}

TEST_CASE("a snapshot cannot be copied or moved into a mutable object", "[core][snapshot]") {
    // A public copy or move would let a caller build a writable SimSnapshot,
    // publish it as shared_ptr<const SimSnapshot> and then overwrite it while
    // readers hold spans into its columns.
    CHECK_FALSE(std::is_copy_constructible_v<SimSnapshot>);
    CHECK_FALSE(std::is_move_constructible_v<SimSnapshot>);
    CHECK_FALSE(std::is_copy_assignable_v<SimSnapshot>);
    CHECK_FALSE(std::is_move_assignable_v<SimSnapshot>);
    CHECK_FALSE((std::is_constructible_v<SimSnapshot, const SimSnapshot&>));
    CHECK_FALSE((std::is_constructible_v<SimSnapshot, SimSnapshot&&>));
    CHECK_FALSE((std::is_assignable_v<SimSnapshot&, const SimSnapshot&>));

    // Sharing goes through the owning pointer: every copy aliases the one
    // immutable object the factory built.
    const SimState state = three_body_state();
    const std::shared_ptr<const SimSnapshot> first = SimSnapshot::capture(state, 7);
    // NOLINTNEXTLINE(performance-unnecessary-copy-initialization): the copy is the subject.
    const std::shared_ptr<const SimSnapshot> second = first;
    CHECK(second.get() == first.get());
    CHECK(first.use_count() == 2);
    CHECK(second->positions().data() == first->positions().data());
    CHECK(second->sequence() == 7);
}

TEST_CASE("an empty state gives an empty snapshot", "[core][snapshot]") {
    const auto snapshot = SimSnapshot::capture(SimState{});
    CHECK(snapshot->empty());
    CHECK(snapshot->ids().empty());
    CHECK_FALSE(snapshot->index_of(BodyId{0}).has_value());
}
