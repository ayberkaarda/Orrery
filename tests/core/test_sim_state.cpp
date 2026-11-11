#include <catch2/catch_test_macros.hpp>

#include <type_traits>
#include <utility>

#include "orrery/core/sim_state.h"

using namespace orrery::core;

static_assert(SimState::units() == UnitSystemId::space);
static_assert(std::is_same_v<SimState::unit_system, SpaceUnits>);

TEST_CASE("default state is empty at time zero", "[core][sim_state]") {
    const SimState state;
    CHECK(state.time() == space::Time(0.0));
    CHECK(state.bodies().empty());
    CHECK(state.units() == UnitSystemId::space);
}

TEST_CASE("state owns its bodies and time", "[core][sim_state]") {
    BodyStore bodies;
    REQUIRE(bodies.add(BodyInit{"Sun", Star{}, space::Mass(1.0), {}, {}}));
    SimState state(space::Time(2451545.0), std::move(bodies));
    CHECK(state.time() == space::Time(2451545.0));
    REQUIRE(state.bodies().size() == 1);
    CHECK(state.bodies().name(0) == "Sun");

    state.set_time(state.time() + space::Time(1.5));
    CHECK(state.time() == space::Time(2451546.5));

    REQUIRE(state.bodies().add(BodyInit{"Earth", Planet{}, space::Mass(3.0e-6), {}, {}}));
    CHECK(state.bodies().size() == 2);

    const SimState copy = state;
    CHECK(copy.bodies().size() == 2);
    CHECK(copy.time() == state.time());
}
