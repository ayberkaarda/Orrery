#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <variant>

#include <glm/geometric.hpp>

#include "orrery/render/camera.h"

// Turntable camera mode of the aero scene (spec 7.1): automatic spin, orbit
// input, pan along the turning right axis, and the time-step contract shared
// with the other modes.

using namespace orrery::render;

namespace {

Camera turntable(const TurntableMode& mode, double smoothing = 0.0) {
    Camera camera;
    camera.set_smoothing_time(smoothing);
    camera.set_mode(mode);
    camera.update(0.0, CameraInput{}, BodyFrameLookup{});
    return camera;
}

glm::dvec3 forward(const Camera& camera) {
    return camera.orientation() * glm::dvec3(0.0, 0.0, -1.0);
}

} // namespace

TEST_CASE("Turntable looks at its focus from yaw, pitch and distance",
          "[render][camera][turntable]") {
    TurntableMode mode;
    mode.focus = {1.0, -2.0, 0.5};
    mode.distance = 4.0;
    mode.yaw = 0.7;
    mode.pitch = 0.3;
    const Camera camera = turntable(mode);
    CHECK(camera.mode_kind() == CameraModeKind::turntable);
    const glm::dvec3 eye = camera.eye_position();
    CHECK(glm::length(eye - mode.focus) == Catch::Approx(4.0));
    CHECK(glm::dot(forward(camera), glm::normalize(mode.focus - eye)) == Catch::Approx(1.0));
    CHECK((eye.z - mode.focus.z) == Catch::Approx(4.0 * std::sin(0.3)));
    // World +Z stays up: the camera right axis is horizontal.
    const glm::dvec3 right = camera.orientation() * glm::dvec3(1.0, 0.0, 0.0);
    CHECK(right.z == Catch::Approx(0.0).margin(1.0e-12));
}

TEST_CASE("Turntable spins on its own and adds the yaw input", "[render][camera][turntable]") {
    TurntableMode mode;
    mode.spin_rate = 0.25;
    Camera camera = turntable(mode);
    camera.update(2.0, CameraInput{}, BodyFrameLookup{});
    const auto& now = std::get<TurntableMode>(camera.current());
    CHECK(now.yaw == Catch::Approx(mode.yaw + 0.5));
    CHECK(now.distance == Catch::Approx(mode.distance));
    CameraInput input;
    input.yaw = 0.5;
    input.pitch = 10.0; // clamps below the pole
    input.zoom = std::log(2.0);
    camera.update(1.0, input, BodyFrameLookup{});
    const auto& after = std::get<TurntableMode>(camera.current());
    CHECK(after.yaw == Catch::Approx(mode.yaw + 0.5 + 0.75));
    CHECK(after.pitch < 1.5708);
    CHECK(after.pitch > 1.56);
    CHECK(after.distance == Catch::Approx(mode.distance / 2.0));
}

TEST_CASE("Turntable pan follows the turning right axis in closed form",
          "[render][camera][turntable]") {
    TurntableMode mode;
    mode.yaw = 0.0;
    mode.distance = 2.0;
    CameraInput input;
    input.move = {1.0, 0.5, 0.0};
    // Without rotation or zoom: right = (0, 1, 0), distance per second.
    {
        Camera camera = turntable(mode);
        camera.update(1.5, input, BodyFrameLookup{});
        const auto& now = std::get<TurntableMode>(camera.current());
        CHECK(now.focus.x == Catch::Approx(0.0).margin(1.0e-12));
        CHECK(now.focus.y == Catch::Approx(3.0));
        CHECK(now.focus.z == Catch::Approx(1.5));
    }
    // With spin: the focus moves along an arc; integral of 2 (-sin wt, cos wt).
    mode.spin_rate = 1.0;
    {
        Camera camera = turntable(mode);
        camera.update(1.0, input, BodyFrameLookup{});
        const auto& now = std::get<TurntableMode>(camera.current());
        CHECK(now.focus.x == Catch::Approx(-2.0 * (1.0 - std::cos(1.0))));
        CHECK(now.focus.y == Catch::Approx(2.0 * std::sin(1.0)));
    }
}

TEST_CASE("Turntable obeys the time-step contract", "[render][camera][turntable]") {
    TurntableMode mode;
    mode.spin_rate = 0.4;
    CameraInput input;
    input.move = {0.6, -0.3, 0.0};
    input.yaw = 0.2;
    input.pitch = 0.15;
    input.zoom = 0.5;
    const auto run = [&](int updates) {
        Camera camera = turntable(mode, 0.15);
        for (int i = 0; i < updates; ++i) {
            camera.update(1.0 / updates, input, BodyFrameLookup{});
        }
        return std::get<TurntableMode>(camera.current());
    };
    const TurntableMode coarse = run(4);
    const TurntableMode fine = run(240);
    CHECK(coarse.yaw == Catch::Approx(fine.yaw).margin(1.0e-9));
    CHECK(coarse.pitch == Catch::Approx(fine.pitch).margin(1.0e-9));
    CHECK(coarse.distance == Catch::Approx(fine.distance).epsilon(1.0e-9));
    CHECK(glm::length(coarse.focus - fine.focus) < 1.0e-6);
}
