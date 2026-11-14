#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <glm/ext/matrix_float4x4.hpp>
#include <glm/ext/vector_float4.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>

#include "orrery/core/units.h"
#include "orrery/render/camera.h"
#include "orrery/render/log_depth.h"

using orrery::core::BodyId;
using namespace orrery::render;

namespace {

const double kAu = orrery::core::constants::astronomical_unit.value();

// A body 1e4 AU from the world origin (outer Oort cloud scale).
const glm::dvec3 kFarBody{1.0e4 * kAu, -3.0 * kAu, 0.5 * kAu};

BodyFrameLookup single_body(BodyId id, BodyFrame frame) {
    return [id, frame](BodyId query) -> std::optional<BodyFrame> {
        if (query == id) {
            return frame;
        }
        return std::nullopt;
    };
}

double relative_error(const glm::dvec3& actual, const glm::dvec3& expected) {
    return glm::length(actual - expected) / glm::length(expected);
}

glm::dvec3 ndc_of(const Camera& camera, const glm::dvec3& world, double aspect) {
    const glm::vec4 clip =
        camera.view_projection_matrix(aspect) * glm::vec4(camera.to_render(world), 1.0F);
    return glm::dvec3(clip.x, clip.y, clip.z) / static_cast<double>(clip.w);
}

} // namespace

TEST_CASE("Floating origin keeps a body at 1e4 AU stable 1 km from a free camera",
          "[render][camera][precision]") {
    // The eye is 1 km from the body. The world coordinates are absolute doubles
    // (spacing about 0.25 m at 1e4 AU), so the reference offset is the exact
    // difference of the two stored doubles, which floating-origin subtraction
    // reproduces without rounding (Sterbenz lemma).
    const glm::dvec3 intended{600.0, -800.0, 0.0};
    FreeMode free;
    free.position = kFarBody + intended;
    free.orientation = glm::dquat(1.0, 0.0, 0.0, 0.0);
    Camera camera;
    camera.set_mode(free);
    REQUIRE(camera.update(0.0, {}, {}));

    const glm::dvec3 exact = kFarBody - camera.eye_position();
    const glm::dvec3 render = glm::dvec3(camera.to_render(kFarBody));
    CHECK(relative_error(render, exact) < 1.0e-6);
    // Against the intended offset the error is bounded by the world spacing.
    CHECK(glm::length(render + intended) < 0.5);

    // Without the floating origin, float world coordinates lose the offset
    // entirely (float spacing at 1.5e15 m is about 1.3e8 m): here both round
    // to the same float and the body collapses onto the eye.
    const glm::dvec3 naive =
        glm::dvec3(glm::vec3(kFarBody)) - glm::dvec3(glm::vec3(camera.eye_position()));
    CHECK(relative_error(naive, exact) >= 1.0);
}

TEST_CASE("Chase camera 1 km from a body at 1e4 AU has an exact render offset",
          "[render][camera][precision]") {
    const BodyId probe{7};
    BodyFrame frame;
    frame.position = kFarBody;
    frame.velocity = glm::dvec3(0.0, 3.0e4, 0.0);
    ChaseMode chase;
    chase.target = probe;
    chase.distance = 1000.0;
    chase.yaw = 0.3;
    chase.pitch = 0.2;
    Camera camera;
    camera.set_mode(chase);
    REQUIRE(camera.update(0.0, {}, single_body(probe, frame)));

    const glm::dvec3 render = glm::dvec3(camera.to_render(kFarBody));
    CHECK(std::abs(glm::length(render) - 1000.0) / 1000.0 < 1.0e-6);
    CHECK(relative_error(render, -camera.eye_offset()) < 1.0e-7);
    // The body is at the centre of the view.
    const glm::dvec3 ndc = ndc_of(camera, kFarBody, 16.0 / 9.0);
    CHECK(std::abs(ndc.x) < 1.0e-5);
    CHECK(std::abs(ndc.y) < 1.0e-5);
    // yaw = pitch = 0 is behind the body: the eye offset opposes the velocity.
    ChaseMode behind = chase;
    behind.yaw = 0.0;
    behind.pitch = 0.0;
    camera.set_mode(behind);
    REQUIRE(camera.update(0.0, {}, single_body(probe, frame)));
    CHECK(camera.eye_offset().y == Catch::Approx(-1000.0));
}

TEST_CASE("Millimetre camera steps at 1e4 AU move the render position by millimetres",
          "[render][camera][precision]") {
    const BodyId body{3};
    BodyFrame frame;
    frame.position = kFarBody;
    Camera camera;
    camera.set_smoothing_time(0.0);
    OrbitMode orbit;
    orbit.target = body;
    orbit.yaw = 1.1;
    orbit.pitch = -0.4;
    double previous = 0.0;
    for (int step = 0; step <= 100; ++step) {
        orbit.distance = 1000.0 + 1.0e-3 * step;
        camera.set_mode(orbit);
        REQUIRE(camera.update(0.0, {}, single_body(body, frame)));
        const double distance = glm::length(glm::dvec3(camera.to_render(kFarBody)));
        CHECK(std::abs(distance - orbit.distance) < 1.0e-4);
        if (step > 0) {
            CHECK(std::abs((distance - previous) - 1.0e-3) < 2.0e-4);
        }
        previous = distance;
    }
}

TEST_CASE("Orbit places the eye on the requested sphere and looks at the focus",
          "[render][camera]") {
    OrbitMode orbit;
    orbit.focus = glm::dvec3(5.0, -2.0, 1.0);
    orbit.distance = 12.0;
    orbit.yaw = 0.7;
    orbit.pitch = 0.4;
    Camera camera;
    camera.set_mode(orbit);
    REQUIRE(camera.update(0.0, {}, {}));
    const glm::dvec3 expected = 12.0 * glm::dvec3(std::cos(0.4) * std::cos(0.7),
                                                  std::cos(0.4) * std::sin(0.7), std::sin(0.4));
    CHECK(relative_error(camera.eye_offset(), expected) < 1.0e-14);
    const glm::dvec3 ndc = ndc_of(camera, orbit.focus, 1.5);
    CHECK(std::abs(ndc.x) < 1.0e-6);
    CHECK(std::abs(ndc.y) < 1.0e-6);
    // World +Z appears up on screen.
    const glm::dvec3 above = ndc_of(camera, orbit.focus + glm::dvec3(0.0, 0.0, 1.0), 1.5);
    CHECK(above.y > 0.05);
}

TEST_CASE("Orbit view shift puts the target off centre at any yaw, pitch and distance",
          "[render][camera]") {
    // The window frames the system in the scene area left free by its
    // panels: the target is drawn at that area's centre, not the window's.
    const BodyId sun{7};
    BodyFrame frame;
    frame.position = kFarBody;
    const BodyFrameLookup lookup = single_body(sun, frame);
    OrbitMode orbit;
    orbit.target = sun;
    orbit.distance = 3.0 * kAu;
    orbit.yaw = 1.2;
    orbit.pitch = 0.5;
    orbit.view_shift = glm::dvec2(0.25, -0.4); // half view heights, right and up
    Camera camera;
    camera.set_mode(orbit);
    REQUIRE(camera.update(0.0, {}, lookup));
    const double aspect = 1.6;
    glm::dvec3 ndc = ndc_of(camera, kFarBody, aspect);
    CHECK(ndc.x == Catch::Approx(0.25 / aspect).margin(1.0e-5));
    CHECK(ndc.y == Catch::Approx(-0.4).margin(1.0e-5));
    // Orbiting and zooming keep it there: the shift is screen-aligned.
    CameraInput input;
    input.yaw = 0.8;
    input.pitch = -0.3;
    input.zoom = 0.5;
    REQUIRE(camera.update(1.0, input, lookup));
    REQUIRE(camera.update(5.0, {}, lookup));
    ndc = ndc_of(camera, kFarBody, aspect);
    CHECK(ndc.x == Catch::Approx(0.25 / aspect).margin(1.0e-5));
    CHECK(ndc.y == Catch::Approx(-0.4).margin(1.0e-5));
    // Without a shift the target stays at the centre.
    orbit.view_shift = glm::dvec2(0.0);
    camera.set_mode(orbit);
    REQUIRE(camera.update(0.0, {}, lookup));
    ndc = ndc_of(camera, kFarBody, aspect);
    CHECK(std::abs(ndc.x) < 1.0e-6);
    CHECK(std::abs(ndc.y) < 1.0e-6);
}

TEST_CASE("Surface camera stands on the body and looks along its heading", "[render][camera]") {
    const BodyId earth{1};
    BodyFrame frame;
    frame.position = glm::dvec3(kAu, 0.0, 0.0);
    frame.radius = 6.371e6;
    // Body rotated 90 degrees about +Z.
    frame.orientation = glm::angleAxis(1.5707963267948966, glm::dvec3(0.0, 0.0, 1.0));
    SurfaceMode surface;
    surface.body = earth;
    surface.latitude = 0.0;
    surface.longitude = 0.0;
    surface.altitude = 10.0;
    surface.heading = 0.0; // north
    surface.pitch = 0.0;
    Camera camera;
    camera.set_mode(surface);
    REQUIRE(camera.update(0.0, {}, single_body(earth, frame)));

    // Body-fixed +X becomes world +Y; the eye is 10 m above the surface there.
    CHECK(relative_error(camera.eye_offset(), glm::dvec3(0.0, 6.371e6 + 10.0, 0.0)) < 1.0e-14);
    // Looking north = world +Z at the equator; camera -Z axis.
    const glm::dvec3 forward = camera.orientation() * glm::dvec3(0.0, 0.0, -1.0);
    CHECK(forward.z == Catch::Approx(1.0).margin(1.0e-12));
    const glm::dvec3 up = camera.orientation() * glm::dvec3(0.0, 1.0, 0.0);
    CHECK(up.y == Catch::Approx(1.0).margin(1.0e-12));

    // Walking north for 1 s at 1000 m/s changes latitude by 1000 / (R + h).
    camera.set_smoothing_time(0.0);
    SurfaceMode fast = surface;
    fast.speed = 1000.0;
    camera.set_mode(fast);
    CameraInput walk;
    walk.move.z = 1.0;
    REQUIRE(camera.update(1.0, walk, single_body(earth, frame)));
    const auto& now = std::get<SurfaceMode>(camera.current());
    CHECK(now.latitude == Catch::Approx(1000.0 / (6.371e6 + 10.0)).epsilon(1.0e-12));
}

TEST_CASE("Unresolved target keeps the last view and reports failure", "[render][camera]") {
    OrbitMode orbit;
    orbit.target = BodyId{9};
    Camera camera;
    camera.set_mode(orbit);
    CHECK_FALSE(camera.update(0.0, {}, single_body(BodyId{1}, BodyFrame{})));
    CHECK(camera.mode_kind() == CameraModeKind::orbit);
}

TEST_CASE("View and projection matrices are reversible", "[render][camera]") {
    OrbitMode orbit;
    orbit.focus = kFarBody;
    orbit.distance = 25.0;
    orbit.yaw = -2.0;
    orbit.pitch = 0.9;
    Camera camera;
    camera.set_mode(orbit);
    REQUIRE(camera.update(0.0, {}, {}));
    const double aspect = 4.0 / 3.0;
    const glm::mat4 vp = camera.view_projection_matrix(aspect);
    const glm::mat4 inverse = camera.inverse_view_projection_matrix(aspect);

    const glm::dvec3 points[] = {{1.0, 2.0, -3.0}, {-4.0, 0.5, 2.0}, {0.0, 0.0, 0.5}};
    for (const glm::dvec3& offset : points) {
        const glm::dvec3 world = kFarBody + offset;
        const glm::vec3 render = camera.to_render(world);
        const glm::vec4 clip = vp * glm::vec4(render, 1.0F);
        REQUIRE(clip.w > 0.0F);
        const glm::vec4 back = inverse * clip;
        const glm::vec3 recovered = glm::vec3(back) / back.w;
        CHECK(glm::length(recovered - render) < 1.0e-4F * glm::length(render));
        // Render -> world round trip: float rounding of the camera-relative
        // position plus the spacing of absolute doubles at 1e4 AU.
        const double spacing =
            std::nextafter(kFarBody.x, std::numeric_limits<double>::infinity()) - kFarBody.x;
        CHECK(glm::length(camera.to_world(render) - world) <= 2.0 * spacing + 1.0e-5);
    }
    // The view matrix is a pure rotation: it preserves lengths.
    const glm::vec4 v = camera.view_matrix() * glm::vec4(3.0F, -4.0F, 12.0F, 0.0F);
    CHECK(glm::length(glm::vec3(v)) == Catch::Approx(13.0).epsilon(1.0e-6));
}

TEST_CASE("Smoothing does not depend on how time is split into updates", "[render][camera]") {
    const auto run = [](int steps) {
        OrbitMode orbit;
        orbit.distance = 100.0;
        Camera camera;
        camera.set_mode(orbit);
        camera.update(0.0, {}, {});
        CameraInput input;
        input.yaw = 1.0;
        input.pitch = 0.3;
        input.zoom = 0.5;
        for (int i = 0; i < steps; ++i) {
            camera.update(1.0 / steps, input, {});
        }
        // Then let it settle partially with no input.
        for (int i = 0; i < steps; ++i) {
            camera.update(0.25 / steps, {}, {});
        }
        return std::get<OrbitMode>(camera.current());
    };
    const OrbitMode coarse = run(30);
    const OrbitMode fine = run(240);
    CHECK(coarse.yaw == Catch::Approx(fine.yaw).epsilon(1.0e-9));
    CHECK(coarse.pitch == Catch::Approx(fine.pitch).epsilon(1.0e-9));
    CHECK(coarse.distance == Catch::Approx(fine.distance).epsilon(1.0e-9));
    // The view lags the goal but moves towards it.
    CHECK(coarse.yaw > 0.5);
    CHECK(coarse.yaw < 1.0);
}

TEST_CASE("Logarithmic depth is monotonic and resolves 1e-5 relative steps to 1e4 AU",
          "[render][camera][log_depth]") {
    const double far = kDefaultLogDepthFar;
    CHECK(log_depth(0.0, far) == 0.0);
    CHECK(log_depth(far, far) == Catch::Approx(1.0).epsilon(1.0e-15));
    for (const double w : {0.1, 1.0, 1.0e3, 6.4e6, kAu, 1.0e4 * kAu}) {
        CHECK(log_depth_inverse(log_depth(w, far), far) == Catch::Approx(w).epsilon(1.0e-12));
        // Two surfaces 1e-5 apart (relative) get different float32 depths.
        const auto near_depth = static_cast<float>(log_depth(w, far));
        const auto far_depth = static_cast<float>(log_depth(w * (1.0 + 1.0e-5), far));
        CHECK(far_depth > near_depth);
        const auto float_epsilon = static_cast<double>(std::numeric_limits<float>::epsilon());
        CHECK(log_depth_relative_resolution(w, far, float_epsilon) < 1.0e-5);
    }
}

namespace {

constexpr double kPitchLimit = 1.5707963267948966 - 1.0e-3;
constexpr double kDefaultTau = 0.15;

// Closed-form first-order lag dc/dt = (g - c) / tau of a goal that starts at
// the view (c(0) = g(0) = start), ramps at `rate` and saturates at `limit`.
double lagged_saturating_ramp(double start, double rate, double limit, double tau, double t) {
    const double saturation = (limit - start) / rate;
    const auto ramp = [&](double s) { return start + rate * (s + tau * std::expm1(-s / tau)); };
    if (t <= saturation) {
        return ramp(t);
    }
    return limit + (ramp(saturation) - limit) * std::exp(-(t - saturation) / tau);
}

double quaternion_distance(const glm::dquat& a, const glm::dquat& b) {
    const glm::dquat sum = a + b;
    const glm::dquat difference = a - b;
    return std::min(std::sqrt(glm::dot(sum, sum)), std::sqrt(glm::dot(difference, difference)));
}

bool all_finite(const glm::mat4& m) {
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            if (!std::isfinite(m[c][r])) {
                return false;
            }
        }
    }
    return true;
}

// Applies `input` for 1 s in `steps` equal updates, then lets the view settle
// for 5 s without input (removing the smoothing lag, so that the comparison
// isolates how the goal was integrated). Returns (goal, view).
template <typename Mode>
std::pair<Mode, Mode> run_split(Camera camera, const CameraInput& input, int steps,
                                const BodyFrameLookup& lookup) {
    for (int i = 0; i < steps; ++i) {
        camera.update(1.0 / steps, input, lookup);
    }
    for (int i = 0; i < steps; ++i) {
        camera.update(5.0 / steps, {}, lookup);
    }
    return {std::get<Mode>(camera.goal()), std::get<Mode>(camera.current())};
}

} // namespace

TEST_CASE("Free flight integrates simultaneous move and zoom for any update split",
          "[render][camera][timestep]") {
    // Speed decays as exp(-t) while moving forward: the distance after 1 s is
    // 1 - exp(-1) = 0.632120559 m whatever the number of updates.
    const double expected = -std::expm1(-1.0);
    for (const int steps : {1, 2, 30, 240}) {
        Camera camera;
        camera.set_smoothing_time(0.0);
        camera.set_mode(FreeMode{});
        REQUIRE(camera.update(0.0, {}, {}));
        CameraInput input;
        input.move.z = 1.0;
        input.zoom = 1.0;
        for (int i = 0; i < steps; ++i) {
            camera.update(1.0 / steps, input, {});
        }
        const auto& mode = std::get<FreeMode>(camera.current());
        INFO(steps << " updates: travelled " << -mode.position.z << " m");
        CHECK(std::abs(-mode.position.z - expected) < 1.0e-12);
        CHECK(std::abs(mode.position.x) < 1.0e-15);
        CHECK(std::abs(mode.position.y) < 1.0e-15);
        CHECK(mode.speed == Catch::Approx(std::exp(-1.0)).epsilon(1.0e-13));
    }
}

TEST_CASE("Free flight follows the exact arc while turning", "[render][camera][timestep]") {
    // Forward at 1 m/s while yawing left at 1 rad/s: the forward direction is
    // (-sin t, 0, -cos t), so after 1 s the eye is at (cos 1 - 1, 0, -sin 1).
    const glm::dvec3 expected{std::cos(1.0) - 1.0, 0.0, -std::sin(1.0)};
    for (const int steps : {1, 3, 30, 240}) {
        Camera camera;
        camera.set_smoothing_time(0.0);
        camera.set_mode(FreeMode{});
        REQUIRE(camera.update(0.0, {}, {}));
        CameraInput input;
        input.move.z = 1.0;
        input.yaw = 1.0;
        for (int i = 0; i < steps; ++i) {
            camera.update(1.0 / steps, input, {});
        }
        INFO(steps << " updates");
        CHECK(glm::length(std::get<FreeMode>(camera.current()).position - expected) < 1.0e-12);
    }
}

TEST_CASE("Free flight with combined input settles to the same pose for any update split",
          "[render][camera][timestep]") {
    FreeMode start;
    start.position = glm::dvec3(3.0, -2.0, 1.0);
    start.orientation = glm::angleAxis(0.4, glm::normalize(glm::dvec3(1.0, -2.0, 0.5)));
    start.speed = 2.0;
    Camera camera;
    camera.set_mode(start);
    REQUIRE(camera.update(0.0, {}, {}));
    CameraInput input;
    input.move = glm::dvec3(0.5, -0.25, 1.0);
    input.yaw = 0.8;
    input.pitch = 0.3;
    input.roll = 0.2;
    input.zoom = 0.4;
    const auto reference = run_split<FreeMode>(camera, input, 240, {});
    for (const int steps : {1, 30, 60}) {
        const auto result = run_split<FreeMode>(camera, input, steps, {});
        INFO(steps << " updates per second");
        CHECK(glm::length(result.first.position - reference.first.position) < 1.0e-9);
        CHECK(quaternion_distance(result.first.orientation, reference.first.orientation) < 1.0e-9);
        CHECK(glm::length(result.second.position - reference.second.position) < 1.0e-9);
        CHECK(quaternion_distance(result.second.orientation, reference.second.orientation) <
              1.0e-9);
        CHECK(result.first.speed == Catch::Approx(reference.first.speed).epsilon(1.0e-12));
    }
}

TEST_CASE("Surface motion with combined input settles to the same place for any update split",
          "[render][camera][timestep]") {
    const BodyId moon{4};
    BodyFrame frame;
    frame.position = glm::dvec3(1.0e9, 2.0e8, -3.0e7);
    frame.radius = 1000.0; // small body: curvature matters within a second
    frame.orientation = glm::angleAxis(0.3, glm::normalize(glm::dvec3(1.0, 1.0, 1.0)));
    const BodyFrameLookup lookup = single_body(moon, frame);
    SurfaceMode start;
    start.body = moon;
    start.latitude = 0.2;
    start.longitude = -0.4;
    start.altitude = 5.0;
    start.heading = 0.3;
    start.speed = 40.0;
    Camera camera;
    camera.set_mode(start);
    REQUIRE(camera.update(0.0, {}, lookup));

    CameraInput diagonal;
    diagonal.move = glm::dvec3(0.6, 0.3, 0.8);
    diagonal.yaw = 0.5;
    diagonal.zoom = 0.3;
    // Descending while pitching up: altitude reaches 0 and pitch its limit
    // part-way through the second.
    CameraInput descent;
    descent.move = glm::dvec3(-0.4, -1.0, 1.0);
    descent.yaw = -0.4;
    descent.pitch = 2.0;
    for (const CameraInput& input : {diagonal, descent}) {
        const auto reference = run_split<SurfaceMode>(camera, input, 240, lookup);
        for (const int steps : {1, 30, 60}) {
            const auto result = run_split<SurfaceMode>(camera, input, steps, lookup);
            INFO(steps << " updates per second, move.y " << input.move.y);
            for (const auto& [actual, expected] : {std::pair{result.first, reference.first},
                                                   std::pair{result.second, reference.second}}) {
                CHECK(std::abs(actual.latitude - expected.latitude) < 1.0e-9);
                CHECK(std::abs(actual.longitude - expected.longitude) < 1.0e-9);
                CHECK(std::abs(actual.altitude - expected.altitude) < 1.0e-9);
                CHECK(std::abs(actual.heading - expected.heading) < 1.0e-12);
                CHECK(std::abs(actual.pitch - expected.pitch) < 1.0e-12);
            }
        }
        if (input.move.y < 0.0) {
            CHECK(reference.first.altitude == 0.0);
            CHECK(reference.first.pitch == Catch::Approx(kPitchLimit).epsilon(1.0e-15));
        }
    }
}

TEST_CASE("Orbit pitch and zoom saturation smooth exactly for any update split",
          "[render][camera][timestep]") {
    OrbitMode start;
    start.pitch = 0.0;
    start.distance = 1.0;
    // Pitch at 2 rad/s reaches its limit after 0.785 s; zoom at 10 e-folds/s
    // reaches the 1 mm minimum distance after 0.691 s. The smoothed view is
    // the exact lag of these saturating ramps (default time constant 0.15 s).
    const double expected_pitch = lagged_saturating_ramp(0.0, 2.0, kPitchLimit, kDefaultTau, 1.0);
    const double expected_distance =
        std::exp(lagged_saturating_ramp(0.0, -10.0, std::log(1.0e-3), kDefaultTau, 1.0));
    const double expected_yaw = lagged_saturating_ramp(0.0, 0.7, 1.0e9, kDefaultTau, 1.0);
    for (const int steps : {1, 2, 30, 240}) {
        Camera camera;
        camera.set_mode(start);
        REQUIRE(camera.update(0.0, {}, {}));
        CameraInput input;
        input.pitch = 2.0;
        input.zoom = 10.0;
        input.yaw = 0.7;
        for (int i = 0; i < steps; ++i) {
            camera.update(1.0 / steps, input, {});
        }
        const auto& view = std::get<OrbitMode>(camera.current());
        INFO(steps << " updates: pitch " << view.pitch << " distance " << view.distance);
        CHECK(std::abs(view.pitch - expected_pitch) < 1.0e-10);
        CHECK(std::abs(view.distance / expected_distance - 1.0) < 1.0e-10);
        CHECK(std::abs(view.yaw - expected_yaw) < 1.0e-10);
    }
}

TEST_CASE("Chase pitch and zoom saturation smooth exactly for any update split",
          "[render][camera][timestep]") {
    const BodyId ship{5};
    BodyFrame frame;
    frame.position = kFarBody;
    frame.velocity = glm::dvec3(0.0, 3.0e4, 0.0);
    const BodyFrameLookup lookup = single_body(ship, frame);
    ChaseMode start;
    start.target = ship;
    start.distance = 50.0;
    start.pitch = 0.0;
    const double expected_pitch = lagged_saturating_ramp(0.0, -2.5, -kPitchLimit, kDefaultTau, 1.0);
    const double expected_distance =
        std::exp(lagged_saturating_ramp(std::log(50.0), -16.0, std::log(1.0e-3), kDefaultTau, 1.0));
    for (const int steps : {1, 30, 240}) {
        Camera camera;
        camera.set_mode(start);
        REQUIRE(camera.update(0.0, {}, lookup));
        CameraInput input;
        input.pitch = -2.5;
        input.zoom = 16.0;
        for (int i = 0; i < steps; ++i) {
            REQUIRE(camera.update(1.0 / steps, input, lookup));
        }
        const auto& view = std::get<ChaseMode>(camera.current());
        INFO(steps << " updates: pitch " << view.pitch << " distance " << view.distance);
        CHECK(std::abs(view.pitch - expected_pitch) < 1.0e-10);
        CHECK(std::abs(view.distance / expected_distance - 1.0) < 1.0e-10);
    }
}

TEST_CASE("Invalid aspect and lens values never produce non-finite projections",
          "[render][camera][lens]") {
    OrbitMode orbit;
    orbit.distance = 10.0;
    orbit.yaw = 0.4;
    Camera camera;
    camera.set_mode(orbit);
    REQUIRE(camera.update(0.0, {}, {}));
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    // Invalid aspect ratios (a minimised window has a zero-width framebuffer)
    // fall back to aspect 1.
    const glm::mat4 fallback = camera.projection_matrix(1.0);
    for (const double aspect : {0.0, -0.0, -1.5, nan, inf, -inf}) {
        INFO("aspect " << aspect);
        CHECK(all_finite(camera.projection_matrix(aspect)));
        CHECK(all_finite(camera.view_projection_matrix(aspect)));
        CHECK(all_finite(camera.inverse_view_projection_matrix(aspect)));
        CHECK(camera.projection_matrix(aspect) == fallback);
    }
    for (const double aspect : {1.0e-300, 1.0e300}) {
        INFO("aspect " << aspect);
        CHECK(all_finite(camera.projection_matrix(aspect)));
        CHECK(all_finite(camera.inverse_view_projection_matrix(aspect)));
    }

    // Invalid lenses are rejected and the previous lens stays in use.
    const CameraLens good = camera.lens();
    std::vector<CameraLens> invalid;
    for (const double near : {0.0, -1.0, 1.0e-300, nan, inf}) {
        CameraLens lens = good;
        lens.near_plane = near;
        invalid.push_back(lens);
    }
    for (const double fov : {0.0, -0.5, 3.141592653589793, 4.0, 1.0e-300, nan, inf}) {
        CameraLens lens = good;
        lens.vertical_fov = fov;
        invalid.push_back(lens);
    }
    for (const double far : {0.0, -1.0, 0.01, nan, inf}) {
        CameraLens lens = good;
        lens.log_depth_far = far;
        invalid.push_back(lens);
    }
    for (const CameraLens& lens : invalid) {
        INFO("fov " << lens.vertical_fov << " near " << lens.near_plane << " far "
                    << lens.log_depth_far);
        const auto rejected = camera.set_lens(lens);
        REQUIRE_FALSE(rejected.has_value());
        CHECK(rejected.error().find("camera lens") != std::string::npos);
        CHECK_FALSE(Camera::validate_lens(lens).has_value());
        CHECK(camera.lens().vertical_fov == good.vertical_fov);
        CHECK(camera.lens().near_plane == good.near_plane);
        CHECK(camera.lens().log_depth_far == good.log_depth_far);
        CHECK(all_finite(camera.projection_matrix(1.5)));
        CHECK(all_finite(camera.inverse_view_projection_matrix(1.5)));
        CHECK(std::isfinite(log_depth_scale(camera.lens().log_depth_far)));
    }

    // A valid lens is accepted and the projection still round-trips.
    CameraLens custom;
    custom.vertical_fov = 1.2;
    custom.near_plane = 0.5;
    custom.log_depth_far = 1.0e12;
    REQUIRE(camera.set_lens(custom).has_value());
    CHECK(camera.lens().vertical_fov == custom.vertical_fov);
    CHECK(camera.lens().near_plane == custom.near_plane);
    CHECK(camera.lens().log_depth_far == custom.log_depth_far);
    const glm::mat4 vp = camera.view_projection_matrix(0.75);
    const glm::mat4 inverse = camera.inverse_view_projection_matrix(0.75);
    const glm::vec3 render = camera.to_render(glm::dvec3(1.0, 2.0, -3.0));
    const glm::vec4 clip = vp * glm::vec4(render, 1.0F);
    REQUIRE(clip.w > 0.0F);
    const glm::vec4 back = inverse * clip;
    CHECK(glm::length(glm::vec3(back) / back.w - render) < 1.0e-4F * glm::length(render));
}
