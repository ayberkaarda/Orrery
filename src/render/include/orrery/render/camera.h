#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <variant>

#include <glm/ext/matrix_float4x4.hpp>
#include <glm/ext/quaternion_double.hpp>
#include <glm/ext/vector_double3.hpp>
#include <glm/ext/vector_float3.hpp>

#include "orrery/core/body.h"
#include "orrery/core/result.h"
#include "orrery/render/log_depth.h"

// Camera for the space workspace (docs/rendering.md, section "Camera").
//
// All camera state and math are double precision and independent of OpenGL.
// World space is the physics frame (ecliptic J2000, +Z north) in metres.
// Render space is camera-relative: the camera eye is the origin and the axes
// are the world axes; positions are formed in double and only then cast to
// float (floating origin). The view matrix therefore holds only the camera
// rotation, and the projection is an infinite-far perspective whose depth is
// replaced by logarithmic depth in the fragment stage.
//
// Precision: the eye is stored as an anchor point plus an offset. In Orbit,
// Chase and Surface modes the anchor is the target body position, so a
// body's render position is (body - anchor) - offset: exact for the target
// itself and for nearby bodies (Sterbenz), independent of how far from the
// world origin the scene is. In Free mode the anchor is the eye position
// itself, whose resolution is the double spacing of the world coordinate
// (about 0.25 m at 1e4 AU).
namespace orrery::render {

// Pose of a body as seen by the camera, in world space.
struct BodyFrame {
    glm::dvec3 position{0.0};                   // [m]
    glm::dvec3 velocity{0.0};                   // [m/s], used by Chase
    glm::dquat orientation{1.0, 0.0, 0.0, 0.0}; // body-fixed -> world rotation (w, x, y, z)
    double radius = 0.0;                        // mean radius [m], used by Surface
};

// Resolves a body id to its current frame; std::nullopt if the body does not
// exist (for example after a merge).
using BodyFrameLookup = std::function<std::optional<BodyFrame>(core::BodyId)>;

// Free flight: 6-DoF eye in world space.
struct FreeMode {
    glm::dvec3 position{0.0};                   // eye [m]
    glm::dquat orientation{1.0, 0.0, 0.0, 0.0}; // camera -> world; the camera looks along -Z
    double speed = 1.0;                         // [m/s] at full translation input
};

// Orbit around a body (or a fixed world point when `target` is empty).
// The eye sits at focus + distance * (cos p cos y, cos p sin y, sin p) and
// looks at the focus with world +Z up.
struct OrbitMode {
    std::optional<core::BodyId> target;
    glm::dvec3 focus{0.0};  // [m], used when target is empty
    double distance = 10.0; // [m]
    double yaw = 0.0;       // azimuth from world +X towards +Y [rad]
    double pitch = 0.3;     // elevation above the world XY plane [rad]
};

// Chase: follows a body from an offset expressed in its travel frame
// (forward = velocity direction, up = world +Z made orthogonal). yaw = 0,
// pitch = 0 is directly behind the body. The frame turns smoothly when the
// velocity direction changes; with zero velocity the world axes are used.
struct ChaseMode {
    core::BodyId target;
    double distance = 1000.0; // [m]
    double yaw = 0.0;         // around the frame up axis [rad]
    double pitch = 0.2;       // above the frame forward/right plane [rad]
};

// Surface: standing on a body (sphere of BodyFrame::radius) in its body-fixed
// frame, looking along a local heading.
struct SurfaceMode {
    core::BodyId body;
    double latitude = 0.0;  // [rad], clamped to +/- (pi/2 - 1e-6)
    double longitude = 0.0; // [rad], body-fixed, from body +X towards +Y
    double altitude = 2.0;  // above the radius [m], >= 0
    double heading = 0.0;   // from local north towards east [rad]
    double pitch = 0.0;     // above the local horizon [rad]
    double speed = 10.0;    // [m/s] at full translation input
};

// Turntable for the aero workspace (spec 7.1): orbits a fixed world point
// like OrbitMode without a target body, and in addition turns on its own at
// `spin_rate` (added to the yaw input) and pans. The eye sits at
// focus + distance * (cos p cos y, cos p sin y, sin p) and looks at the focus
// with world +Z up (the body axes of orrery_aero: x downstream, z up).
// Panning moves the focus at `distance` per second for a unit input: move.x
// along the horizontal camera right (-sin y, cos y, 0), move.y along world
// +Z; move.z is ignored.
struct TurntableMode {
    glm::dvec3 focus{0.0};  // [m]
    double distance = 5.0;  // [m]
    double yaw = 0.6;       // azimuth from world +X towards +Y [rad]
    double pitch = 0.35;    // elevation above the world XY plane [rad]
    double spin_rate = 0.0; // automatic yaw rate [rad/s], 0 = still
};

using CameraMode = std::variant<FreeMode, OrbitMode, ChaseMode, SurfaceMode, TurntableMode>;

enum class CameraModeKind : std::uint8_t { free, orbit, chase, surface, turntable };

// Rate input for one update: every component is a rate per second, held
// constant over the update and integrated simultaneously (see update()).
//   move:  x right, y up, z forward, each in [-1, 1] (Free: camera axes at
//          `speed`; Surface: x right of the heading, z along the heading,
//          y altitude; Turntable: x and y pan, see TurntableMode).
//   yaw, pitch, roll: [rad/s] (positive yaw turns left, positive pitch looks
//          up; roll is used only by Free; Free turns about the fixed camera
//          axis (pitch, yaw, -roll)).
//   zoom:  e-folds per second; positive moves closer (Orbit, Chase,
//          Turntable) or slows translation down (Free, Surface:
//          speed(t) = speed * exp(-zoom t)).
struct CameraInput {
    glm::dvec3 move{0.0};
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    double zoom = 0.0;
};

// Accepted values (Camera::validate_lens), chosen so that every projection
// matrix and its inverse stay finite in float:
//   vertical_fov   in [1e-6, pi - 1e-6] rad
//   near_plane     in [1e-9, 1e12] m
//   log_depth_far  finite and greater than near_plane [m]
struct CameraLens {
    double vertical_fov = 0.8726646259971648; // 50 degrees [rad]
    double near_plane = 0.05;                 // [m]
    double log_depth_far = kDefaultLogDepthFar;
};

class Camera {
public:
    Camera() = default;

    // Switches mode. The new mode is applied without smoothing at the next
    // update(); call update(0, {}, lookup) to place the camera immediately.
    void set_mode(const CameraMode& mode);
    [[nodiscard]] CameraModeKind mode_kind() const noexcept;
    // Mode parameters after input (the smoothing target).
    [[nodiscard]] const CameraMode& goal() const noexcept { return goal_; }
    // Smoothed mode parameters that define the current view.
    [[nodiscard]] const CameraMode& current() const noexcept { return current_; }

    // Time constant of the exponential smoothing [s]; 0 disables smoothing.
    void set_smoothing_time(double seconds) noexcept;
    [[nodiscard]] double smoothing_time() const noexcept { return smoothing_time_; }

    // Fails with a message naming the offending value when `lens` is outside
    // the accepted ranges (see CameraLens).
    [[nodiscard]] static core::Result<std::monostate, std::string>
    validate_lens(const CameraLens& lens);
    // Applies a valid lens; an invalid one is rejected with the
    // validate_lens() message and the previous lens stays in use.
    [[nodiscard]] core::Result<std::monostate, std::string> set_lens(const CameraLens& lens);
    [[nodiscard]] const CameraLens& lens() const noexcept { return lens_; }

    // Applies `input` over `dt` seconds (negative or NaN: 0), advances the
    // smoothing and resolves the target body. Returns false if a target body
    // could not be resolved; the camera then keeps its last anchor and offset.
    //
    // Time-step contract (the same input over the same total time gives the
    // same result however it is split into updates):
    //   - Goals are integrated exactly for constant rates: Free position
    //     along the turning, decelerating path (closed form), Orbit/Chase/
    //     Turntable yaw, pitch and log distance, Turntable pan along the
    //     turning right axis (closed form), Surface heading, pitch, speed and
    //     altitude, and Surface ground motion in the local tangent plane.
    //   - Clamps (pitch limit, minimum distance, zero altitude) split the
    //     update at the moment they are reached, so the smoothing sees the
    //     exact piecewise-linear goal.
    //   - Free and Surface goals are advanced in internal substeps of at most
    //     1/240 s (kept below 10000 per update); Surface converts tangent
    //     motion to latitude/longitude with the midpoint rule per substep, and
    //     the smoothing of their curved goal paths is exact per substep. Splits
    //     into multiples of 1/240 s reproduce the same substeps; other splits
    //     differ by the second-order error of these substeps.
    bool update(double dt, const CameraInput& input, const BodyFrameLookup& lookup);

    // World position of the eye (anchor + offset). Informational: rounding of
    // the sum can lose the sub-metre part far from the world origin.
    [[nodiscard]] glm::dvec3 eye_position() const noexcept { return anchor_ + eye_offset_; }
    [[nodiscard]] const glm::dvec3& anchor() const noexcept { return anchor_; }
    [[nodiscard]] const glm::dvec3& eye_offset() const noexcept { return eye_offset_; }
    // Camera -> world rotation (camera looks along -Z, +Y up).
    [[nodiscard]] const glm::dquat& orientation() const noexcept { return orientation_; }

    // Floating origin: world position [m] -> camera-relative render space,
    // computed in double and cast to float at the end.
    [[nodiscard]] glm::dvec3 to_render_double(const glm::dvec3& world) const noexcept;
    [[nodiscard]] glm::vec3 to_render(const glm::dvec3& world) const noexcept;
    // Inverse of to_render (for picking).
    [[nodiscard]] glm::dvec3 to_world(const glm::vec3& render) const noexcept;

    // Render space -> view space (rotation only; the eye is the origin).
    [[nodiscard]] glm::mat4 view_matrix() const noexcept;
    // View -> clip, infinite far plane; depth is replaced by log depth.
    // `aspect` (width / height) that is not a positive finite number, as from
    // a minimised window, is replaced by 1; positive values are clamped to
    // [1e-6, 1e6]. The matrices are always finite.
    [[nodiscard]] static double usable_aspect(double aspect) noexcept;
    [[nodiscard]] glm::mat4 projection_matrix(double aspect) const noexcept;
    [[nodiscard]] glm::mat4 view_projection_matrix(double aspect) const noexcept;
    [[nodiscard]] glm::mat4 inverse_view_projection_matrix(double aspect) const noexcept;

private:
    CameraMode goal_{FreeMode{}};
    CameraMode current_{FreeMode{}};
    bool snap_ = true;
    double smoothing_time_ = 0.15;
    CameraLens lens_{};
    glm::dvec3 anchor_{0.0};
    glm::dvec3 eye_offset_{0.0};
    glm::dquat orientation_{1.0, 0.0, 0.0, 0.0};
    // Smoothed chase travel frame (frame -> world).
    std::optional<glm::dquat> chase_frame_;
};

} // namespace orrery::render
