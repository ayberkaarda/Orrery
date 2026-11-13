#include "orrery/render/camera.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>
#include <type_traits>

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_double3x3.hpp>
#include <glm/ext/matrix_double4x4.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/matrix.hpp>

namespace orrery::render {

namespace {

constexpr double kHalfPi = 1.5707963267948966;
// Keeps the view direction away from the up axis (look_rotation is singular there).
constexpr double kPitchLimit = kHalfPi - 1.0e-3;
constexpr double kLatitudeLimit = kHalfPi - 1.0e-6;
constexpr double kMinDistance = 1.0e-3; // [m]
constexpr double kWorldUpZ = 1.0;       // world up is +Z
constexpr double kNever = std::numeric_limits<double>::infinity();
// Longest internal step for Free and Surface goals [s] and the most steps
// one update takes (a longer update gets longer steps).
constexpr double kSubstep = 1.0 / 240.0;
constexpr int kMaxSubsteps = 10000;
// Projection limits (CameraLens, Camera::usable_aspect): they keep every
// projection matrix and its inverse finite in float.
constexpr double kMinFov = 1.0e-6;                     // [rad]
constexpr double kMaxFov = 3.141592653589793 - 1.0e-6; // [rad]
constexpr double kMinNearPlane = 1.0e-9;               // [m]
constexpr double kMaxNearPlane = 1.0e12;               // [m]
constexpr double kMinAspect = 1.0e-6;
constexpr double kMaxAspect = 1.0e6;

// Exact discretisation of the first-order smoothing dc/dt = (g(t) - c) / tau
// over one update of length dt, with the goal g moving linearly from g0 to g1
// during the update (constant input rates):
//   c1 = e c0 + k g0 + w (g1 - g0),  e = exp(-dt/tau), k = 1 - e,
//   w = 1 - tau k / dt.
// Because the formula is exact for piecewise-linear goals, splitting a time
// span into more or fewer updates gives the same result.
struct Blend {
    double e = 0.0;
    double k = 1.0;
    double w = 1.0;
};

Blend make_blend(double dt, double tau) {
    if (tau <= 0.0) {
        return {0.0, 1.0, 1.0};
    }
    if (dt <= 0.0) {
        return {1.0, 0.0, 0.0};
    }
    const double k = -std::expm1(-dt / tau);
    return {1.0 - k, k, 1.0 - tau * k / dt};
}

double blend(double c0, double g0, double g1, const Blend& b) {
    return b.e * c0 + b.k * g0 + b.w * (g1 - g0);
}

glm::dvec3 blend(const glm::dvec3& c0, const glm::dvec3& g0, const glm::dvec3& g1, const Blend& b) {
    return b.e * c0 + b.k * g0 + b.w * (g1 - g0);
}

// Distances are smoothed in log space so that zooming feels uniform across
// orders of magnitude (and constant zoom input is a linear ramp there).
double log_blend(double c0, double g0, double g1, const Blend& b) {
    return std::exp(blend(std::log(c0), std::log(g0), std::log(g1), b));
}

// Rotation analogue for goals that change by local (right-multiplied)
// rotations: approach g0 with weight k, then apply the fraction w of the
// increment g0^-1 g1.
// APPROXIMATION: rotation smoothing - slerp-based analogue of the exact
// scalar blend - exact when the lag and the increment share an axis, otherwise
// differs at second order in the per-update rotation - camera feel only.
glm::dquat blend(const glm::dquat& c0, const glm::dquat& g0, const glm::dquat& g1, const Blend& b) {
    const glm::dquat approach = glm::slerp(c0, g0, b.k);
    const glm::dquat increment = glm::normalize(glm::inverse(g0) * g1);
    const glm::dquat partial =
        glm::slerp(glm::dquat(1.0, 0.0, 0.0, 0.0), increment, std::clamp(b.w, 0.0, 1.0));
    return glm::normalize(approach * partial);
}
// Camera -> world rotation for a camera looking along `forward` with `up` as
// the approximate up direction (camera -Z = forward, +Y = up).
glm::dquat look_rotation(const glm::dvec3& forward, const glm::dvec3& up) {
    const glm::dvec3 f = glm::normalize(forward);
    const glm::dvec3 r = glm::normalize(glm::cross(f, up));
    const glm::dvec3 u = glm::cross(r, f);
    return glm::normalize(glm::quat_cast(glm::dmat3(r, u, -f)));
}

// Chase travel frame (frame -> world): columns forward, left, up.
glm::dquat travel_frame(const glm::dvec3& velocity) {
    const double speed = glm::length(velocity);
    const glm::dvec3 forward = speed > 0.0 ? velocity / speed : glm::dvec3(1.0, 0.0, 0.0);
    glm::dvec3 up_reference = glm::dvec3(0.0, 0.0, kWorldUpZ);
    if (glm::length(glm::cross(forward, up_reference)) < 1.0e-6) {
        up_reference = glm::dvec3(0.0, 1.0, 0.0);
    }
    const glm::dvec3 left = glm::normalize(glm::cross(up_reference, forward));
    const glm::dvec3 up = glm::cross(forward, left);
    return glm::normalize(glm::quat_cast(glm::dmat3(forward, left, up)));
}

// Infinite-far perspective for a validated lens and any aspect.
glm::dmat4 projection(const CameraLens& lens, double aspect) {
    return glm::infinitePerspective(lens.vertical_fov, Camera::usable_aspect(aspect),
                                    lens.near_plane);
}

glm::dvec3 spherical(double yaw, double pitch) {
    return {std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch)};
}

struct SurfaceBasis {
    glm::dvec3 up;
    glm::dvec3 east;
    glm::dvec3 north;
};

// Local basis at (latitude, longitude) in the body-fixed frame.
SurfaceBasis surface_basis(double latitude, double longitude) {
    const glm::dvec3 up{std::cos(latitude) * std::cos(longitude),
                        std::cos(latitude) * std::sin(longitude), std::sin(latitude)};
    const glm::dvec3 east{-std::sin(longitude), std::cos(longitude), 0.0};
    return {up, east, glm::cross(up, east)};
}

std::optional<core::BodyId> target_of(const CameraMode& mode) {
    return std::visit(
        [](const auto& m) -> std::optional<core::BodyId> {
            using T = std::decay_t<decltype(m)>;
            if constexpr (std::is_same_v<T, OrbitMode> || std::is_same_v<T, ChaseMode>) {
                return m.target;
            } else if constexpr (std::is_same_v<T, SurfaceMode>) {
                return m.body;
            } else {
                return std::nullopt;
            }
        },
        mode);
}

// Integral of exp(c s) over s in [0, t] for the complex rate c = a + i b,
// evaluated without cancellation when |c t| is small.
std::complex<double> exp_integral(double a, double b, double t) {
    if (t <= 0.0) {
        return {0.0, 0.0};
    }
    const std::complex<double> x(a * t, b * t);
    if (std::abs(x) < 1.0e-6) {
        // (e^x - 1) / x = 1 + x/2 + x^2/6 + x^3/24 + O(x^4)
        return t * (1.0 + x * (0.5 + x * (1.0 / 6.0 + x / 24.0)));
    }
    // e^x - 1 = (expm1(re) cos(im) - 2 sin^2(im / 2)) + i e^re sin(im)
    const double half = std::sin(0.5 * x.imag());
    const std::complex<double> exp_minus_one(std::expm1(x.real()) * std::cos(x.imag()) -
                                                 2.0 * half * half,
                                             std::exp(x.real()) * std::sin(x.imag()));
    return t * exp_minus_one / x;
}

// Time until value + rate t reaches the bound it moves towards; infinity if it
// never does (no motion, or already at or beyond that bound).
double time_to_bound(double value, double rate, double lower, double upper) {
    if (rate > 0.0 && value < upper) {
        return (upper - value) / rate;
    }
    if (rate < 0.0 && value > lower) {
        return (lower - value) / rate;
    }
    return kNever;
}

// Time until distance exp(-zoom t) reaches kMinDistance.
double time_to_min_distance(double distance, double zoom) {
    if (zoom > 0.0 && distance > kMinDistance) {
        return std::log(distance / kMinDistance) / zoom;
    }
    return kNever;
}

// Time until altitude + climb * integral_0^t exp(-zoom s) ds reaches 0.
double time_to_ground(double altitude, double climb, double zoom) {
    if (!(climb < 0.0) || !(altitude > 0.0)) {
        return kNever;
    }
    const double needed = altitude / -climb; // value of the integral at touchdown
    const double x = zoom * needed;
    if (std::abs(x) < 1.0e-12) {
        return needed;
    }
    if (x >= 1.0) {
        return kNever; // the speed decays before the ground is reached
    }
    return -std::log1p(-x) / zoom;
}

// Free flight with constant rates: the orientation turns about the fixed
// camera-frame axis omega = (pitch, yaw, -roll), q(t) = q0 exp(omega t), and
// the speed decays as s0 exp(-zoom t), so the displacement
//   integral_0^dt s0 exp(-zoom t) q(t) v dt
// has a closed form (v split along and across the turning axis).
void advance(FreeMode& mode, const CameraInput& input, double dt) {
    const glm::dvec3 omega{input.pitch, input.yaw, -input.roll};
    const double rate = glm::length(omega);
    const glm::dvec3 v{input.move.x, input.move.y, -input.move.z};
    const double straight = exp_integral(-input.zoom, 0.0, dt).real();
    glm::dvec3 local = v * straight;
    glm::dquat turn(1.0, 0.0, 0.0, 0.0);
    if (rate > 0.0) {
        const glm::dvec3 axis = omega / rate;
        const glm::dvec3 along = axis * glm::dot(axis, v);
        const glm::dvec3 across = v - along;
        const std::complex<double> turning = exp_integral(-input.zoom, rate, dt);
        local =
            along * straight + across * turning.real() + glm::cross(axis, across) * turning.imag();
        turn = glm::angleAxis(rate * dt, axis);
    }
    mode.position += (mode.orientation * local) * mode.speed;
    mode.orientation = glm::normalize(mode.orientation * turn);
    mode.speed *= std::exp(-input.zoom * dt);
}

// Orbit and Chase: yaw, pitch and log distance are linear in time until a
// clamp is reached (update() splits the step there).
template <typename Mode> void advance_angles(Mode& mode, const CameraInput& input, double dt) {
    mode.yaw += input.yaw * dt;
    mode.pitch = std::clamp(mode.pitch + input.pitch * dt, -kPitchLimit, kPitchLimit);
    mode.distance = std::max(mode.distance * std::exp(-input.zoom * dt), kMinDistance);
}

// Turntable: yaw turns at the input rate plus the spin rate, pitch and log
// distance as for Orbit. The pan moves the focus at `distance` per second
// along the horizontal right axis (-sin y, cos y, 0) = i e^{i y} (as x + i y)
// and world +Z; with y(t) = y0 + w t and distance d0 exp(-zoom t) the
// displacement is closed form:
//   right: d0 move.x i e^{i y0} integral exp((-zoom + i w) t) dt
//   up:    d0 move.y integral exp(-zoom t) dt
// At the minimum distance the distance no longer shrinks (update() splits
// the step where it is reached), so the pan uses zoom 0 there.
void advance(TurntableMode& mode, const CameraInput& input, double dt) {
    const double rate = input.yaw + mode.spin_rate;
    const double zoom = (input.zoom > 0.0 && mode.distance <= kMinDistance) ? 0.0 : input.zoom;
    const double d0 = mode.distance;
    const std::complex<double> right =
        std::complex<double>(0.0, 1.0) * std::polar(1.0, mode.yaw) * exp_integral(-zoom, rate, dt);
    mode.focus.x += d0 * input.move.x * right.real();
    mode.focus.y += d0 * input.move.x * right.imag();
    mode.focus.z += d0 * input.move.y * exp_integral(-zoom, 0.0, dt).real();
    mode.yaw += rate * dt;
    mode.pitch = std::clamp(mode.pitch + input.pitch * dt, -kPitchLimit, kPitchLimit);
    mode.distance = std::max(mode.distance * std::exp(-input.zoom * dt), kMinDistance);
}

// Surface: heading is linear, speed decays as exp(-zoom t) and altitude
// follows the decaying climb rate exactly. Ground motion along the turning
// heading is exact in the local tangent plane,
//   north + i east = s0 (move.z + i move.x) e^{i h0} integral exp((-zoom - i yaw) t) dt,
// and is converted to latitude/longitude as arc length on the sphere of
// radius R + altitude, with the midpoint altitude and latitude (the caller
// keeps steps at most kSubstep long).
// APPROXIMATION: Surface latitude/longitude - midpoint rule per substep of at
// most 1/240 s instead of the exact path on the sphere - second order in the
// substep (the tangent-plane motion itself is exact) - camera placement only.
void advance(SurfaceMode& mode, const CameraInput& input, double dt,
             const std::optional<BodyFrame>& body) {
    const double speed = mode.speed;
    const double travel = exp_integral(-input.zoom, 0.0, dt).real();
    const std::complex<double> ground = speed * std::complex<double>(input.move.z, input.move.x) *
                                        std::polar(1.0, mode.heading) *
                                        exp_integral(-input.zoom, -input.yaw, dt);
    mode.heading -= input.yaw * dt;
    mode.pitch = std::clamp(mode.pitch + input.pitch * dt, -kPitchLimit, kPitchLimit);
    mode.speed = speed * std::exp(-input.zoom * dt);
    const double start_altitude = mode.altitude;
    mode.altitude = std::max(start_altitude + input.move.y * speed * travel, 0.0);
    if (!body) {
        return;
    }
    const double radius =
        std::max(body->radius + 0.5 * (start_altitude + mode.altitude), kMinDistance);
    const double middle_latitude =
        std::clamp(mode.latitude + 0.5 * ground.real() / radius, -kLatitudeLimit, kLatitudeLimit);
    mode.latitude =
        std::clamp(mode.latitude + ground.real() / radius, -kLatitudeLimit, kLatitudeLimit);
    mode.longitude += ground.imag() / (radius * std::cos(middle_latitude));
}

FreeMode smooth(const FreeMode& c, const FreeMode& g0, const FreeMode& g1, const Blend& b) {
    FreeMode result = g1;
    result.position = blend(c.position, g0.position, g1.position, b);
    result.orientation = blend(c.orientation, g0.orientation, g1.orientation, b);
    return result;
}

OrbitMode smooth(const OrbitMode& c, const OrbitMode& g0, const OrbitMode& g1, const Blend& b) {
    OrbitMode result = g1;
    result.focus = blend(c.focus, g0.focus, g1.focus, b);
    result.distance = log_blend(c.distance, g0.distance, g1.distance, b);
    result.yaw = blend(c.yaw, g0.yaw, g1.yaw, b);
    result.pitch = blend(c.pitch, g0.pitch, g1.pitch, b);
    return result;
}

TurntableMode smooth(const TurntableMode& c, const TurntableMode& g0, const TurntableMode& g1,
                     const Blend& b) {
    TurntableMode result = g1;
    result.focus = blend(c.focus, g0.focus, g1.focus, b);
    result.distance = log_blend(c.distance, g0.distance, g1.distance, b);
    result.yaw = blend(c.yaw, g0.yaw, g1.yaw, b);
    result.pitch = blend(c.pitch, g0.pitch, g1.pitch, b);
    return result;
}

ChaseMode smooth(const ChaseMode& c, const ChaseMode& g0, const ChaseMode& g1, const Blend& b) {
    ChaseMode result = g1;
    result.distance = log_blend(c.distance, g0.distance, g1.distance, b);
    result.yaw = blend(c.yaw, g0.yaw, g1.yaw, b);
    result.pitch = blend(c.pitch, g0.pitch, g1.pitch, b);
    return result;
}

SurfaceMode smooth(const SurfaceMode& c, const SurfaceMode& g0, const SurfaceMode& g1,
                   const Blend& b) {
    SurfaceMode result = g1;
    result.latitude = blend(c.latitude, g0.latitude, g1.latitude, b);
    result.longitude = blend(c.longitude, g0.longitude, g1.longitude, b);
    result.altitude = std::max(blend(c.altitude, g0.altitude, g1.altitude, b), 0.0);
    result.heading = blend(c.heading, g0.heading, g1.heading, b);
    result.pitch = blend(c.pitch, g0.pitch, g1.pitch, b);
    return result;
}

// Advances `goal` over [0, dt] and, when `view` is given, smooths it towards
// the goal piece by piece. Pieces end at the clamp `breakpoints` (times in
// (0, dt); others are ignored), and each segment between them is cut into
// equal pieces of at most `max_piece` seconds. Within every piece the goal
// is linear (or, for curved Free/Surface paths, close to linear), which is
// what the blend of make_blend() integrates exactly.
template <typename Mode, std::size_t N, typename Advance>
void integrate(Mode& goal, Mode* view, double dt, double tau, std::array<double, N> breakpoints,
               double max_piece, const Advance& advance_goal) {
    std::ranges::sort(breakpoints);
    double t = 0.0;
    const auto run_segment = [&](double end) {
        const double length = end - t;
        int pieces = 1;
        if (length > max_piece) {
            // The small allowance keeps splits into multiples of max_piece
            // from gaining an extra piece through rounding.
            const double needed = std::ceil(length / max_piece - 1.0e-9);
            pieces = static_cast<int>(std::min(needed, static_cast<double>(kMaxSubsteps)));
        }
        const double piece = length / pieces;
        for (int i = 0; i < pieces; ++i) {
            const Mode before = goal;
            advance_goal(goal, piece);
            if (view != nullptr) {
                *view = smooth(*view, before, goal, make_blend(piece, tau));
            }
        }
        t = end;
    };
    for (const double breakpoint : breakpoints) {
        if (breakpoint > t && breakpoint < dt) {
            run_segment(breakpoint);
        }
    }
    run_segment(dt);
}

} // namespace

void Camera::set_mode(const CameraMode& mode) {
    goal_ = mode;
    current_ = mode;
    snap_ = true;
    chase_frame_.reset();
}

CameraModeKind Camera::mode_kind() const noexcept {
    return static_cast<CameraModeKind>(goal_.index());
}

void Camera::set_smoothing_time(double seconds) noexcept {
    smoothing_time_ = std::max(seconds, 0.0);
}

bool Camera::update(double dt, const CameraInput& input, const BodyFrameLookup& lookup) {
    if (!(dt > 0.0)) {
        dt = 0.0;
    }
    const std::optional<core::BodyId> target = target_of(goal_);
    std::optional<BodyFrame> frame;
    if (target && lookup) {
        frame = lookup(*target);
    }

    // The view follows the goal with smoothing unless the mode or target just
    // changed (snap).
    const bool follow =
        !snap_ && goal_.index() == current_.index() && target_of(current_) == target;
    const double tau = smoothing_time_;
    std::visit(
        [&](auto& goal) {
            using T = std::decay_t<decltype(goal)>;
            T* view = follow ? &std::get<T>(current_) : nullptr;
            if constexpr (std::is_same_v<T, FreeMode>) {
                integrate(goal, view, dt, tau, std::array<double, 0>{}, kSubstep,
                          [&](FreeMode& mode, double step) { advance(mode, input, step); });
            } else if constexpr (std::is_same_v<T, SurfaceMode>) {
                const std::array<double, 2> breakpoints{
                    time_to_bound(goal.pitch, input.pitch, -kPitchLimit, kPitchLimit),
                    time_to_ground(goal.altitude, input.move.y * goal.speed, input.zoom)};
                integrate(
                    goal, view, dt, tau, breakpoints, kSubstep,
                    [&](SurfaceMode& mode, double step) { advance(mode, input, step, frame); });
            } else if constexpr (std::is_same_v<T, TurntableMode>) {
                // Substeps keep the curved pan path close to linear per piece
                // for the smoothing, as for Free.
                const std::array<double, 2> breakpoints{
                    time_to_bound(goal.pitch, input.pitch, -kPitchLimit, kPitchLimit),
                    time_to_min_distance(goal.distance, input.zoom)};
                integrate(goal, view, dt, tau, breakpoints, kSubstep,
                          [&](TurntableMode& mode, double step) { advance(mode, input, step); });
            } else {
                const std::array<double, 2> breakpoints{
                    time_to_bound(goal.pitch, input.pitch, -kPitchLimit, kPitchLimit),
                    time_to_min_distance(goal.distance, input.zoom)};
                integrate(goal, view, dt, tau, breakpoints, kNever,
                          [&](T& mode, double step) { advance_angles(mode, input, step); });
            }
        },
        goal_);
    if (!follow) {
        current_ = goal_;
    }
    // Weight of the chase travel frame towards the current velocity frame
    // (a fixed target over the update, so one exponential step is exact).
    const double k = snap_ ? 1.0 : make_blend(dt, tau).k;

    if (target && !frame) {
        return false;
    }

    std::visit(
        [&](const auto& mode) {
            using T = std::decay_t<decltype(mode)>;
            if constexpr (std::is_same_v<T, FreeMode>) {
                anchor_ = mode.position;
                eye_offset_ = glm::dvec3(0.0);
                orientation_ = glm::normalize(mode.orientation);
            } else if constexpr (std::is_same_v<T, OrbitMode>) {
                anchor_ = mode.target ? frame->position : mode.focus;
                const glm::dvec3 direction = spherical(mode.yaw, mode.pitch);
                eye_offset_ = direction * mode.distance;
                orientation_ = look_rotation(-direction, glm::dvec3(0.0, 0.0, kWorldUpZ));
            } else if constexpr (std::is_same_v<T, TurntableMode>) {
                anchor_ = mode.focus;
                const glm::dvec3 direction = spherical(mode.yaw, mode.pitch);
                eye_offset_ = direction * mode.distance;
                orientation_ = look_rotation(-direction, glm::dvec3(0.0, 0.0, kWorldUpZ));
            } else if constexpr (std::is_same_v<T, ChaseMode>) {
                const glm::dquat wanted = travel_frame(frame->velocity);
                if (!chase_frame_ || snap_) {
                    chase_frame_ = wanted;
                } else {
                    chase_frame_ = glm::normalize(glm::slerp(*chase_frame_, wanted, k));
                }
                // Frame coordinates: x forward, y left, z up. yaw = pitch = 0
                // is straight behind the body.
                const glm::dvec3 local{-std::cos(mode.pitch) * std::cos(mode.yaw),
                                       -std::cos(mode.pitch) * std::sin(mode.yaw),
                                       std::sin(mode.pitch)};
                const glm::dvec3 direction = *chase_frame_ * local;
                const glm::dvec3 frame_up = *chase_frame_ * glm::dvec3(0.0, 0.0, 1.0);
                anchor_ = frame->position;
                eye_offset_ = direction * mode.distance;
                orientation_ = look_rotation(-direction, frame_up);
            } else {
                const SurfaceBasis basis = surface_basis(mode.latitude, mode.longitude);
                const glm::dvec3 heading =
                    basis.north * std::cos(mode.heading) + basis.east * std::sin(mode.heading);
                const glm::dvec3 forward =
                    heading * std::cos(mode.pitch) + basis.up * std::sin(mode.pitch);
                const glm::dquat body = glm::normalize(frame->orientation);
                anchor_ = frame->position;
                eye_offset_ = body * (basis.up * (frame->radius + mode.altitude));
                orientation_ = look_rotation(body * forward, body * basis.up);
            }
        },
        current_);
    snap_ = false;
    return true;
}

glm::dvec3 Camera::to_render_double(const glm::dvec3& world) const noexcept {
    return (world - anchor_) - eye_offset_;
}

glm::vec3 Camera::to_render(const glm::dvec3& world) const noexcept {
    return {to_render_double(world)};
}

glm::dvec3 Camera::to_world(const glm::vec3& render) const noexcept {
    return anchor_ + (eye_offset_ + glm::dvec3(render));
}

glm::mat4 Camera::view_matrix() const noexcept {
    return {glm::mat4_cast(glm::conjugate(orientation_))};
}

double Camera::usable_aspect(double aspect) noexcept {
    if (!(aspect > 0.0) || !std::isfinite(aspect)) {
        return 1.0;
    }
    return std::clamp(aspect, kMinAspect, kMaxAspect);
}

core::Result<std::monostate, std::string> Camera::validate_lens(const CameraLens& lens) {
    const auto reject = [](const char* field, double value, const char* range) {
        std::ostringstream text;
        text.precision(17);
        text << "camera lens: " << field << " = " << value << " is outside " << range;
        return core::failure(text.str());
    };
    if (!(lens.vertical_fov >= kMinFov && lens.vertical_fov <= kMaxFov)) {
        return reject("vertical_fov", lens.vertical_fov, "[1e-6, pi - 1e-6] rad");
    }
    if (!(lens.near_plane >= kMinNearPlane && lens.near_plane <= kMaxNearPlane)) {
        return reject("near_plane", lens.near_plane, "[1e-9, 1e12] m");
    }
    if (!(std::isfinite(lens.log_depth_far) && lens.log_depth_far > lens.near_plane)) {
        return reject("log_depth_far", lens.log_depth_far, "(near_plane, finite) m");
    }
    return std::monostate{};
}

core::Result<std::monostate, std::string> Camera::set_lens(const CameraLens& lens) {
    auto valid = validate_lens(lens);
    if (valid) {
        lens_ = lens;
    }
    return valid;
}

glm::mat4 Camera::projection_matrix(double aspect) const noexcept {
    return {projection(lens_, aspect)};
}

glm::mat4 Camera::view_projection_matrix(double aspect) const noexcept {
    return {projection(lens_, aspect) * glm::mat4_cast(glm::conjugate(orientation_))};
}

glm::mat4 Camera::inverse_view_projection_matrix(double aspect) const noexcept {
    return {glm::inverse(projection(lens_, aspect) * glm::mat4_cast(glm::conjugate(orientation_)))};
}

} // namespace orrery::render
