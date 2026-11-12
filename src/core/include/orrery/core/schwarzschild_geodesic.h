#pragma once

#include <cstdint>
#include <optional>
#include <vector>

// Null geodesics of the Schwarzschild metric (docs/physics.md, "Black hole
// rendering"; ADR-003, ADR-017). This is the CPU reference of the GPU ray
// marcher in src/render/shaders/black_hole_trace.comp.glsl: the same orbit
// equation, integrator, step rule and disk model, in double precision, so
// that tests can check the shader numerically and the maths on its own.
//
// Units are geometrized: G = c = 1 and every length is measured in units of
// the gravitational radius M = G M_hole / c^2, so the horizon is at r = 2
// (r_s = 2 M). All quantities below are therefore dimensionless.
//
// A photon moves in the plane spanned by its position and direction. With
// u = 1/r and phi the angle in that plane, its orbit obeys
//
//     d^2u/dphi^2 = -u + 3 u^2,
//
// which is integrated with the classical fourth-order Runge-Kutta method.
namespace orrery::core::geodesic {

// Plain vector in geometrized units (lengths in M, or a direction).
struct GeoVec {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

inline constexpr double kHorizonRadius = 2.0;      // r_s = 2 M
inline constexpr double kPhotonSphereRadius = 3.0; // 1.5 r_s
inline constexpr double kIscoRadius = 6.0;         // 3 r_s, inner edge of the thin disk
// 3 sqrt(3) M: photons with a smaller impact parameter that move inwards
// from outside the photon sphere are captured.
inline constexpr double kCriticalImpactParameter = 5.196152422706632;

// Step rule of the integrator: the step in phi is the base step 4 pi /
// max_steps, shortened so that u changes by at most kStepGrowth of
// max(u, kStepReferenceU) per step (nearly radial rays, where |du/dphi| is
// large).
inline constexpr double kStepGrowth = 0.25;
inline constexpr double kStepReferenceU = 0.01;
inline constexpr int kMaxSteps = 512;
inline constexpr int kMinSteps = 32;

// Base step 4 pi / max_steps (max_steps clamped to [kMinSteps, kMaxSteps]).
[[nodiscard]] double base_step(int max_steps) noexcept;
// Step for the state (u, du/dphi).
[[nodiscard]] double step_size(double u, double du, double base) noexcept;

// Impact parameter b = L / E of a photon seen by a static observer at
// radius r, travelling at an angle with cosine `cos_angle` from the outward
// radial direction: b = r sin(angle) / sqrt(1 - 2 / r). Requires r > 2.
[[nodiscard]] double impact_parameter(double radius, double cos_angle) noexcept;

// Deflection of a ray from infinity with impact parameter b:
//   weak field, first order:   4 / b
//   second order:              4 / b + 15 pi / (4 b^2)
[[nodiscard]] double weak_field_deflection(double b) noexcept;
[[nodiscard]] double second_order_deflection(double b) noexcept;
// Integrated deflection (total sweep minus pi) of a ray from infinity, or
// nothing when the ray is captured (b below the critical value) or the step
// budget runs out first.
[[nodiscard]] std::optional<double> deflection_angle(double b, int max_steps = kMaxSteps);

// Coordinate angular velocity of a circular geodesic, Omega = r^(-3/2).
[[nodiscard]] double keplerian_angular_velocity(double radius) noexcept;

// Frequency ratio g = nu_observed / nu_emitted for light emitted by gas on a
// prograde circular geodesic at radius r (> 3) and received by a static
// observer at `observer_radius` (> 2):
//
//     g = sqrt(1 - 3 / r) / (1 - Omega lambda) / sqrt(1 - 2 / r_obs),
//
// with lambda = L_z / E the photon's angular momentum about the disk axis per
// unit energy (positive when the photon moves with the gas). It combines the
// gravitational redshift, the transverse Doppler shift and the Doppler shift
// of the orbital motion.
[[nodiscard]] double disk_redshift_factor(double radius, double lambda_z,
                                          double observer_radius) noexcept;

// Thin-disk temperature profile with a zero-torque inner edge (Shakura and
// Sunyaev 1973, Page and Thorne 1974 in the Newtonian limit):
//
//     T(r) ~ r^(-3/4) (1 - sqrt(r_in / r))^(1/4),
//
// normalised to 1 at its maximum, r = (49/36) r_in; 0 for r <= r_in. Far from
// the inner edge it falls as r^(-3/4).
[[nodiscard]] double thin_disk_temperature(double radius, double inner_radius) noexcept;
// Radius of the temperature maximum, (49/36) r_in.
[[nodiscard]] double thin_disk_peak_radius(double inner_radius) noexcept;

struct DiskModel {
    GeoVec axis{0.0, 0.0, 1.0}; // unit normal; the gas orbits counter-clockwise about it
    double inner_radius = kIscoRadius;
    double outer_radius = 30.0;
};

struct TraceSettings {
    int max_steps = kMaxSteps;
};

enum class RayFate : std::uint8_t {
    escaped,   // reached infinity (u = 0)
    captured,  // crossed the horizon (u = 1/2), or the camera is inside it
    exhausted, // step budget used up (orbits close to the photon sphere)
};

// One crossing of the disk plane at a radius outside the horizon.
struct PlaneCrossing {
    double radius = 0.0;
    double g = 0.0;       // disk_redshift_factor at this radius (meaningful on the disk)
    bool on_disk = false; // inner_radius <= radius <= outer_radius
};

struct RayTrace {
    RayFate fate = RayFate::captured;
    GeoVec escape_direction; // unit direction at infinity (escaped rays)
    double impact_parameter = 0.0;
    double lambda_z = 0.0; // photon angular momentum about the disk axis per energy
    int steps = 0;
    std::vector<PlaneCrossing> crossings; // in the order the ray meets them
};

// Traces a ray backwards from a static camera at `camera` (relative to the
// hole) along the unit `direction`, through every crossing of the disk plane
// until it escapes, is captured or the budget runs out. The disk is not
// treated as opaque here: every crossing is reported and the caller decides
// (the shader stops once the accumulated opacity is saturated).
[[nodiscard]] RayTrace trace_ray(const GeoVec& camera, const GeoVec& direction,
                                 const DiskModel& disk, const TraceSettings& settings = {});

} // namespace orrery::core::geodesic
