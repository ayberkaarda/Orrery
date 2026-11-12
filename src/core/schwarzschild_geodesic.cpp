#include "orrery/core/schwarzschild_geodesic.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace orrery::core::geodesic {

namespace {

constexpr double kPi = std::numbers::pi;
// Sweep covered by the base step budget: two full turns, enough for every
// ray except those orbiting the photon sphere more than about once.
constexpr double kSweepBudget = 4.0 * kPi;
// Directions with a smaller transverse component count as radial.
constexpr double kRadialThreshold = 1.0e-12;

struct OrbitState {
    double u = 0.0;  // 1 / r
    double du = 0.0; // du / dphi
};

OrbitState derivative(const OrbitState& s) noexcept {
    return {s.du, -s.u + 3.0 * s.u * s.u};
}

OrbitState rk4_step(const OrbitState& s, double h) noexcept {
    const OrbitState k1 = derivative(s);
    const OrbitState k2 = derivative({s.u + 0.5 * h * k1.u, s.du + 0.5 * h * k1.du});
    const OrbitState k3 = derivative({s.u + 0.5 * h * k2.u, s.du + 0.5 * h * k2.du});
    const OrbitState k4 = derivative({s.u + h * k3.u, s.du + h * k3.du});
    return {s.u + h / 6.0 * (k1.u + 2.0 * k2.u + 2.0 * k3.u + k4.u),
            s.du + h / 6.0 * (k1.du + 2.0 * k2.du + 2.0 * k3.du + k4.du)};
}

// Cubic Hermite interpolation of u across one step (dense output of RK4 to
// fourth order in the step), at the fraction t of the step h.
double hermite(const OrbitState& a, const OrbitState& b, double h, double t) noexcept {
    const double t2 = t * t;
    const double t3 = t2 * t;
    return (2.0 * t3 - 3.0 * t2 + 1.0) * a.u + (t3 - 2.0 * t2 + t) * h * a.du +
           (-2.0 * t3 + 3.0 * t2) * b.u + (t3 - t2) * h * b.du;
}

double dot(const GeoVec& a, const GeoVec& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

GeoVec cross(const GeoVec& a, const GeoVec& b) noexcept {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

GeoVec scaled(const GeoVec& a, double s) noexcept {
    return {a.x * s, a.y * s, a.z * s};
}

GeoVec sum(const GeoVec& a, const GeoVec& b) noexcept {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

double length(const GeoVec& a) noexcept {
    return std::sqrt(dot(a, a));
}

GeoVec normalized(const GeoVec& a) noexcept {
    const double l = length(a);
    return l > 0.0 ? scaled(a, 1.0 / l) : GeoVec{0.0, 0.0, 1.0};
}

// Angle from the end of the step that crossed u = 0 back to u = 0 itself.
// APPROXIMATION: escape angle from the straight line - near u = 0 the orbit
// equation u'' = -u + 3u^2 is replaced by u'' = -u, whose solution is the
// straight line u = A sin(phi_inf - phi) - avoids a root search inside the
// last step - the dropped term is second order in the overshoot (below
// 1e-12 rad for the default budget).
double infinity_offset(const OrbitState& s) noexcept {
    return std::atan2(s.u, -s.du);
}

} // namespace

double base_step(int max_steps) noexcept {
    return kSweepBudget / static_cast<double>(std::clamp(max_steps, kMinSteps, kMaxSteps));
}

double step_size(double u, double du, double base) noexcept {
    const double slope = std::max(std::abs(du), 1.0e-300);
    return std::min(base, kStepGrowth * std::max(u, kStepReferenceU) / slope);
}

double impact_parameter(double radius, double cos_angle) noexcept {
    const double c = std::clamp(cos_angle, -1.0, 1.0);
    const double sin_angle = std::sqrt(std::max(0.0, 1.0 - c * c));
    return radius * sin_angle / std::sqrt(1.0 - 2.0 / radius);
}

double weak_field_deflection(double b) noexcept {
    return 4.0 / b;
}

double second_order_deflection(double b) noexcept {
    return 4.0 / b + 15.0 * kPi / (4.0 * b * b);
}

std::optional<double> deflection_angle(double b, int max_steps) {
    if (!(b > 0.0) || !std::isfinite(b)) {
        return std::nullopt;
    }
    const double base = base_step(max_steps);
    // From infinity: u = 0, du/dphi = 1/b (the orbit equation's first
    // integral (du/dphi)^2 = 1/b^2 - u^2 (1 - 2u) at u = 0).
    OrbitState s{0.0, 1.0 / b};
    double phi = 0.0;
    const int budget = std::clamp(max_steps, kMinSteps, kMaxSteps);
    for (int step = 0; step < budget; ++step) {
        const double h = step_size(s.u, s.du, base);
        const OrbitState next = rk4_step(s, h);
        phi += h;
        if (next.u >= 0.5) {
            return std::nullopt;
        }
        if (next.u <= 0.0 && next.du < 0.0) {
            return phi + infinity_offset(next) - kPi;
        }
        s = next;
    }
    return std::nullopt;
}

double keplerian_angular_velocity(double radius) noexcept {
    return 1.0 / (radius * std::sqrt(radius));
}

double disk_redshift_factor(double radius, double lambda_z, double observer_radius) noexcept {
    const double omega = keplerian_angular_velocity(radius);
    return std::sqrt(1.0 - 3.0 / radius) / (1.0 - omega * lambda_z) /
           std::sqrt(1.0 - 2.0 / observer_radius);
}

double thin_disk_peak_radius(double inner_radius) noexcept {
    return 49.0 / 36.0 * inner_radius;
}

double thin_disk_temperature(double radius, double inner_radius) noexcept {
    if (!(radius > inner_radius) || !(inner_radius > 0.0)) {
        return 0.0;
    }
    const auto profile = [](double x) {
        return std::pow(x, -0.75) * std::pow(1.0 - 1.0 / std::sqrt(x), 0.25);
    };
    return profile(radius / inner_radius) / profile(49.0 / 36.0);
}

RayTrace trace_ray(const GeoVec& camera, const GeoVec& direction, const DiskModel& disk,
                   const TraceSettings& settings) {
    RayTrace trace;
    const double r0 = length(camera);
    if (!(r0 > kHorizonRadius) || !std::isfinite(r0)) {
        trace.fate = RayFate::captured;
        return trace;
    }
    const GeoVec e1 = scaled(camera, 1.0 / r0);
    const GeoVec d = normalized(direction);
    const double cos_a = dot(d, e1);
    const GeoVec transverse = sum(d, scaled(e1, -cos_a));
    const double sin_a = length(transverse);
    if (sin_a < kRadialThreshold) {
        // Radial ray: straight into the hole or straight out.
        trace.fate = cos_a < 0.0 ? RayFate::captured : RayFate::escaped;
        trace.escape_direction = d;
        return trace;
    }
    const GeoVec e2 = scaled(transverse, 1.0 / sin_a);

    const double u0 = 1.0 / r0;
    const double lapse = std::sqrt(1.0 - 2.0 * u0);
    trace.impact_parameter = r0 * sin_a / lapse;
    // The physical photon travels the traced path backwards, so its angular
    // momentum is opposite to e1 x e2.
    const GeoVec axis = normalized(disk.axis);
    trace.lambda_z = -trace.impact_parameter * dot(cross(e1, e2), axis);

    // Crossings of the disk plane: a cos(phi) + c sin(phi) = 0, at
    // phi = atan2(c, a) + pi/2 + k pi. A ray in the plane has none.
    const double a = dot(e1, axis);
    const double c = dot(e2, axis);
    const bool crosses = std::hypot(a, c) > kRadialThreshold;
    double next_crossing = std::fmod(std::atan2(c, a) + 0.5 * kPi + 2.0 * kPi, kPi);
    if (next_crossing <= 0.0) {
        next_crossing += kPi;
    }

    const double base = base_step(settings.max_steps);
    const int budget = std::clamp(settings.max_steps, kMinSteps, kMaxSteps);
    OrbitState s{u0, -u0 * lapse * cos_a / sin_a};
    double phi = 0.0;
    trace.fate = RayFate::exhausted;
    for (int step = 0; step < budget; ++step) {
        const double h = step_size(s.u, s.du, base);
        const OrbitState next = rk4_step(s, h);
        ++trace.steps;
        while (crosses && next_crossing <= phi + h) {
            const double uc = hermite(s, next, h, (next_crossing - phi) / h);
            if (uc > 0.0 && uc < 0.5) {
                PlaneCrossing crossing;
                crossing.radius = 1.0 / uc;
                crossing.on_disk =
                    crossing.radius >= disk.inner_radius && crossing.radius <= disk.outer_radius;
                crossing.g = crossing.radius > kPhotonSphereRadius
                                 ? disk_redshift_factor(crossing.radius, trace.lambda_z, r0)
                                 : 0.0;
                trace.crossings.push_back(crossing);
            }
            next_crossing += kPi;
        }
        phi += h;
        s = next;
        if (s.u >= 0.5) {
            trace.fate = RayFate::captured;
            break;
        }
        if (s.u <= 0.0) {
            const double phi_inf = phi + infinity_offset(s);
            trace.escape_direction =
                sum(scaled(e1, std::cos(phi_inf)), scaled(e2, std::sin(phi_inf)));
            trace.fate = RayFate::escaped;
            break;
        }
    }
    return trace;
}

} // namespace orrery::core::geodesic
