#include "orrery/core/kepler.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace orrery::core {

namespace {

constexpr double pi = std::numbers::pi;
constexpr double two_pi = 2.0 * std::numbers::pi;
constexpr double epsilon = std::numeric_limits<double>::epsilon();
constexpr int max_solver_iterations = 200;

// Plain 3-vector used only inside this translation unit; the public API is
// fully typed.
struct V3 {
    double x;
    double y;
    double z;
};

V3 operator+(V3 a, V3 b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
V3 operator-(V3 a, V3 b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
V3 operator*(double s, V3 a) {
    return {s * a.x, s * a.y, s * a.z};
}
V3 operator/(V3 a, double s) {
    return {a.x / s, a.y / s, a.z / s};
}
double dot(V3 a, V3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
V3 cross(V3 a, V3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
// Scaled norm: no intermediate overflow or underflow of the squared sum.
double length(V3 a) {
    return std::hypot(a.x, a.y, a.z);
}

template <QuantityType Q> V3 raw(const Vec3<Q>& v) {
    return {v.x.value(), v.y.value(), v.z.value()};
}

template <QuantityType Q> Vec3<Q> typed(V3 v) {
    return {Q(v.x), Q(v.y), Q(v.z)};
}

bool valid_mu(space::GravitationalParameter mu) {
    return std::isfinite(mu.value()) && mu.value() > 0.0;
}

double wrap_angle(double radians) {
    return wrap_two_pi(Angle::from_radians(radians)).radians();
}

// 1 - e^2 evaluated as (1 - e)(1 + e) to keep relative precision near e = 1.
double one_minus_e_squared(double e) {
    return (1.0 - e) * (1.0 + e);
}

bool steps_converged(double step, double value) {
    return std::abs(step) <= 4.0 * epsilon * std::max(1.0, std::abs(value));
}

// The eccentric and true anomalies lie in the same half-plane, so they differ
// by less than pi. Moves an atan2 result onto the branch of the reduced
// argument it was computed from, should rounding place it across the cut.
double same_branch(double angle, double reduced) {
    if (angle - reduced > pi) {
        return angle - two_pi;
    }
    if (angle - reduced < -pi) {
        return angle + two_pi;
    }
    return angle;
}

// Result validation: elements and rates must be finite to be returned.
bool finite_elements(const KeplerElements& el) {
    return std::isfinite(el.semi_major_axis.value()) && std::isfinite(el.eccentricity) &&
           std::isfinite(el.inclination.radians()) &&
           std::isfinite(el.longitude_of_ascending_node.radians()) &&
           std::isfinite(el.argument_of_periapsis.radians()) &&
           std::isfinite(el.mean_anomaly.radians());
}

} // namespace

std::string_view to_string(KeplerError error) noexcept {
    switch (error) {
        case KeplerError::invalid_gravitational_parameter:
            return "gravitational parameter must be finite and positive";
        case KeplerError::non_finite_input:
            return "input is not finite";
        case KeplerError::invalid_eccentricity:
            return "eccentricity is negative or out of range for this function";
        case KeplerError::invalid_inclination:
            return "inclination is outside [0, pi]";
        case KeplerError::parabolic_orbit:
            return "parabolic orbits are not supported";
        case KeplerError::inconsistent_semi_major_axis:
            return "semi-major axis sign does not match the eccentricity";
        case KeplerError::degenerate_state:
            return "zero radius or rectilinear motion";
        case KeplerError::beyond_asymptote:
            return "true anomaly lies outside the hyperbolic asymptotes";
        case KeplerError::no_convergence:
            return "anomaly solver did not converge";
    }
    return "unknown Kepler error";
}

Result<Angle, KeplerError> eccentric_anomaly_from_mean(Angle mean_anomaly, double eccentricity) {
    const double e = eccentricity;
    if (!std::isfinite(mean_anomaly.radians()) || !std::isfinite(e)) {
        return failure(KeplerError::non_finite_input);
    }
    if (e < 0.0 || e >= 1.0) {
        return failure(KeplerError::invalid_eccentricity);
    }

    // Solve on the reduced mean anomaly in [-pi, pi), then restore the
    // revolution, so precision does not depend on the number of revolutions.
    const double m_full = mean_anomaly.radians();
    const double m = wrap_pi(mean_anomaly).radians();
    const double revolution_offset = m_full - m;

    // f(E) = E - e sin E - M is increasing; f(M - e) <= 0 <= f(M + e).
    double lo = m - e;
    double hi = m + e;
    const double sign = std::sin(m) < 0.0 ? -1.0 : 1.0;
    double ecc = std::clamp(m + 0.85 * e * sign, lo, hi);

    for (int iteration = 0; iteration < max_solver_iterations; ++iteration) {
        const double f = ecc - e * std::sin(ecc) - m;
        if (f == 0.0) {
            return Angle::from_radians(ecc + revolution_offset);
        }
        if (f > 0.0) {
            hi = ecc;
        } else {
            lo = ecc;
        }
        // f'(E) = 1 - e cos E >= 1 - e > 0.
        const double derivative = 1.0 - e * std::cos(ecc);
        double next = ecc - f / derivative;
        if (!(next > lo && next < hi)) {
            next = 0.5 * (lo + hi);
        }
        const double step = next - ecc;
        ecc = next;
        if (steps_converged(step, ecc) || steps_converged(hi - lo, ecc)) {
            return Angle::from_radians(ecc + revolution_offset);
        }
    }
    return failure(KeplerError::no_convergence);
}

Result<Angle, KeplerError> hyperbolic_anomaly_from_mean(Angle mean_anomaly, double eccentricity) {
    const double e = eccentricity;
    if (!std::isfinite(mean_anomaly.radians()) || !std::isfinite(e)) {
        return failure(KeplerError::non_finite_input);
    }
    if (!(e > 1.0)) {
        return failure(KeplerError::invalid_eccentricity);
    }

    // The equation is odd in (M, H); solve for |M| and restore the sign.
    const double m = std::abs(mean_anomaly.radians());
    const double sign = mean_anomaly.radians() < 0.0 ? -1.0 : 1.0;
    if (m == 0.0) {
        return Angle::from_radians(0.0);
    }

    // f(H) = e sinh H - H - M is increasing for H >= 0.
    // f(asinh(M / e)) = -asinh(M / e) <= 0 and, since sinh H >= H,
    // f(asinh(M / (e - 1))) >= (e - 1) sinh H - M = 0.
    double lo = std::asinh(m / e);
    double hi = std::asinh(m / (e - 1.0));
    if (!std::isfinite(hi)) {
        return failure(KeplerError::no_convergence);
    }
    // f is convex for H > 0, so Newton started where f >= 0 descends
    // monotonically onto the root.
    double h = hi;

    for (int iteration = 0; iteration < max_solver_iterations; ++iteration) {
        const double f = e * std::sinh(h) - h - m;
        if (!std::isfinite(f)) {
            return failure(KeplerError::no_convergence);
        }
        if (f == 0.0) {
            return Angle::from_radians(sign * h);
        }
        if (f > 0.0) {
            hi = h;
        } else {
            lo = h;
        }
        const double derivative = e * std::cosh(h) - 1.0;
        double next = h - f / derivative;
        if (!(next > lo && next < hi)) {
            next = 0.5 * (lo + hi);
        }
        const double step = next - h;
        h = next;
        if (steps_converged(step, h) || steps_converged(hi - lo, h)) {
            return Angle::from_radians(sign * h);
        }
    }
    return failure(KeplerError::no_convergence);
}

Angle mean_anomaly_from_eccentric(Angle eccentric_anomaly, double eccentricity) {
    const double ecc = eccentric_anomaly.radians();
    return Angle::from_radians(ecc - eccentricity * std::sin(ecc));
}

Angle mean_anomaly_from_hyperbolic(Angle hyperbolic_anomaly, double eccentricity) {
    const double h = hyperbolic_anomaly.radians();
    return Angle::from_radians(eccentricity * std::sinh(h) - h);
}

Angle true_anomaly_from_eccentric(Angle eccentric_anomaly, double eccentricity) {
    const double e = eccentricity;
    // Evaluate on the reduced anomaly and keep the result in its revolution.
    const double reduced = wrap_pi(eccentric_anomaly).radians();
    const double offset = eccentric_anomaly.radians() - reduced;
    const double nu = same_branch(
        std::atan2(std::sqrt(one_minus_e_squared(e)) * std::sin(reduced), std::cos(reduced) - e),
        reduced);
    return Angle::from_radians(nu + offset);
}

Angle eccentric_anomaly_from_true(Angle true_anomaly, double eccentricity) {
    const double e = eccentricity;
    const double reduced = wrap_pi(true_anomaly).radians();
    const double offset = true_anomaly.radians() - reduced;
    const double ecc = same_branch(
        std::atan2(std::sqrt(one_minus_e_squared(e)) * std::sin(reduced), e + std::cos(reduced)),
        reduced);
    return Angle::from_radians(ecc + offset);
}

Angle true_anomaly_from_hyperbolic(Angle hyperbolic_anomaly, double eccentricity) {
    const double e = eccentricity;
    const double h = hyperbolic_anomaly.radians();
    return Angle::from_radians(
        std::atan2(std::sqrt((e - 1.0) * (e + 1.0)) * std::sinh(h), e - std::cosh(h)));
}

Result<Angle, KeplerError> hyperbolic_anomaly_from_true(Angle true_anomaly, double eccentricity) {
    const double e = eccentricity;
    const double nu = wrap_pi(true_anomaly).radians();
    if (!std::isfinite(nu) || !std::isfinite(e)) {
        return failure(KeplerError::non_finite_input);
    }
    if (!(e > 1.0)) {
        return failure(KeplerError::invalid_eccentricity);
    }
    const double denominator = 1.0 + e * std::cos(nu);
    if (!(denominator > 0.0)) {
        return failure(KeplerError::beyond_asymptote);
    }
    return Angle::from_radians(
        std::asinh(std::sqrt((e - 1.0) * (e + 1.0)) * std::sin(nu) / denominator));
}

// APPROXIMATION: two-body Keplerian motion about a point mass - the elements
// describe the osculating conic of the instantaneous state and ignore
// perturbations from other bodies, oblateness and relativistic corrections -
// exact at the instant of conversion; using them to predict motion is valid
// only while perturbations are small over the time span considered.
Result<CartesianState, KeplerError> to_cartesian(const KeplerElements& elements,
                                                 space::GravitationalParameter mu) {
    if (!valid_mu(mu)) {
        return failure(KeplerError::invalid_gravitational_parameter);
    }
    const double a = elements.semi_major_axis.value();
    const double e = elements.eccentricity;
    const double inc = elements.inclination.radians();
    const double node = elements.longitude_of_ascending_node.radians();
    const double arg = elements.argument_of_periapsis.radians();
    const double mean = elements.mean_anomaly.radians();
    if (!std::isfinite(a) || !std::isfinite(e) || !std::isfinite(inc) || !std::isfinite(node) ||
        !std::isfinite(arg) || !std::isfinite(mean)) {
        return failure(KeplerError::non_finite_input);
    }
    if (e < 0.0) {
        return failure(KeplerError::invalid_eccentricity);
    }
    if (inc < 0.0 || inc > pi) {
        return failure(KeplerError::invalid_inclination);
    }
    if (std::abs(e - 1.0) < kepler_tolerance::parabolic_band) {
        return failure(KeplerError::parabolic_orbit);
    }
    const bool elliptic = e < 1.0;
    if ((elliptic && !(a > 0.0)) || (!elliptic && !(a < 0.0))) {
        return failure(KeplerError::inconsistent_semi_major_axis);
    }

    // Position and velocity in the perifocal frame (x towards periapsis).
    double xp = 0.0;
    double yp = 0.0;
    double vxp = 0.0;
    double vyp = 0.0;
    if (elliptic) {
        const auto solved = eccentric_anomaly_from_mean(elements.mean_anomaly, e);
        if (!solved) {
            return failure(solved.error());
        }
        const double ecc = solved.value().radians();
        const double cos_e = std::cos(ecc);
        const double sin_e = std::sin(ecc);
        const double root = std::sqrt(one_minus_e_squared(e));
        const double r = a * (1.0 - e * cos_e);
        const double speed_scale = std::sqrt(mu.value() * a) / r;
        xp = a * (cos_e - e);
        yp = a * root * sin_e;
        vxp = -speed_scale * sin_e;
        vyp = speed_scale * root * cos_e;
    } else {
        const auto solved = hyperbolic_anomaly_from_mean(elements.mean_anomaly, e);
        if (!solved) {
            return failure(solved.error());
        }
        const double h = solved.value().radians();
        const double cosh_h = std::cosh(h);
        const double sinh_h = std::sinh(h);
        const double abs_a = -a;
        const double root = std::sqrt((e - 1.0) * (e + 1.0));
        const double r = abs_a * (e * cosh_h - 1.0);
        const double speed_scale = std::sqrt(mu.value() * abs_a) / r;
        xp = abs_a * (e - cosh_h);
        yp = abs_a * root * sinh_h;
        vxp = -speed_scale * sinh_h;
        vyp = speed_scale * root * cosh_h;
    }

    // Rotation R3(node) R1(inc) R3(arg) applied to the perifocal basis.
    const double c_node = std::cos(node);
    const double s_node = std::sin(node);
    const double c_inc = std::cos(inc);
    const double s_inc = std::sin(inc);
    const double c_arg = std::cos(arg);
    const double s_arg = std::sin(arg);
    const V3 p{c_node * c_arg - s_node * s_arg * c_inc, s_node * c_arg + c_node * s_arg * c_inc,
               s_arg * s_inc};
    const V3 q{-c_node * s_arg - s_node * c_arg * c_inc, -s_node * s_arg + c_node * c_arg * c_inc,
               c_arg * s_inc};

    const V3 position = xp * p + yp * q;
    const V3 velocity = vxp * p + vyp * q;
    if (!std::isfinite(length(position)) || !std::isfinite(length(velocity))) {
        return failure(KeplerError::no_convergence);
    }
    return CartesianState{typed<space::Length>(position), typed<space::Velocity>(velocity)};
}

Result<KeplerElements, KeplerError> to_elements(const CartesianState& state,
                                                space::GravitationalParameter mu) {
    if (!valid_mu(mu)) {
        return failure(KeplerError::invalid_gravitational_parameter);
    }
    if (!isfinite(state.position) || !isfinite(state.velocity)) {
        return failure(KeplerError::non_finite_input);
    }
    const double gm = mu.value();
    const V3 r = raw(state.position);
    const V3 v = raw(state.velocity);
    const double r_len = length(r);
    const double v_len = length(v);
    const V3 h = cross(r, v);
    const double h_len = length(h);
    if (!(r_len > 0.0) || !(h_len > kepler_tolerance::rectilinear_sine * r_len * v_len)) {
        return failure(KeplerError::degenerate_state);
    }

    // Eccentricity vector e = ((v^2 - mu/r) r - (r.v) v) / mu.
    // Dividing by mu (rather than multiplying by 1 / mu) avoids overflowing
    // 1 / mu for tiny mu.
    const V3 e_vec = ((dot(v, v) - gm / r_len) * r - dot(r, v) * v) / gm;
    const double e = length(e_vec);
    if (std::abs(e - 1.0) < kepler_tolerance::parabolic_band) {
        return failure(KeplerError::parabolic_orbit);
    }
    // Vis-viva: 1/a = 2/r - v^2/mu.
    const double inverse_a = 2.0 / r_len - dot(v, v) / gm;
    const bool elliptic = e < 1.0;
    if ((elliptic && !(inverse_a > 0.0)) || (!elliptic && !(inverse_a < 0.0))) {
        return failure(KeplerError::inconsistent_semi_major_axis);
    }
    const double a = 1.0 / inverse_a;

    // Orbit normal W, node direction N and the in-plane direction W x N that
    // follows N in the direction of motion.
    const V3 w = (1.0 / h_len) * h;
    const double node_len = std::hypot(h.x, h.y);
    const double inclination = std::atan2(node_len, h.z);
    const bool equatorial = node_len < kepler_tolerance::equatorial_sine * h_len;
    double node_angle = 0.0;
    V3 n_hat{1.0, 0.0, 0.0};
    if (!equatorial) {
        // N = z x h / |z x h| = (-h_y, h_x, 0) / |h_xy|.
        n_hat = V3{-h.y / node_len, h.x / node_len, 0.0};
        node_angle = std::atan2(n_hat.y, n_hat.x);
    }
    const V3 m_hat = cross(w, n_hat);

    const bool circular = e < kepler_tolerance::circular_eccentricity;
    double periapsis_angle = 0.0;
    V3 p_hat = n_hat;
    if (!circular) {
        p_hat = (1.0 / e) * e_vec;
        periapsis_angle = std::atan2(dot(p_hat, m_hat), dot(p_hat, n_hat));
    }
    const V3 q_hat = cross(w, p_hat);
    const double true_anomaly = std::atan2(dot(r, q_hat), dot(r, p_hat));

    double mean = 0.0;
    if (elliptic) {
        const Angle ecc = eccentric_anomaly_from_true(Angle::from_radians(true_anomaly), e);
        mean = wrap_angle(mean_anomaly_from_eccentric(ecc, e).radians());
    } else {
        // e sinh H = (r . v) / sqrt(mu |a|). Computing H from r . v directly
        // avoids the cancellation in 1 + e cos(nu) near the asymptotes.
        // sqrt(mu) sqrt(|a|) cannot underflow to zero where mu |a| would.
        const double radial = dot(r, v) / (std::sqrt(gm) * std::sqrt(-a));
        const double h_anomaly = std::asinh(radial / e);
        mean = radial - h_anomaly;
    }

    KeplerElements elements;
    elements.semi_major_axis = space::Length(a);
    elements.eccentricity = e;
    elements.inclination = Angle::from_radians(inclination);
    elements.longitude_of_ascending_node = Angle::from_radians(wrap_angle(node_angle));
    elements.argument_of_periapsis = Angle::from_radians(wrap_angle(periapsis_angle));
    elements.mean_anomaly = Angle::from_radians(mean);
    // Squared speeds or radii beyond the double range still overflow; such
    // states are reported instead of returning non-finite elements.
    if (!finite_elements(elements) || a == 0.0) {
        return failure(KeplerError::no_convergence);
    }
    return elements;
}

Result<space::Frequency, KeplerError> mean_motion(space::Length semi_major_axis,
                                                  space::GravitationalParameter mu) {
    if (!valid_mu(mu)) {
        return failure(KeplerError::invalid_gravitational_parameter);
    }
    if (!isfinite(semi_major_axis)) {
        return failure(KeplerError::non_finite_input);
    }
    if (semi_major_axis.value() == 0.0) {
        return failure(KeplerError::inconsistent_semi_major_axis);
    }
    // sqrt(mu / a) / a instead of sqrt(mu / a^3): a^3 overflows long before n
    // leaves the double range.
    const space::Length a = abs(semi_major_axis);
    const space::Frequency n = sqrt(mu / a) / a;
    if (!isfinite(n) || !(n.value() > 0.0)) {
        return failure(KeplerError::no_convergence);
    }
    return n;
}

Result<space::Time, KeplerError> orbital_period(space::Length semi_major_axis,
                                                space::GravitationalParameter mu) {
    if (!valid_mu(mu)) {
        return failure(KeplerError::invalid_gravitational_parameter);
    }
    if (!isfinite(semi_major_axis)) {
        return failure(KeplerError::non_finite_input);
    }
    if (!(semi_major_axis.value() > 0.0)) {
        return failure(KeplerError::inconsistent_semi_major_axis);
    }
    // a sqrt(a / mu) instead of sqrt(a^3 / mu), for the same reason.
    const space::Length a = semi_major_axis;
    const space::Time period = two_pi * (a * sqrt(a / mu));
    if (!isfinite(period) || !(period.value() > 0.0)) {
        return failure(KeplerError::no_convergence);
    }
    return period;
}

} // namespace orrery::core
