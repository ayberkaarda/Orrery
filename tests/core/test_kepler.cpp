#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <sstream>
#include <string>
#include <utility>

#include "orrery/core/kepler.h"
#include "orrery/core/units.h"

using namespace orrery::core;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

constexpr double pi = std::numbers::pi;
const space::GravitationalParameter sun_mu = constants::gm_sun_space;

// Smallest absolute difference between two angles, in radians.
double angle_error(Angle a, Angle b) {
    return std::abs(wrap_pi(a - b).radians());
}

double relative_error(double value, double reference) {
    return std::abs(value - reference) / std::max(std::abs(reference), 1e-300);
}

template <QuantityType Q> double relative_vector_error(const Vec3<Q>& a, const Vec3<Q>& b) {
    return norm(a - b).value() / norm(b).value();
}

KeplerElements make_elements(double a, double e, double i, double node, double arg, double mean) {
    KeplerElements el;
    el.semi_major_axis = space::Length(a);
    el.eccentricity = e;
    el.inclination = Angle::from_radians(i);
    el.longitude_of_ascending_node = Angle::from_radians(node);
    el.argument_of_periapsis = Angle::from_radians(arg);
    el.mean_anomaly = Angle::from_radians(mean);
    return el;
}

std::string describe(const KeplerElements& el) {
    std::ostringstream out;
    out.precision(17);
    out << "a=" << el.semi_major_axis.value() << " e=" << el.eccentricity
        << " i=" << el.inclination.radians() << " node=" << el.longitude_of_ascending_node.radians()
        << " arg=" << el.argument_of_periapsis.radians() << " M=" << el.mean_anomaly.radians();
    return out.str();
}

// Elements as to_elements reports them for orbits whose node or periapsis is
// undefined (see the conventions in kepler.h).
KeplerElements canonical(const KeplerElements& in) {
    KeplerElements out = in;
    const double i = in.inclination.radians();
    const bool equatorial = std::sin(i) < kepler_tolerance::equatorial_sine;
    if (equatorial) {
        const Angle node = in.longitude_of_ascending_node;
        out.longitude_of_ascending_node = Angle::from_radians(0.0);
        out.argument_of_periapsis =
            i < pi / 2 ? in.argument_of_periapsis + node : in.argument_of_periapsis - node;
    }
    if (in.eccentricity < kepler_tolerance::circular_eccentricity) {
        out.mean_anomaly = out.mean_anomaly + out.argument_of_periapsis;
        out.argument_of_periapsis = Angle::from_radians(0.0);
    }
    return out;
}

bool all_finite(const KeplerElements& el) {
    return std::isfinite(el.semi_major_axis.value()) && std::isfinite(el.eccentricity) &&
           std::isfinite(el.inclination.radians()) &&
           std::isfinite(el.longitude_of_ascending_node.radians()) &&
           std::isfinite(el.argument_of_periapsis.radians()) &&
           std::isfinite(el.mean_anomaly.radians());
}

struct RoundTripErrors {
    double a = 0.0;
    double e = 0.0;
    double angles = 0.0;
    double position = 0.0;
    double velocity = 0.0;
    std::string worst;
};

void accumulate(RoundTripErrors& errors, const KeplerElements& input,
                space::GravitationalParameter mu, bool hyperbolic) {
    const auto state = to_cartesian(input, mu);
    REQUIRE(state);
    const auto output = to_elements(state.value(), mu);
    REQUIRE(output);
    const KeplerElements& out = output.value();
    CHECK(all_finite(out));
    const KeplerElements expected = canonical(input);

    errors.a = std::max(
        errors.a, relative_error(out.semi_major_axis.value(), expected.semi_major_axis.value()));
    errors.e = std::max(errors.e, std::abs(out.eccentricity - expected.eccentricity) /
                                      std::max(expected.eccentricity, 1.0e-2));
    double angles = std::max(
        {angle_error(out.inclination, expected.inclination),
         angle_error(out.longitude_of_ascending_node, expected.longitude_of_ascending_node),
         angle_error(out.argument_of_periapsis, expected.argument_of_periapsis)});
    if (hyperbolic) {
        // The hyperbolic mean anomaly is not periodic.
        angles = std::max(angles,
                          std::abs(out.mean_anomaly.radians() - expected.mean_anomaly.radians()) /
                              std::max(1.0, std::abs(expected.mean_anomaly.radians())));
    } else {
        angles = std::max(angles, angle_error(out.mean_anomaly, expected.mean_anomaly));
    }
    if (angles > errors.angles) {
        errors.worst = describe(input);
    }
    errors.angles = std::max(errors.angles, angles);

    const auto again = to_cartesian(out, mu);
    REQUIRE(again);
    errors.position = std::max(
        errors.position, relative_vector_error(again.value().position, state.value().position));
    errors.velocity = std::max(
        errors.velocity, relative_vector_error(again.value().velocity, state.value().velocity));
}

} // namespace

TEST_CASE("Kepler's equation is solved to machine precision", "[core][kepler]") {
    const std::array<double, 10> eccentricities{0.0, 1e-9, 0.01, 0.1,   0.5,
                                                0.8, 0.9,  0.99, 0.999, 0.999999};
    double worst = 0.0;
    for (const double e : eccentricities) {
        for (int k = -40; k <= 40; ++k) {
            const Angle mean = Angle::from_radians(0.1 * k + 1e-10);
            const auto solved = eccentric_anomaly_from_mean(mean, e);
            REQUIRE(solved);
            const double ecc = solved.value().radians();
            worst = std::max(worst, std::abs(ecc - e * std::sin(ecc) - mean.radians()));
            // Inverse chain E -> nu -> E is consistent. Near apoapsis dE/dnu
            // grows as sqrt((1 + e) / (1 - e)), which bounds the attainable
            // precision for e close to 1.
            const Angle nu = true_anomaly_from_eccentric(solved.value(), e);
            CHECK(angle_error(eccentric_anomaly_from_true(nu, e), solved.value()) <
                  1e-13 / (1.0 - e));
        }
    }
    CHECK(worst < 1e-14);

    // Large mean anomalies keep the revolution.
    const auto many = eccentric_anomaly_from_mean(Angle::from_radians(1000.0), 0.3);
    REQUIRE(many);
    CHECK_THAT(mean_anomaly_from_eccentric(many.value(), 0.3).radians(), WithinAbs(1000.0, 1e-11));
}

TEST_CASE("hyperbolic Kepler equation", "[core][kepler]") {
    const std::array<double, 5> eccentricities{1.000001, 1.01, 1.5, 3.0, 50.0};
    for (const double e : eccentricities) {
        for (const double m : {-200.0, -3.0, -0.1, 0.0, 1e-8, 0.4, 2.0, 25.0, 700.0}) {
            const auto solved = hyperbolic_anomaly_from_mean(Angle::from_radians(m), e);
            REQUIRE(solved);
            const double h = solved.value().radians();
            CHECK_THAT(e * std::sinh(h) - h, WithinAbs(m, 1e-13 * std::max(1.0, std::abs(m))));
        }
    }
    const auto overflow = hyperbolic_anomaly_from_mean(Angle::from_radians(1e308), 1.5);
    REQUIRE_FALSE(overflow);
    CHECK(overflow.error() == KeplerError::no_convergence);
}

TEST_CASE("elliptic elements -> state -> elements round trip over a grid", "[core][kepler]") {
    RoundTripErrors errors;
    const std::array<double, 4> semi_major_axes{0.3, 1.0, 5.2, 40.0};
    const std::array<double, 8> eccentricities{0.0, 0.01, 0.1, 0.3, 0.5, 0.7, 0.9, 0.99};
    const std::array<double, 7> inclinations{0.0, 0.1, 0.5, 1.2, pi / 2, 2.5, pi};
    const std::array<double, 3> nodes{0.0, 1.0, 4.0};
    const std::array<double, 3> arguments{0.0, 2.0, 5.5};
    const std::array<double, 5> means{0.0, 0.5, 3.0, pi, 6.0};
    int cases = 0;
    for (const double a : semi_major_axes) {
        for (const double e : eccentricities) {
            for (const double i : inclinations) {
                for (const double node : nodes) {
                    for (const double arg : arguments) {
                        for (const double m : means) {
                            accumulate(errors, make_elements(a, e, i, node, arg, m), sun_mu, false);
                            ++cases;
                        }
                    }
                }
            }
        }
    }
    INFO("cases: " << cases << ", worst angle case: " << errors.worst);
    CHECK(cases == 10080);
    CHECK(errors.a < 1e-12);
    CHECK(errors.e < 1e-12);
    CHECK(errors.angles < 1e-12);
    CHECK(errors.position < 1e-12);
    CHECK(errors.velocity < 1e-12);
}

TEST_CASE("hyperbolic elements round trip", "[core][kepler]") {
    RoundTripErrors errors;
    for (const double a : {-0.5, -2.0, -10.0}) {
        for (const double e : {1.01, 1.5, 3.0, 10.0}) {
            for (const double i : {0.0, 0.3, 2.0}) {
                for (const double m : {-5.0, -0.5, 0.0, 0.5, 2.0, 10.0}) {
                    accumulate(errors, make_elements(a, e, i, 1.1, 2.2, m), sun_mu, true);
                }
            }
        }
    }
    INFO("worst angle case: " << errors.worst);
    CHECK(errors.a < 1e-12);
    CHECK(errors.e < 1e-12);
    CHECK(errors.angles < 1e-12);
    CHECK(errors.position < 1e-12);
    CHECK(errors.velocity < 1e-12);
}

TEST_CASE("near-circular and near-equatorial orbits", "[core][kepler]") {
    // Undefined angles are ill-conditioned, but the combinations that fix the
    // position (longitudes) and the state itself are recovered accurately.
    for (const double e : {1e-14, 1e-12, 1e-9, 1e-6, 1e-4}) {
        for (const double i : {0.0, 1e-15, 1e-13, 1e-9, 1e-5, 0.7, pi - 1e-9, pi}) {
            const KeplerElements input = make_elements(1.3, e, i, 0.4, 1.9, 2.6);
            const auto state = to_cartesian(input, sun_mu);
            REQUIRE(state);
            const auto output = to_elements(state.value(), sun_mu);
            REQUIRE(output);
            const KeplerElements& out = output.value();
            INFO(describe(input) << " -> " << describe(out));
            CHECK_THAT(out.semi_major_axis.value(), WithinRel(1.3, 1e-13));
            CHECK_THAT(out.eccentricity, WithinAbs(e, 1e-15));
            CHECK_THAT(out.inclination.radians(), WithinAbs(i, 1e-15));
            // Mean longitude: node + argument of periapsis + mean anomaly; for
            // retrograde orbits near i = pi the node enters with opposite sign.
            const double node_sign = i < pi / 2 ? 1.0 : -1.0;
            const Angle longitude = node_sign * out.longitude_of_ascending_node +
                                    out.argument_of_periapsis + out.mean_anomaly;
            CHECK(angle_error(longitude, Angle::from_radians(node_sign * 0.4 + 1.9 + 2.6)) < 1e-12);
            const auto again = to_cartesian(out, sun_mu);
            REQUIRE(again);
            CHECK(relative_vector_error(again.value().position, state.value().position) < 1e-12);
            CHECK(relative_vector_error(again.value().velocity, state.value().velocity) < 1e-12);
        }
    }
}

TEST_CASE("circular equatorial orbit has the analytic state", "[core][kepler]") {
    const auto state = to_cartesian(make_elements(2.0, 0.0, 0.0, 0.0, 0.0, pi / 2), sun_mu);
    REQUIRE(state);
    const double speed = std::sqrt(sun_mu.value() / 2.0);
    CHECK_THAT(state.value().position.x.value(), WithinAbs(0.0, 1e-15));
    CHECK_THAT(state.value().position.y.value(), WithinRel(2.0, 1e-15));
    CHECK_THAT(state.value().velocity.x.value(), WithinRel(-speed, 1e-15));
    CHECK_THAT(state.value().velocity.y.value(), WithinAbs(0.0, 1e-18));
    CHECK(state.value().position.z.value() == 0.0);
}

TEST_CASE("Earth-like orbit period", "[core][kepler]") {
    const space::Length a(1.0);
    const auto period = orbital_period(a, sun_mu);
    REQUIRE(period);
    // T = 2 pi sqrt(a^3 / k^2) = 2 pi / k for a = 1 AU: the Gaussian year.
    const double k = constants::gauss_constant_value;
    CHECK_THAT(period.value().value(), WithinRel(2.0 * pi * std::sqrt(1.0 / (k * k)), 1e-15));
    CHECK_THAT(period.value().value(), WithinRel(365.2568983263281, 1e-14));

    const auto n = mean_motion(a, sun_mu);
    REQUIRE(n);
    CHECK_THAT(n.value().value(), WithinRel(k, 1e-15));
    CHECK_THAT((n.value() * period.value()).value(), WithinRel(2.0 * pi, 1e-15));

    // Advancing the mean anomaly by n T returns to the same state; by n T / 2
    // a body that started at periapsis reaches apoapsis.
    const double e = 0.0167;
    const KeplerElements start = make_elements(1.0, e, 0.0, 0.0, 1.8, 0.0);
    KeplerElements later = start;
    later.mean_anomaly =
        start.mean_anomaly + Angle::from_radians((n.value() * period.value()).value());
    KeplerElements half = start;
    half.mean_anomaly =
        start.mean_anomaly + Angle::from_radians((n.value() * period.value()).value() / 2.0);
    const auto s0 = to_cartesian(start, sun_mu);
    const auto s1 = to_cartesian(later, sun_mu);
    const auto sh = to_cartesian(half, sun_mu);
    REQUIRE(s0);
    REQUIRE(s1);
    REQUIRE(sh);
    CHECK(relative_vector_error(s1.value().position, s0.value().position) < 1e-12);
    CHECK(relative_vector_error(s1.value().velocity, s0.value().velocity) < 1e-12);
    CHECK_THAT(norm(s0.value().position).value(), WithinRel(1.0 - e, 1e-15));
    CHECK_THAT(norm(sh.value().position).value(), WithinRel(1.0 + e, 1e-15));

    // Circular speed at 1 AU in SI is about 29.78 km/s.
    const auto circular = to_cartesian(make_elements(1.0, 0.0, 0.0, 0.0, 0.0, 0.0), sun_mu);
    REQUIRE(circular);
    CHECK_THAT(to_si(norm(circular.value().velocity)).value(), WithinRel(29784.69, 1e-6));
}

TEST_CASE("unsupported and invalid inputs return explicit errors", "[core][kepler]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const auto error_of = [](const KeplerElements& el, space::GravitationalParameter mu) {
        const auto r = to_cartesian(el, mu);
        REQUIRE_FALSE(r);
        return r.error();
    };
    CHECK(error_of(make_elements(1.0, 1.0, 0.1, 0.0, 0.0, 0.0), sun_mu) ==
          KeplerError::parabolic_orbit);
    CHECK(error_of(make_elements(1.0, 1.5, 0.1, 0.0, 0.0, 0.0), sun_mu) ==
          KeplerError::inconsistent_semi_major_axis);
    CHECK(error_of(make_elements(-1.0, 0.5, 0.1, 0.0, 0.0, 0.0), sun_mu) ==
          KeplerError::inconsistent_semi_major_axis);
    CHECK(error_of(make_elements(1.0, -0.1, 0.1, 0.0, 0.0, 0.0), sun_mu) ==
          KeplerError::invalid_eccentricity);
    CHECK(error_of(make_elements(1.0, 0.1, 4.0, 0.0, 0.0, 0.0), sun_mu) ==
          KeplerError::invalid_inclination);
    CHECK(error_of(make_elements(1.0, 0.1, 0.1, nan, 0.0, 0.0), sun_mu) ==
          KeplerError::non_finite_input);
    CHECK(
        error_of(make_elements(1.0, 0.1, 0.1, 0.0, 0.0, 0.0), space::GravitationalParameter(0.0)) ==
        KeplerError::invalid_gravitational_parameter);

    const auto elements_error = [](const CartesianState& s) {
        const auto r = to_elements(s, constants::gm_sun_space);
        REQUIRE_FALSE(r);
        return r.error();
    };
    const space::Length one(1.0);
    const space::Length zero(0.0);
    const space::Velocity v0(0.0);
    const space::Velocity v1(0.01);
    // Rectilinear: velocity parallel to position.
    CHECK(elements_error(CartesianState{{one, zero, zero}, {v1, v0, v0}}) ==
          KeplerError::degenerate_state);
    CHECK(elements_error(CartesianState{{zero, zero, zero}, {v0, v1, v0}}) ==
          KeplerError::degenerate_state);
    // Exactly parabolic: v = sqrt(2 mu / r), perpendicular to r.
    const space::Velocity escape(std::sqrt(2.0 * constants::gm_sun_space.value()));
    CHECK(elements_error(CartesianState{{one, zero, zero}, {v0, escape, v0}}) ==
          KeplerError::parabolic_orbit);
    CHECK(elements_error(CartesianState{{one, zero, zero}, {v0, space::Velocity(nan), v0}}) ==
          KeplerError::non_finite_input);

    CHECK(eccentric_anomaly_from_mean(Angle::from_radians(1.0), 1.0).error() ==
          KeplerError::invalid_eccentricity);
    CHECK(hyperbolic_anomaly_from_mean(Angle::from_radians(1.0), 0.5).error() ==
          KeplerError::invalid_eccentricity);
    CHECK(hyperbolic_anomaly_from_true(Angle::from_radians(3.0), 2.0).error() ==
          KeplerError::beyond_asymptote);
    CHECK(orbital_period(space::Length(-1.0), sun_mu).error() ==
          KeplerError::inconsistent_semi_major_axis);
    CHECK_FALSE(to_string(KeplerError::parabolic_orbit).empty());
}

TEST_CASE("signed anomaly reduction keeps precision near periapsis", "[core][kepler]") {
    // small negative angles must not detour through [0, 2 pi).
    for (const double x : {1e-16, 1e-12, 0.5, 3.0}) {
        CHECK(wrap_pi(Angle::from_radians(-x)).radians() == -x);
        CHECK(wrap_pi(Angle::from_radians(x)).radians() == x);
    }

    // Kepler's equation is odd in (M, E): E(-M) = -E(M).
    const double e = 0.99999999;
    for (const double m : {1e-12, 1e-16}) {
        const auto plus = eccentric_anomaly_from_mean(Angle::from_radians(m), e);
        const auto minus = eccentric_anomaly_from_mean(Angle::from_radians(-m), e);
        REQUIRE(plus);
        REQUIRE(minus);
        const double e_plus = plus.value().radians();
        const double e_minus = minus.value().radians();
        INFO("M = +-" << m << ": E(+M) = " << e_plus << ", E(-M) = " << e_minus);
        CHECK(std::abs(e_minus + e_plus) <= 1e-12 * std::abs(e_plus));

        // Reflected states: x and v_y equal, y and v_x opposite.
        const auto sp = to_cartesian(make_elements(1.0, e, 0.0, 0.0, 0.0, m), sun_mu);
        const auto sm = to_cartesian(make_elements(1.0, e, 0.0, 0.0, 0.0, -m), sun_mu);
        REQUIRE(sp);
        REQUIRE(sm);
        const Vec3<space::Length> p = sp.value().position;
        const Vec3<space::Velocity> v = sp.value().velocity;
        const Vec3<space::Length> p_reflected{p.x, -p.y, p.z};
        const Vec3<space::Velocity> v_reflected{-v.x, v.y, v.z};
        CHECK(relative_vector_error(sm.value().position, p_reflected) < 1e-12);
        CHECK(relative_vector_error(sm.value().velocity, v_reflected) < 1e-12);
    }
}

TEST_CASE("anomaly conversions keep the revolution at the branch cut", "[core][kepler]") {
    // raw radians are compared, so an extra revolution is detected.
    const double inf = std::numeric_limits<double>::infinity();
    for (const double e : {0.0, 0.3}) {
        for (const double base : {pi, -pi, 3.0 * pi, -3.0 * pi}) {
            for (const double x : {std::nextafter(base, -inf), base, std::nextafter(base, inf)}) {
                INFO("e = " << e << ", anomaly = " << x);
                const Angle anomaly = Angle::from_radians(x);
                CHECK(std::abs(true_anomaly_from_eccentric(anomaly, e).radians() - x) < 1e-12);
                const Angle ecc = eccentric_anomaly_from_true(anomaly, e);
                CHECK(std::abs(ecc.radians() - x) < 1e-12);
                CHECK(std::abs(mean_anomaly_from_eccentric(ecc, e).radians() - x) < 1e-12);
            }
        }
    }
}

TEST_CASE("extreme finite inputs never succeed with non-finite results", "[core][kepler]") {
    // a^3 overflows although n and T are representable.
    const space::GravitationalParameter unit_mu(1.0);
    const auto n = mean_motion(space::Length(1e110), unit_mu);
    REQUIRE(n);
    CHECK_THAT(n.value().value(), WithinRel(1e-165, 1e-14));
    const auto period = orbital_period(space::Length(1e110), unit_mu);
    REQUIRE(period);
    CHECK_THAT(period.value().value(), WithinRel(6.283185307179586e165, 1e-14));
    // Results outside the double range are reported, not returned as 0 or inf.
    const space::GravitationalParameter tiny_mu(1e-300);
    CHECK(orbital_period(space::Length(1e300), tiny_mu).error() == KeplerError::no_convergence);
    CHECK(mean_motion(space::Length(1e300), tiny_mu).error() == KeplerError::no_convergence);

    // States whose intermediate squared norms overflow or underflow.
    const space::Length one(1.0);
    const space::Length zero(0.0);
    const space::Velocity v0(0.0);
    const std::array<std::pair<CartesianState, double>, 2> cases{
        std::pair{CartesianState{{one, zero, zero}, {v0, space::Velocity(1.0), v0}}, 1e-200},
        std::pair{CartesianState{{one, zero, zero}, {v0, space::Velocity(1e160), v0}}, 1.0}};
    for (const auto& [state, gm] : cases) {
        const auto elements = to_elements(state, space::GravitationalParameter(gm));
        if (elements) {
            INFO(describe(elements.value()));
            CHECK(all_finite(elements.value()));
            CHECK(elements.value().semi_major_axis.value() != 0.0);
        }
    }
    // The first state is representable: a hyperbola with e = v^2 r / mu - 1.
    const auto representable = to_elements(cases[0].first, space::GravitationalParameter(1e-200));
    REQUIRE(representable);
    CHECK_THAT(representable.value().eccentricity, WithinRel(1e200, 1e-14));
    CHECK_THAT(representable.value().semi_major_axis.value(), WithinRel(-1e-200, 1e-14));
}
