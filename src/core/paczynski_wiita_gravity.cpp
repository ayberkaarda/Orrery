#include "orrery/core/paczynski_wiita_gravity.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <stdexcept>
#include <variant>
#include <vector>

#include "orrery/core/black_hole.h"
#include "orrery/core/units.h"

namespace orrery::core {

namespace {

void check_sizes(const BodyStore& bodies, std::size_t positions, std::size_t out) {
    if (positions != bodies.size() || out != bodies.size()) {
        throw std::invalid_argument("force model: span sizes must equal the number of bodies");
    }
}

// Schwarzschild radius of body i if it is a black hole, 0 otherwise.
double horizon_radius(const BodyStore& bodies, std::size_t i) {
    if (std::holds_alternative<BlackHole>(bodies.kind(i))) {
        return schwarzschild_radius(bodies.masses()[i]).value();
    }
    return 0.0;
}

std::vector<double> horizon_radii(const BodyStore& bodies) {
    std::vector<double> radii(bodies.size(), 0.0);
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        radii[i] = horizon_radius(bodies, i);
    }
    return radii;
}

// Newtonian pairs use the same ordinary range, scaled path and expressions as
// NewtonianGravity (newtonian_gravity.cpp), so a system without black holes
// gives bit-identical accelerations and potential for every separation.
constexpr double min_ordinary_r2 = 0x1p-200;
constexpr double max_ordinary_r2 = 0x1p+200;
constexpr double min_ordinary_mass = 0x1p-280;
constexpr double max_ordinary_mass = 0x1p+280;

bool ordinary_masses(std::span<const space::Mass> masses) noexcept {
    for (const space::Mass m : masses) {
        const double v = m.value();
        if (v != 0.0 && (v < min_ordinary_mass || v > max_ordinary_mass)) {
            return false;
        }
    }
    return true;
}

bool ordinary_r2(double r2) noexcept {
    return r2 >= min_ordinary_r2 && r2 <= max_ordinary_r2;
}

// (dx, dy, dz, eps) = 2^e (xs, ys, zs, es) with the largest magnitude in
// [0.5, 1); rs = sqrt(xs^2 + ys^2 + zs^2 + es^2) lies in [0.5, 2) and
// u = (dx, dy, dz) / sqrt(r^2 + eps^2) has components of magnitude at most 1.
struct ScaledSeparation {
    double ux = 0.0;
    double uy = 0.0;
    double uz = 0.0;
    double rs = 0.0;
    int e = 0;
};

// False for a zero separation without softening and for a non-finite one.
bool scale_separation(double dx, double dy, double dz, double eps, ScaledSeparation& out) noexcept {
    const double largest = std::max({std::abs(dx), std::abs(dy), std::abs(dz), eps});
    if (!(largest > 0.0) || !std::isfinite(largest)) {
        return false;
    }
    int e = 0;
    static_cast<void>(std::frexp(largest, &e));
    const double xs = std::ldexp(dx, -e);
    const double ys = std::ldexp(dy, -e);
    const double zs = std::ldexp(dz, -e);
    const double es = std::ldexp(eps, -e);
    out.rs = std::sqrt(xs * xs + ys * ys + zs * zs + es * es);
    out.ux = xs / out.rs;
    out.uy = ys / out.rs;
    out.uz = zs / out.rs;
    out.e = e;
    return true;
}

double scaled_field(double gm, const ScaledSeparation& s, double u) noexcept {
    return std::ldexp((gm / (s.rs * s.rs)) * u, -2 * s.e);
}

struct Field {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// on_i is the field of source j at body i along r_j - r_i, on_j the field of
// source i at body j along the same vector (subtracted from a_j by the
// caller).
struct PairFields {
    Field on_i;
    Field on_j;
};

PairFields scaled_pair_fields(double dx, double dy, double dz, double eps, double gm_j,
                              double gm_i) noexcept {
    ScaledSeparation s;
    if (!scale_separation(dx, dy, dz, eps, s)) {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        return {{nan, nan, nan}, {nan, nan, nan}};
    }
    return {
        {scaled_field(gm_j, s, s.ux), scaled_field(gm_j, s, s.uy), scaled_field(gm_j, s, s.uz)},
        {scaled_field(gm_i, s, s.ux), scaled_field(gm_i, s, s.uy), scaled_field(gm_i, s, s.uz)}};
}

// Distance |(dx, dy, dz)| without forming r^2 (finite unless r exceeds the
// largest double); 0 for a zero separation, NaN for a non-finite one.
double pair_distance(double dx, double dy, double dz) noexcept {
    ScaledSeparation s;
    if (scale_separation(dx, dy, dz, 0.0, s)) {
        return std::ldexp(s.rs, s.e);
    }
    return std::abs(dx) + std::abs(dy) + std::abs(dz) == 0.0
               ? 0.0
               : std::numeric_limits<double>::quiet_NaN();
}

// Paczynski-Wiita pair fields: G m / gap^2 along the unit separation u, with
// gap = max(r - R_ij, h). The direction comes from the scaled separation and
// gap = g 2^k with g in [0.5, 1), so the field is ((G m / g^2) u) 2^(-2k):
// G m / g^2 is at most 4 G m, |u| <= 1 and the power of two is applied last,
// so neither r^2 nor gap^2 r is formed and every representable field is
// obtained to rounding. Zero at r = 0 (undefined direction) and when r
// overflows (the field then underflows for every supported mass); NaN for a
// non-finite separation.
PairFields pw_pair_fields(double dx, double dy, double dz, double pair_rs, double gm_j,
                          double gm_i) noexcept {
    ScaledSeparation s;
    if (!scale_separation(dx, dy, dz, 0.0, s)) {
        if (std::abs(dx) + std::abs(dy) + std::abs(dz) == 0.0) {
            return {};
        }
        const double nan = std::numeric_limits<double>::quiet_NaN();
        return {{nan, nan, nan}, {nan, nan, nan}};
    }
    const double r = std::ldexp(s.rs, s.e);
    if (!std::isfinite(r)) {
        return {};
    }
    const double x = r - pair_rs;
    const double h = PaczynskiWiitaGravity::horizon_floor_fraction * pair_rs;
    const double gap = x >= h ? x : h;
    int k = 0;
    const double g = std::frexp(gap, &k);
    const double inv_g2 = 1.0 / (g * g);
    const auto field = [&](double gm, double u) { return std::ldexp((gm * inv_g2) * u, -2 * k); };
    return {{field(gm_j, s.ux), field(gm_j, s.uy), field(gm_j, s.uz)},
            {field(gm_i, s.ux), field(gm_i, s.uy), field(gm_i, s.uz)}};
}

// Pair potential per unit (G m_i m_j), with the linear continuation below the
// horizon floor.
double pw_inverse_distance(double r, double pair_rs) noexcept {
    const double x = r - pair_rs;
    const double h = PaczynskiWiitaGravity::horizon_floor_fraction * pair_rs;
    if (x >= h) {
        return 1.0 / x;
    }
    return (2.0 * h - x) / (h * h);
}

bool is_source(const BodyStore& bodies, std::span<const space::Mass> masses, std::size_t i) {
    return masses[i].value() > 0.0 && bodies.exerts_gravity(i);
}

} // namespace

Result<PaczynskiWiitaGravity, GravityError> PaczynskiWiitaGravity::create(space::Length softening) {
    if (!isfinite(softening) || softening.value() < 0.0) {
        return failure(GravityError::invalid_softening);
    }
    return PaczynskiWiitaGravity(softening);
}

// APPROXIMATION: Paczynski-Wiita pseudo-Newtonian potential - replaces general
// relativity around a black hole by Phi = -G M / (r - r_s), which reproduces
// the Schwarzschild ISCO (3 r_s) and marginally bound orbit (2 r_s) exactly
// - valid for qualitative strong-field orbits; it gives no correct periapsis
// precession, no frame dragging (spin is ignored) and no gravitational waves
// (ADR-004), and differs from Newton by a relative 2 r_s / r far away.
void PaczynskiWiitaGravity::accelerations(const BodyStore& bodies,
                                          std::span<const Vec3<space::Length>> positions,
                                          std::span<Vec3<space::Acceleration>> out) const {
    check_sizes(bodies, positions.size(), out.size());
    const std::size_t n = bodies.size();
    const std::span<const space::Mass> masses = bodies.masses();
    const double g = constants::gravitational_constant_space.value();
    const double eps = softening_.value();
    const double eps2 = eps * eps;
    const std::vector<double> rs = horizon_radii(bodies);
    const bool ordinary = ordinary_masses(masses);

    for (Vec3<space::Acceleration>& a : out) {
        a = Vec3<space::Acceleration>{};
    }
    for (std::size_t i = 0; i < n; ++i) {
        const bool source_i = bodies.exerts_gravity(i);
        const double gm_i = g * masses[i].value();
        const double xi = positions[i].x.value();
        const double yi = positions[i].y.value();
        const double zi = positions[i].z.value();
        double ax = out[i].x.value();
        double ay = out[i].y.value();
        double az = out[i].z.value();
        for (std::size_t j = i + 1; j < n; ++j) {
            const bool source_j = bodies.exerts_gravity(j);
            if (!source_i && !source_j) {
                continue;
            }
            const double dx = positions[j].x.value() - xi;
            const double dy = positions[j].y.value() - yi;
            const double dz = positions[j].z.value() - zi;
            const double pair_rs = rs[i] + rs[j];
            if (pair_rs > 0.0) {
                const PairFields f =
                    pw_pair_fields(dx, dy, dz, pair_rs, g * masses[j].value(), gm_i);
                if (source_j) {
                    ax += f.on_i.x;
                    ay += f.on_i.y;
                    az += f.on_i.z;
                }
                if (source_i) {
                    out[j].x -= space::Acceleration(f.on_j.x);
                    out[j].y -= space::Acceleration(f.on_j.y);
                    out[j].z -= space::Acceleration(f.on_j.z);
                }
                continue;
            }
            // Newtonian pair: NewtonianGravity's ordinary path, then its
            // scaled path, with the same expressions.
            const double r2 = dx * dx + dy * dy + dz * dz + eps2;
            if (ordinary && ordinary_r2(r2)) {
                const double inv_r = 1.0 / std::sqrt(r2);
                const double inv_r3 = inv_r * inv_r * inv_r;
                if (source_j) {
                    const double s = g * masses[j].value() * inv_r3;
                    ax += s * dx;
                    ay += s * dy;
                    az += s * dz;
                }
                if (source_i) {
                    const double s = gm_i * inv_r3;
                    out[j].x -= space::Acceleration(s * dx);
                    out[j].y -= space::Acceleration(s * dy);
                    out[j].z -= space::Acceleration(s * dz);
                }
                continue;
            }
            const bool from_j = source_j && masses[j].value() > 0.0;
            const bool from_i = source_i && masses[i].value() > 0.0;
            if (!from_i && !from_j) {
                continue;
            }
            const PairFields f = scaled_pair_fields(dx, dy, dz, eps, g * masses[j].value(), gm_i);
            if (from_j) {
                ax += f.on_i.x;
                ay += f.on_i.y;
                az += f.on_i.z;
            }
            if (from_i) {
                out[j].x -= space::Acceleration(f.on_j.x);
                out[j].y -= space::Acceleration(f.on_j.y);
                out[j].z -= space::Acceleration(f.on_j.z);
            }
        }
        out[i] = Vec3<space::Acceleration>{space::Acceleration(ax), space::Acceleration(ay),
                                           space::Acceleration(az)};
    }
}

space::Energy
PaczynskiWiitaGravity::potential_energy(const BodyStore& bodies,
                                        std::span<const Vec3<space::Length>> positions) const {
    check_sizes(bodies, positions.size(), positions.size());
    const std::size_t n = bodies.size();
    const std::span<const space::Mass> masses = bodies.masses();
    const double g = constants::gravitational_constant_space.value();
    const double eps = softening_.value();
    const double eps2 = eps * eps;
    const std::vector<double> rs = horizon_radii(bodies);
    const bool ordinary = ordinary_masses(masses);

    double u = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        if (!is_source(bodies, masses, i)) {
            continue;
        }
        const double gm_i = g * masses[i].value();
        for (std::size_t j = i + 1; j < n; ++j) {
            if (!is_source(bodies, masses, j)) {
                continue;
            }
            const double dx = positions[j].x.value() - positions[i].x.value();
            const double dy = positions[j].y.value() - positions[i].y.value();
            const double dz = positions[j].z.value() - positions[i].z.value();
            const double pair_rs = rs[i] + rs[j];
            if (pair_rs > 0.0) {
                u -= gm_i * masses[j].value() *
                     pw_inverse_distance(pair_distance(dx, dy, dz), pair_rs);
                continue;
            }
            const double r2 = dx * dx + dy * dy + dz * dz + eps2;
            if (ordinary && ordinary_r2(r2)) {
                u -= gm_i * masses[j].value() / std::sqrt(r2);
                continue;
            }
            ScaledSeparation s;
            u -= scale_separation(dx, dy, dz, eps, s)
                     ? std::ldexp((gm_i / s.rs) * masses[j].value(), -s.e)
                     : std::numeric_limits<double>::infinity();
        }
    }
    return space::Energy(u);
}

} // namespace orrery::core
