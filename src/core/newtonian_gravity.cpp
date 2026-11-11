#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <stdexcept>

#include "orrery/core/force_model.h"
#include "orrery/core/units.h"

namespace orrery::core {

namespace {

void check_sizes(const BodyStore& bodies, std::size_t positions, std::size_t out) {
    if (positions != bodies.size() || out != bodies.size()) {
        throw std::invalid_argument("force model: span sizes must equal the number of bodies");
    }
}

// Ordinary range. For squared distances r^2 + eps^2 in [2^-200, 2^200] and
// masses that are 0 or in [2^-280, 2^280] Msun, every intermediate of the
// direct formula (1 / sqrt, its cube, G m / r^3, G m_i m_j / r) is a normal
// double, so it is accurate to rounding. Any other pair takes the scaled path
// below, which is accurate for every representable separation and softening.
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

// A source of gravity: not a probe, and a positive mass (a zero-mass body has
// no field, so it contributes nothing and is never a singular source).
bool is_source(const BodyStore& bodies, std::span<const space::Mass> masses, std::size_t i) {
    return masses[i].value() > 0.0 && bodies.exerts_gravity(i);
}

// Separation scaled by an exact power of two:
// (dx, dy, dz, eps) = 2^e (xs, ys, zs, es) with the largest magnitude in
// [0.5, 1). The scaled distance rs = sqrt(xs^2 + ys^2 + zs^2 + es^2) lies in
// [0.5, 2), so neither the squares nor 1 / rs can overflow or lose the
// leading terms to underflow. u = (dx, dy, dz) / d with
// d = sqrt(r^2 + eps^2) has components of magnitude at most 1.
struct ScaledSeparation {
    double ux = 0.0;
    double uy = 0.0;
    double uz = 0.0;
    double rs = 0.0;
    int e = 0;
};

// False for a zero separation without softening (a point-mass singularity)
// and for a separation that is not finite.
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

// Component of G m (r_j - r_i) / d^3 = (G m / rs^2) u 2^(-2e). The factor
// G m / rs^2 is bounded by 4 G m, |u| <= 1, and the power of two is applied
// last, so no inverse cube is formed.
double scaled_field(double gm, const ScaledSeparation& s, double u) noexcept {
    return std::ldexp((gm / (s.rs * s.rs)) * u, -2 * s.e);
}

struct Field {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// Scaled-path field of one pair, with (dx, dy, dz) = r_j - r_i: on_i is
// G m_j (r_j - r_i) / d^3 and on_j is G m_i (r_j - r_i) / d^3 (subtracted
// from a_j by the caller). NaN for a point-mass singularity. Kept out of the
// ordinary loop body, which it would otherwise slow down.
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

} // namespace

std::string_view to_string(GravityError error) noexcept {
    switch (error) {
        case GravityError::invalid_softening:
            return "softening length is not finite or is negative";
    }
    return "unknown gravity error";
}

Result<NewtonianGravity, GravityError> NewtonianGravity::create(space::Length softening) {
    if (!isfinite(softening) || softening.value() < 0.0) {
        return failure(GravityError::invalid_softening);
    }
    return NewtonianGravity(softening);
}

// APPROXIMATION: Plummer softening - replaces 1/r^2 by r/(r^2 + eps^2)^(3/2)
// to bound the force in close encounters - dynamics are exact Newtonian for
// separations r >> eps (relative force error about 1.5 eps^2 / r^2) and
// altered for r comparable to eps; eps = 0 disables it.
void NewtonianGravity::accelerations(const BodyStore& bodies,
                                     std::span<const Vec3<space::Length>> positions,
                                     std::span<Vec3<space::Acceleration>> out) const {
    check_sizes(bodies, positions.size(), out.size());
    const std::size_t n = bodies.size();
    const std::span<const space::Mass> masses = bodies.masses();
    const double g = constants::gravitational_constant_space.value();
    const double eps = softening_.value();
    const double eps2 = eps * eps;
    const bool ordinary = ordinary_masses(masses);

    // Raw doubles in the inner loop: every value is in space base units (AU,
    // Msun, day), so G m / r^2 is directly in AU day^-2.
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
            const double r2 = dx * dx + dy * dy + dz * dz + eps2;
            if (ordinary && ordinary_r2(r2)) {
                // A zero-mass source contributes an exact 0 * finite here.
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
            // Zero-mass sources are skipped before any singular arithmetic
            // (0 * infinity); a zero-mass receiver still feels its sources.
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
NewtonianGravity::potential_energy(const BodyStore& bodies,
                                   std::span<const Vec3<space::Length>> positions) const {
    check_sizes(bodies, positions.size(), positions.size());
    const std::size_t n = bodies.size();
    const std::span<const space::Mass> masses = bodies.masses();
    const double g = constants::gravitational_constant_space.value();
    const double eps = softening_.value();
    const double eps2 = eps * eps;
    const bool ordinary = ordinary_masses(masses);

    // Pairs with a zero mass contribute exactly 0 and are skipped, so a
    // zero-mass body never forms 0 / 0.
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
            const double r2 = dx * dx + dy * dy + dz * dz + eps2;
            if (ordinary && ordinary_r2(r2)) {
                u -= gm_i * masses[j].value() / std::sqrt(r2);
                continue;
            }
            // G m_i m_j / d = (G m_i / rs) m_j 2^(-e); a point-mass singularity
            // gives -infinity.
            ScaledSeparation s;
            u -= scale_separation(dx, dy, dz, eps, s)
                     ? std::ldexp((gm_i / s.rs) * masses[j].value(), -s.e)
                     : std::numeric_limits<double>::infinity();
        }
    }
    return space::Energy(u);
}

} // namespace orrery::core
