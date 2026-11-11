#include "orrery/core/barnes_hut.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <stdexcept>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>

#include "orrery/core/octree.h"
#include "orrery/core/units.h"

namespace orrery::core {

namespace {

constexpr std::uint32_t no_body = std::numeric_limits<std::uint32_t>::max();
// Largest accepted leaf capacity; larger leaves only slow the walk down.
constexpr std::size_t max_leaf_capacity = 4096;
// Receivers handed to one task. Any value gives the same result.
constexpr std::size_t receiver_grain = 64;

void check_sizes(const BodyStore& bodies, std::size_t positions, std::size_t out) {
    if (positions != bodies.size() || out != bodies.size()) {
        throw std::invalid_argument("force model: span sizes must equal the number of bodies");
    }
    if (bodies.size() >= std::size_t{no_body}) {
        throw std::length_error("BarnesHutGravity: too many bodies");
    }
}

// Everything one evaluation needs. Only the key computation and sort inside
// Octree::build run in parallel, and both are order-independent.
// The part of a node the walk reads at every visit, packed into 48 bytes so
// that the opening test stays in cache; the moments are read from the
// Octree node only when it is accepted.
struct WalkNode {
    double com_x = 0.0;
    double com_y = 0.0;
    double com_z = 0.0;
    // Squared opening radius (side / theta + offset)^2 (Barnes 1994): the
    // node is used as a multipole when the squared distance from the receiver
    // to its centre of mass exceeds it. +infinity (never) for theta = 0.
    double open_r2 = 0.0;
    std::uint32_t next = 0;
    std::uint32_t first = 0;
    std::uint32_t count = 0;
    std::uint32_t leaf = 0;
};

struct Prepared {
    Octree tree;
    std::vector<WalkNode> walk;
    // Receivers: the sources in sorted (Morton) order, for cache locality,
    // then every other body in body order. Each body appears exactly once.
    std::vector<std::uint32_t> receivers;
    std::vector<bool> is_source;
    bool valid = false;
};

Prepared prepare(const BodyStore& bodies, std::span<const Vec3<space::Length>> positions,
                 const BarnesHutOptions& options) {
    const std::size_t n = bodies.size();
    const std::span<const space::Mass> masses = bodies.masses();
    const double g = constants::gravitational_constant_space.value();
    Prepared p;
    std::vector<double> gm(n);
    std::vector<std::uint32_t> sources;
    sources.reserve(n);
    p.is_source.assign(n, false);
    for (std::size_t i = 0; i < n; ++i) {
        gm[i] = g * masses[i].value();
        // A zero-mass body has no field (as in NewtonianGravity).
        if (masses[i].value() > 0.0 && bodies.exerts_gravity(i)) {
            sources.push_back(static_cast<std::uint32_t>(i));
            p.is_source[i] = true;
        }
    }
    p.valid = p.tree.build(positions, gm, sources, options.leaf_capacity);
    if (!p.valid) {
        return p;
    }
    const std::span<const Octree::Node> nodes = p.tree.nodes();
    p.walk.resize(nodes.size());
    const double theta = options.opening_angle;
    for (std::size_t k = 0; k < nodes.size(); ++k) {
        const Octree::Node& node = nodes[k];
        WalkNode& w = p.walk[k];
        w.com_x = node.com_x;
        w.com_y = node.com_y;
        w.com_z = node.com_z;
        if (theta > 0.0) {
            const double r = node.side / theta + node.offset;
            w.open_r2 = r * r;
        } else {
            w.open_r2 = std::numeric_limits<double>::infinity();
        }
        w.next = node.next;
        w.first = node.first;
        w.count = node.count;
        w.leaf = node.leaf ? 1U : 0U;
    }
    p.receivers.reserve(n);
    for (const std::uint32_t s : p.tree.index()) {
        p.receivers.push_back(s);
    }
    for (std::size_t i = 0; i < n; ++i) {
        if (!p.is_source[i]) {
            p.receivers.push_back(static_cast<std::uint32_t>(i));
        }
    }
    return p;
}

struct Field {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// Acceleration at (xi, yi, zi) of every source except `self` (a sorted-source
// index, or no_body). Stackless pre-order walk: an accepted node contributes
// its multipole and is skipped, a leaf that is not accepted is summed
// directly, any other node is opened. The order of the contributions is
// fixed by the tree alone.
Field field_at(const Prepared& p, double eps2, double xi, double yi, double zi,
               std::uint32_t self) {
    const Octree& tree = p.tree;
    const std::span<const Octree::Node> nodes = tree.nodes();
    const std::span<const double> sx = tree.x();
    const std::span<const double> sy = tree.y();
    const std::span<const double> sz = tree.z();
    const std::span<const double> sgm = tree.gm();
    double ax = 0.0;
    double ay = 0.0;
    double az = 0.0;
    std::size_t k = 0;
    while (k < p.walk.size()) {
        const WalkNode& w = p.walk[k];
        const double rx = xi - w.com_x;
        const double ry = yi - w.com_y;
        const double rz = zi - w.com_z;
        const double r2 = rx * rx + ry * ry + rz * rz;
        if (r2 > w.open_r2) {
            const Octree::Node& node = nodes[k];
            // APPROXIMATION: multipole truncated after the quadrupole - the
            // octupole and higher terms of the group are dropped - relative
            // error of order (side / d)^3, bounded through theta.
            const double inv = 1.0 / std::sqrt(r2 + eps2);
            const double inv2 = inv * inv;
            const double inv3 = inv2 * inv;
            const double inv5 = inv3 * inv2;
            const double inv7 = inv5 * inv2;
            const std::array<double, 6>& q = node.second;
            const double qrx = q[0] * rx + q[1] * ry + q[2] * rz;
            const double qry = q[1] * rx + q[3] * ry + q[4] * rz;
            const double qrz = q[2] * rx + q[4] * ry + q[5] * rz;
            const double rqr = rx * qrx + ry * qry + rz * qrz;
            const double trace = q[0] + q[3] + q[5];
            const double radial = -node.gm * inv3 + 1.5 * trace * inv5 - 7.5 * rqr * inv7;
            const double tensor = 3.0 * inv5;
            ax += radial * rx + tensor * qrx;
            ay += radial * ry + tensor * qry;
            az += radial * rz + tensor * qrz;
            k = w.next;
        } else if (w.leaf != 0U) {
            const std::uint32_t end = w.first + w.count;
            for (std::uint32_t j = w.first; j < end; ++j) {
                if (j == self) {
                    continue;
                }
                const double dx = sx[j] - xi;
                const double dy = sy[j] - yi;
                const double dz = sz[j] - zi;
                const double d2 = dx * dx + dy * dy + dz * dz + eps2;
                const double inv_r = 1.0 / std::sqrt(d2);
                const double s = sgm[j] * (inv_r * inv_r * inv_r);
                ax += s * dx;
                ay += s * dy;
                az += s * dz;
            }
            k = w.next;
        } else {
            ++k;
        }
    }
    return {ax, ay, az};
}

// Potential per unit mass at (xi, yi, zi) of every source except `self`,
// same walk as field_at.
double potential_at(const Prepared& p, double eps2, double xi, double yi, double zi,
                    std::uint32_t self) {
    const Octree& tree = p.tree;
    const std::span<const Octree::Node> nodes = tree.nodes();
    const std::span<const double> sx = tree.x();
    const std::span<const double> sy = tree.y();
    const std::span<const double> sz = tree.z();
    const std::span<const double> sgm = tree.gm();
    double phi = 0.0;
    std::size_t k = 0;
    while (k < p.walk.size()) {
        const WalkNode& w = p.walk[k];
        const double rx = xi - w.com_x;
        const double ry = yi - w.com_y;
        const double rz = zi - w.com_z;
        const double r2 = rx * rx + ry * ry + rz * rz;
        if (r2 > w.open_r2) {
            const Octree::Node& node = nodes[k];
            const double inv = 1.0 / std::sqrt(r2 + eps2);
            const double inv3 = inv * inv * inv;
            const double inv5 = inv3 * inv * inv;
            const std::array<double, 6>& q = node.second;
            const double rqr = rx * (q[0] * rx + q[1] * ry + q[2] * rz) +
                               ry * (q[1] * rx + q[3] * ry + q[4] * rz) +
                               rz * (q[2] * rx + q[4] * ry + q[5] * rz);
            const double trace = q[0] + q[3] + q[5];
            phi += -node.gm * inv - 1.5 * rqr * inv5 + 0.5 * trace * inv3;
            k = w.next;
        } else if (w.leaf != 0U) {
            const std::uint32_t end = w.first + w.count;
            for (std::uint32_t j = w.first; j < end; ++j) {
                if (j == self) {
                    continue;
                }
                const double dx = sx[j] - xi;
                const double dy = sy[j] - yi;
                const double dz = sz[j] - zi;
                phi -= sgm[j] / std::sqrt(dx * dx + dy * dy + dz * dz + eps2);
            }
            k = w.next;
        } else {
            ++k;
        }
    }
    return phi;
}

// Runs visit(r) for every receiver slot r in [0, count) with oneTBB,
// optionally inside an arena limited to max_threads. Each slot writes only
// its own outputs, so neither the partitioning nor the thread count can
// change a result.
template <typename Visit>
void for_each_receiver(std::size_t count, int max_threads, const Visit& visit) {
    const auto run = [&] {
        tbb::parallel_for(tbb::blocked_range<std::size_t>(0, count, receiver_grain),
                          [&](const tbb::blocked_range<std::size_t>& range) {
                              for (std::size_t r = range.begin(); r != range.end(); ++r) {
                                  visit(r);
                              }
                          });
    };
    if (max_threads > 0) {
        tbb::task_arena arena(max_threads);
        arena.execute(run);
    } else {
        run();
    }
}

} // namespace

std::string_view to_string(BarnesHutError error) noexcept {
    switch (error) {
        case BarnesHutError::invalid_softening:
            return "softening length is not finite or is negative";
        case BarnesHutError::invalid_opening_angle:
            return "opening angle is not in [0, 1]";
        case BarnesHutError::invalid_leaf_capacity:
            return "leaf capacity is not in [1, 4096]";
        case BarnesHutError::invalid_thread_count:
            return "maximum thread count is negative";
    }
    return "unknown Barnes-Hut error";
}

Result<BarnesHutGravity, BarnesHutError> BarnesHutGravity::create(const BarnesHutOptions& options) {
    if (!isfinite(options.softening) || options.softening.value() < 0.0) {
        return failure(BarnesHutError::invalid_softening);
    }
    // theta <= 1 also guarantees that a node containing the receiver is never
    // accepted: the receiver is then within offset + side sqrt(3) / 2 of the
    // centre of mass, below the opening radius side / theta + offset.
    if (!(options.opening_angle >= 0.0 && options.opening_angle <= 1.0)) {
        return failure(BarnesHutError::invalid_opening_angle);
    }
    if (options.leaf_capacity < 1 || options.leaf_capacity > max_leaf_capacity) {
        return failure(BarnesHutError::invalid_leaf_capacity);
    }
    if (options.max_threads < 0) {
        return failure(BarnesHutError::invalid_thread_count);
    }
    return BarnesHutGravity(options);
}

void BarnesHutGravity::accelerations(const BodyStore& bodies,
                                     std::span<const Vec3<space::Length>> positions,
                                     std::span<Vec3<space::Acceleration>> out) const {
    check_sizes(bodies, positions.size(), out.size());
    const Prepared p = prepare(bodies, positions, options_);
    if (!p.valid) {
        // A non-finite or overflowing source position: report it as a
        // non-finite acceleration, which the integrators reject.
        const auto nan = space::Acceleration(std::numeric_limits<double>::quiet_NaN());
        for (Vec3<space::Acceleration>& a : out) {
            a = Vec3<space::Acceleration>{nan, nan, nan};
        }
        return;
    }
    const double eps = options_.softening.value();
    const double eps2 = eps * eps;
    const std::size_t sources = p.tree.index().size();
    for_each_receiver(p.receivers.size(), options_.max_threads, [&](std::size_t r) {
        const std::uint32_t body = p.receivers[r];
        const Vec3<space::Length>& x = positions[body];
        // Receivers [0, sources) are the sorted sources themselves.
        const std::uint32_t self = r < sources ? static_cast<std::uint32_t>(r) : no_body;
        const Field f = field_at(p, eps2, x.x.value(), x.y.value(), x.z.value(), self);
        out[body] = Vec3<space::Acceleration>{space::Acceleration(f.x), space::Acceleration(f.y),
                                              space::Acceleration(f.z)};
    });
}

space::Energy
BarnesHutGravity::potential_energy(const BodyStore& bodies,
                                   std::span<const Vec3<space::Length>> positions) const {
    check_sizes(bodies, positions.size(), positions.size());
    const Prepared p = prepare(bodies, positions, options_);
    if (!p.valid) {
        return space::Energy(std::numeric_limits<double>::quiet_NaN());
    }
    const double eps = options_.softening.value();
    const double eps2 = eps * eps;
    // phi of every source, computed independently and stored by body index.
    const std::size_t sources = p.tree.index().size();
    std::vector<double> phi(bodies.size(), 0.0);
    for_each_receiver(sources, options_.max_threads, [&](std::size_t r) {
        const std::uint32_t body = p.receivers[r];
        const Vec3<space::Length>& x = positions[body];
        phi[body] = potential_at(p, eps2, x.x.value(), x.y.value(), x.z.value(),
                                 static_cast<std::uint32_t>(r));
    });
    // U = 1/2 sum_i m_i phi_i, reduced sequentially in body order.
    const std::span<const space::Mass> masses = bodies.masses();
    double u = 0.0;
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        if (p.is_source[i]) {
            u += masses[i].value() * phi[i];
        }
    }
    return space::Energy(0.5 * u);
}

} // namespace orrery::core
