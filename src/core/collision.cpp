#include "orrery/core/collision.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <variant>

namespace orrery::core {

namespace {

using LengthVec = Vec3<space::Length>;

// Power-of-two exponent that brings the largest magnitude to [0.5, 1).
int scale_exponent(std::initializer_list<double> values) noexcept {
    double largest = 0.0;
    for (const double v : values) {
        largest = std::max(largest, std::abs(v));
    }
    if (!(largest > 0.0) || !std::isfinite(largest)) {
        return 0;
    }
    int e = 0;
    static_cast<void>(std::frexp(largest, &e));
    return e;
}

struct Scaled {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

Scaled scaled(const LengthVec& v, int e) noexcept {
    return {std::ldexp(v.x.value(), -e), std::ldexp(v.y.value(), -e), std::ldexp(v.z.value(), -e)};
}

double dot(const Scaled& a, const Scaled& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// a * b - c * d with one rounding error in each product recovered (Kahan's
// algorithm with fused multiply-add): the result is within about one unit in
// the last place of the exact value even when the two products cancel.
// std::fma is correctly rounded, so the result is the same on every platform.
double difference_of_products(double a, double b, double c, double d) noexcept {
    const double w = c * d;
    const double error = std::fma(-c, d, w);
    const double f = std::fma(a, b, -w);
    return f + error;
}

// |a x b| for the scaled relative positions a (start) and b (end). Since
// a x (b - a) = a x b, this is |p0 x (p1 - p0)|: the perpendicular distance
// of the origin from the line of the path times the path length. Every
// component is accurate to a few ulps, unlike the difference
// (a.d)^2 - |d|^2 |a|^2 of nearly equal squares.
double cross_norm(const Scaled& a, const Scaled& b) noexcept {
    const double cx = difference_of_products(a.y, b.z, a.z, b.y);
    const double cy = difference_of_products(a.z, b.x, a.x, b.z);
    const double cz = difference_of_products(a.x, b.y, a.y, b.x);
    return std::sqrt(cx * cx + cy * cy + cz * cz);
}

bool is_black_hole(const BodyStore& bodies, std::size_t i) {
    return std::holds_alternative<BlackHole>(bodies.kind(i));
}

bool is_probe(const BodyStore& bodies, std::size_t i) {
    return std::holds_alternative<Probe>(bodies.kind(i));
}

bool collides(const BodyStore& bodies, std::size_t i) {
    return !is_black_hole(bodies, i) && !is_probe(bodies, i);
}

struct Box {
    std::array<double, 3> lo{};
    std::array<double, 3> hi{};
};

Box swept_box(const SweptSphere& s) noexcept {
    const double r = s.radius.value();
    const std::array<double, 3> a{s.start.x.value(), s.start.y.value(), s.start.z.value()};
    const std::array<double, 3> b{s.end.x.value(), s.end.y.value(), s.end.z.value()};
    Box box;
    for (std::size_t k = 0; k < 3; ++k) {
        box.lo[k] = std::min(a[k], b[k]) - r;
        box.hi[k] = std::max(a[k], b[k]) + r;
    }
    return box;
}

bool overlap(const Box& a, const Box& b) noexcept {
    for (std::size_t k = 0; k < 3; ++k) {
        if (a.hi[k] < b.lo[k] || b.hi[k] < a.lo[k]) {
            return false;
        }
    }
    return true;
}

bool finite_box(const Box& b) noexcept {
    for (std::size_t k = 0; k < 3; ++k) {
        if (!std::isfinite(b.lo[k]) || !std::isfinite(b.hi[k])) {
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Loose octree over swept boxes.
// ---------------------------------------------------------------------------

constexpr std::size_t no_child = std::numeric_limits<std::size_t>::max();
// A node with at most this many movable boxes is not split.
constexpr std::size_t leaf_capacity = 8;
constexpr int max_depth = 64;
// A cell is split only while its half size stays above 2^-40 of the largest
// coordinate magnitude of the cell (about 2^12 ulps), so that the rounding of
// cell corners and box centres stays far below the loose margins.
constexpr double min_relative_cell = 0x1p-40;

struct OctNode {
    std::array<double, 3> lo{};
    double size = 0.0;
    // Boxes stored in this node: items[first, first + count).
    std::size_t first = 0;
    std::size_t count = 0;
    std::array<std::size_t, 8> child{};
};

// Loose bounds of a node: its cell widened by half its size on every side.
// A box whose centre lies in a cell of size S and whose sides are at most
// S / 2 lies inside the loose bounds with a margin of S / 4, and the loose
// bounds of a child lie inside those of its parent with the same margin.
Box loose_bounds(const OctNode& node) noexcept {
    Box b;
    const double half = node.size / 2.0;
    for (std::size_t k = 0; k < 3; ++k) {
        b.lo[k] = node.lo[k] - half;
        b.hi[k] = node.lo[k] + node.size + half;
    }
    return b;
}

class LooseOctree {
public:
    LooseOctree(const std::vector<Box>& boxes, std::vector<std::size_t> items)
        : boxes_(boxes), items_(std::move(items)), centres_(boxes.size()) {
        std::array<double, 3> lo{};
        std::array<double, 3> hi{};
        for (std::size_t k = 0; k < 3; ++k) {
            lo[k] = std::numeric_limits<double>::infinity();
            hi[k] = -std::numeric_limits<double>::infinity();
        }
        for (const std::size_t i : items_) {
            for (std::size_t k = 0; k < 3; ++k) {
                // Halves first: no overflow for coordinates near the limit.
                centres_[i][k] = boxes_[i].lo[k] / 2.0 + boxes_[i].hi[k] / 2.0;
                lo[k] = std::min(lo[k], centres_[i][k]);
                hi[k] = std::max(hi[k], centres_[i][k]);
            }
        }
        OctNode root;
        root.lo = lo;
        for (std::size_t k = 0; k < 3; ++k) {
            root.size = std::max(root.size, hi[k] - lo[k]);
        }
        root.child.fill(no_child);
        nodes_.push_back(root);
        build(0, 0, items_.size(), 0);
    }

    // Every pair (i, j), i < j, of overlapping boxes, unsorted.
    void collect(std::vector<std::pair<std::size_t, std::size_t>>& pairs,
                 BroadPhaseStats& stats) const {
        std::vector<std::size_t> stack;
        for (const std::size_t i : sorted_items()) {
            const Box& box = boxes_[i];
            stack.assign(1, 0);
            while (!stack.empty()) {
                const OctNode& node = nodes_[stack.back()];
                stack.pop_back();
                for (std::size_t m = node.first; m < node.first + node.count; ++m) {
                    const std::size_t j = items_[m];
                    if (j <= i) {
                        continue;
                    }
                    ++stats.box_tests;
                    if (overlap(box, boxes_[j])) {
                        pairs.emplace_back(i, j);
                    }
                }
                // Reverse octant order on the stack: children are visited in
                // ascending octant order.
                for (std::size_t o = 8; o-- > 0;) {
                    const std::size_t c = node.child[o];
                    if (c == no_child) {
                        continue;
                    }
                    ++stats.node_tests;
                    if (overlap(box, loose_bounds(nodes_[c]))) {
                        stack.push_back(c);
                    }
                }
            }
        }
        stats.nodes = nodes_.size();
    }

private:
    [[nodiscard]] std::vector<std::size_t> sorted_items() const {
        std::vector<std::size_t> out(items_);
        std::ranges::sort(out);
        return out;
    }

    [[nodiscard]] double extent(std::size_t i) const noexcept {
        const Box& b = boxes_[i];
        return std::max({b.hi[0] - b.lo[0], b.hi[1] - b.lo[1], b.hi[2] - b.lo[2]});
    }

    [[nodiscard]] std::size_t octant(std::size_t i, const OctNode& node) const noexcept {
        const double half = node.size / 2.0;
        std::size_t o = 0;
        for (std::size_t k = 0; k < 3; ++k) {
            if (centres_[i][k] >= node.lo[k] + half) {
                o |= std::size_t{1} << k;
            }
        }
        return o;
    }

    [[nodiscard]] static bool splittable(const OctNode& node, int depth) noexcept {
        if (depth >= max_depth || !(node.size > 0.0)) {
            return false;
        }
        double magnitude = 0.0;
        for (std::size_t k = 0; k < 3; ++k) {
            magnitude =
                std::max({magnitude, std::abs(node.lo[k]), std::abs(node.lo[k] + node.size)});
        }
        return node.size / 2.0 > min_relative_cell * magnitude;
    }

    // Distributes items_[begin, end) over node `index` and its subtree. The
    // boxes kept in the node come first (input order preserved), the others
    // are grouped by octant with a stable partition.
    void build(std::size_t index, std::size_t begin, std::size_t end, int depth) {
        const OctNode node = nodes_[index];
        const auto first = items_.begin() + static_cast<std::ptrdiff_t>(begin);
        const auto last = items_.begin() + static_cast<std::ptrdiff_t>(end);
        const double child_size = node.size / 2.0;
        // Boxes too large for a child stay here.
        const auto movable_begin = std::stable_partition(
            first, last, [&](std::size_t i) { return !(extent(i) <= child_size / 2.0); });
        std::size_t kept = static_cast<std::size_t>(movable_begin - first);
        const std::size_t movable = end - begin - kept;
        if (movable <= leaf_capacity || !splittable(node, depth)) {
            kept = end - begin;
        }
        nodes_[index].first = begin;
        nodes_[index].count = kept;
        if (kept == end - begin) {
            return;
        }
        std::stable_sort(movable_begin, last, [&](std::size_t a, std::size_t b) {
            return octant(a, node) < octant(b, node);
        });
        std::size_t cursor = begin + kept;
        while (cursor < end) {
            const std::size_t o = octant(items_[cursor], node);
            std::size_t stop = cursor;
            while (stop < end && octant(items_[stop], node) == o) {
                ++stop;
            }
            OctNode child;
            child.size = child_size;
            for (std::size_t k = 0; k < 3; ++k) {
                child.lo[k] = node.lo[k] + ((o >> k) & 1U ? child_size : 0.0);
            }
            child.child.fill(no_child);
            const std::size_t child_index = nodes_.size();
            nodes_.push_back(child);
            nodes_[index].child[o] = child_index;
            build(child_index, cursor, stop, depth + 1);
            cursor = stop;
        }
    }

    const std::vector<Box>& boxes_;
    std::vector<std::size_t> items_;
    std::vector<std::array<double, 3>> centres_;
    std::vector<OctNode> nodes_;
};

// ---------------------------------------------------------------------------
// Narrow phase and resolution.
// ---------------------------------------------------------------------------

struct Contact {
    double step_fraction = 0.0;
    BodyId first;
    BodyId second;
};

auto contact_key(const Contact& c) noexcept {
    return std::tie(c.step_fraction, c.first, c.second);
}

Contact make_contact(double fraction, BodyId a, BodyId b) noexcept {
    return a < b ? Contact{fraction, a, b} : Contact{fraction, b, a};
}

std::vector<Contact> initial_contacts(const BodyStore& bodies, std::span<const LengthVec> start,
                                      std::span<const LengthVec> end) {
    const std::size_t n = bodies.size();
    std::vector<SweptSphere> spheres(n);
    for (std::size_t i = 0; i < n; ++i) {
        spheres[i] = {start[i], end[i], bodies.radius(i), collides(bodies, i)};
    }
    std::vector<Contact> contacts;
    for (const auto& [i, j] : broad_phase_pairs(spheres)) {
        const space::Length reach = bodies.radius(i) + bodies.radius(j);
        if (!(reach.value() > 0.0)) {
            continue;
        }
        const auto tau = swept_contact_fraction(start[j] - start[i], end[j] - end[i], reach);
        if (tau) {
            contacts.push_back(make_contact(*tau, bodies.id(i), bodies.id(j)));
        }
    }
    return contacts;
}

LengthVec at_fraction(const LengthVec& start, const LengthVec& end, double tau) noexcept {
    return start + (end - start) * tau;
}

void check_sizes(std::size_t bodies, std::size_t start, std::size_t end, const char* what) {
    if (start != bodies || end != bodies) {
        throw std::invalid_argument(what);
    }
}

} // namespace

std::optional<double> swept_contact_fraction(const LengthVec& p0, const LengthVec& p1,
                                             space::Length reach) noexcept {
    const double raw_reach = reach.value();
    if (!(raw_reach >= 0.0) || !isfinite(p0) || !isfinite(p1) || !std::isfinite(raw_reach)) {
        return std::nullopt;
    }
    const int e = scale_exponent({p0.x.value(), p0.y.value(), p0.z.value(), p1.x.value(),
                                  p1.y.value(), p1.z.value(), raw_reach});
    const Scaled a = scaled(p0, e);
    const Scaled b = scaled(p1, e);
    const double rr = std::ldexp(raw_reach, -e);
    const double a2 = dot(a, a);
    if (a2 <= rr * rr) {
        return 0.0;
    }
    const Scaled d{b.x - a.x, b.y - a.y, b.z - a.z};
    const double d2 = dot(d, d);
    // Distance travelled towards the point of closest approach.
    const double approach = -dot(a, d);
    if (!(d2 > 0.0) || !(approach > 0.0)) {
        return std::nullopt;
    }
    const double length = std::sqrt(d2);
    // Perpendicular distance of the line of the path, |a x d| / |d|.
    const double miss = cross_norm(a, b) / length;
    if (miss > rr) {
        return std::nullopt;
    }
    // Half chord of the sphere along the path, and the distance along the
    // path to the entry point,
    //   s = approach/|d| - half = (|a|^2 - R^2) / (approach/|d| + half),
    // written without cancellation (approach^2/|d|^2 = |a|^2 - miss^2).
    const double half = std::sqrt((rr - miss) * (rr + miss));
    const double na = std::sqrt(a2);
    const double entry = (na - rr) * (na + rr) / (approach / length + half);
    const double tau = entry / length;
    if (!(tau <= 1.0)) {
        return std::nullopt;
    }
    return tau;
}

ClosestApproach swept_closest_approach(const LengthVec& p0, const LengthVec& p1) noexcept {
    const int e = scale_exponent(
        {p0.x.value(), p0.y.value(), p0.z.value(), p1.x.value(), p1.y.value(), p1.z.value()});
    const Scaled a = scaled(p0, e);
    const Scaled b = scaled(p1, e);
    const Scaled d{b.x - a.x, b.y - a.y, b.z - a.z};
    const double dd = dot(d, d);
    double tau = 0.0;
    if (dd > 0.0) {
        tau = std::clamp(-dot(a, d) / dd, 0.0, 1.0);
    }
    double distance = 0.0;
    if (tau > 0.0 && tau < 1.0) {
        // Interior minimum: the perpendicular distance, free of cancellation.
        distance = cross_norm(a, b) / std::sqrt(dd);
    } else {
        const Scaled& p = tau > 0.0 ? b : a;
        distance = std::sqrt(dot(p, p));
    }
    return {tau, space::Length(std::ldexp(distance, e))};
}

std::size_t heavier_of(const BodyStore& bodies, std::size_t a, std::size_t b) {
    const double ma = bodies.mass(a).value();
    const double mb = bodies.mass(b).value();
    if (ma != mb) {
        return ma > mb ? a : b;
    }
    return bodies.id(a) < bodies.id(b) ? a : b;
}

LengthVec merged_start_position(const BodyStore& bodies, std::size_t survivor, std::size_t absorbed,
                                const LengthVec& survivor_start, const LengthVec& absorbed_start) {
    const double m1 = bodies.mass(survivor).value();
    const double m2 = bodies.mass(absorbed).value();
    const double total = m1 + m2;
    const double f = total > 0.0 ? m2 / total : 0.5;
    return survivor_start + (absorbed_start - survivor_start) * f;
}

// APPROXIMATION: perfectly inelastic point merge - colliding bodies coalesce
// into one body at their centre of mass with the centre-of-mass velocity; the
// relative kinetic energy is dissipated and the relative angular momentum
// (spin of the merger) is reported in the event but not carried by the point
// mass - valid for low-velocity accretion where fragmentation and ejecta are
// negligible; high-speed impacts (v_rel above the escape speed) are
// represented only by their conserved totals.
Result<MergeOutcome, BodyError> merge_bodies(BodyStore& bodies, std::size_t survivor,
                                             std::size_t absorbed, RadiusRule rule) {
    if (survivor == absorbed) {
        throw std::invalid_argument("merge_bodies: a body cannot absorb itself");
    }
    const double m1 = bodies.mass(survivor).value();
    const double m2 = bodies.mass(absorbed).value();
    const LengthVec r1 = bodies.position(survivor);
    const LengthVec r2 = bodies.position(absorbed);
    const Vec3<space::Velocity> v1 = bodies.velocity(survivor);
    const Vec3<space::Velocity> v2 = bodies.velocity(absorbed);
    const double total = m1 + m2;
    const double f = total > 0.0 ? m2 / total : 0.5;
    const double mu = total > 0.0 ? m1 * f : 0.0;

    const LengthVec r_rel = r2 - r1;
    const Vec3<space::Velocity> v_rel = v2 - v1;
    MergeOutcome out;
    out.survivor = bodies.id(survivor);
    out.absorbed = bodies.id(absorbed);
    out.survivor_mass_before = space::Mass(m1);
    out.absorbed_mass = space::Mass(m2);
    out.position = r1 + r_rel * f;
    out.velocity = v1 + v_rel * f;
    out.relative_speed = norm(v_rel);
    const double vrel = out.relative_speed.value();
    out.dissipated_energy = space::Energy(0.5 * mu * vrel * vrel);
    const auto l = cross(r_rel, v_rel);
    out.internal_angular_momentum = {space::AngularMomentum(mu * l.x.value()),
                                     space::AngularMomentum(mu * l.y.value()),
                                     space::AngularMomentum(mu * l.z.value())};

    double radius = bodies.radius(survivor).value();
    if (rule == RadiusRule::combine_volumes) {
        const double ra = bodies.radius(survivor).value();
        const double rb = bodies.radius(absorbed).value();
        const double big = std::max(ra, rb);
        if (big > 0.0) {
            const double ratio = std::min(ra, rb) / big;
            radius = big * std::cbrt(1.0 + ratio * ratio * ratio);
        }
    }

    if (!std::isfinite(total) || !isfinite(out.position) || !isfinite(out.velocity) ||
        !std::isfinite(radius)) {
        return failure(BodyError::non_finite_state);
    }
    // Validate everything before the first write (strong guarantee): the
    // survivor's kind accepts the summed mass because both masses are valid.
    if (const auto ok = bodies.set_mass(survivor, space::Mass(total)); !ok) {
        return failure(ok.error());
    }
    static_cast<void>(bodies.set_position(survivor, out.position));
    static_cast<void>(bodies.set_velocity(survivor, out.velocity));
    static_cast<void>(bodies.set_radius(survivor, space::Length(radius)));
    static_cast<void>(bodies.remove(out.absorbed));
    return out;
}

std::vector<std::pair<std::size_t, std::size_t>>
broad_phase_pairs(std::span<const SweptSphere> spheres) {
    return broad_phase_pairs(spheres, nullptr);
}

std::vector<std::pair<std::size_t, std::size_t>>
broad_phase_pairs(std::span<const SweptSphere> spheres, BroadPhaseStats* stats) {
    BroadPhaseStats local;
    BroadPhaseStats& st = stats != nullptr ? *stats : local;
    st = {};
    std::vector<std::pair<std::size_t, std::size_t>> pairs;
    std::vector<Box> boxes(spheres.size());
    std::vector<std::size_t> active;
    bool finite = true;
    for (std::size_t i = 0; i < spheres.size(); ++i) {
        if (!spheres[i].active) {
            continue;
        }
        boxes[i] = swept_box(spheres[i]);
        finite = finite && finite_box(boxes[i]);
        active.push_back(i);
    }
    if (active.size() < 2) {
        return pairs;
    }
    if (!finite) {
        // Infinite or NaN extents: exhaustive test in index order.
        for (std::size_t a = 0; a < active.size(); ++a) {
            for (std::size_t b = a + 1; b < active.size(); ++b) {
                ++st.box_tests;
                if (overlap(boxes[active[a]], boxes[active[b]])) {
                    pairs.emplace_back(active[a], active[b]);
                }
            }
        }
        return pairs;
    }
    const LooseOctree tree(boxes, std::move(active));
    tree.collect(pairs, st);
    std::ranges::sort(pairs);
    return pairs;
}

std::optional<double> earliest_collision_fraction(const BodyStore& bodies,
                                                  std::span<const LengthVec> start_positions,
                                                  std::span<const LengthVec> end_positions) {
    check_sizes(bodies.size(), start_positions.size(), end_positions.size(),
                "earliest_collision_fraction: one start and end position per body is required");
    const std::vector<Contact> contacts = initial_contacts(bodies, start_positions, end_positions);
    std::optional<double> earliest;
    for (const Contact& c : contacts) {
        if (!earliest || c.step_fraction < *earliest) {
            earliest = c.step_fraction;
        }
    }
    return earliest;
}

std::size_t resolve_collisions(BodyStore& bodies, std::vector<LengthVec>& start_positions,
                               space::Time t_start, space::Time dt, EventLog& log) {
    if (start_positions.size() != bodies.size()) {
        throw std::invalid_argument("resolve_collisions: one start position per body is required");
    }
    std::vector<LengthVec> end_positions(bodies.size());
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        end_positions[i] = bodies.position(i);
    }
    std::vector<Contact> contacts = initial_contacts(bodies, start_positions, end_positions);

    std::size_t merges = 0;
    while (!contacts.empty()) {
        const auto next =
            std::ranges::min_element(contacts, [](const Contact& a, const Contact& b) {
                return contact_key(a) < contact_key(b);
            });
        const Contact c = *next;
        contacts.erase(next);
        const auto ia = bodies.index_of(c.first);
        const auto ib = bodies.index_of(c.second);
        if (!ia || !ib) {
            continue;
        }
        const std::size_t s = heavier_of(bodies, *ia, *ib);
        const std::size_t a = s == *ia ? *ib : *ia;
        const LengthVec start_s =
            merged_start_position(bodies, s, a, start_positions[s], start_positions[a]);
        const auto merged = merge_bodies(bodies, s, a, RadiusRule::combine_volumes);
        if (!merged) {
            continue;
        }
        const MergeOutcome& m = merged.value();
        // The survivor's swept path becomes the merged centre of mass, linear
        // from its start to its end position; for tau >= c.step_fraction it
        // is the path of the merged body.
        start_positions[s] = start_s;
        start_positions.erase(start_positions.begin() + static_cast<std::ptrdiff_t>(a));
        log.append(CollisionEvent{t_start + c.step_fraction * dt, c.step_fraction, m.survivor,
                                  m.absorbed, m.survivor_mass_before, m.absorbed_mass, m.position,
                                  m.relative_speed, m.dissipated_energy,
                                  m.internal_angular_momentum});
        ++merges;

        // Contacts of both participants are stale: drop them and sweep the
        // survivor (new mass, radius, path) over the rest of the step.
        std::erase_if(contacts, [&](const Contact& x) {
            return x.first == m.survivor || x.second == m.survivor || x.first == m.absorbed ||
                   x.second == m.absorbed;
        });
        const auto si = bodies.index_of(m.survivor);
        if (!si) {
            continue;
        }
        const double tau = c.step_fraction;
        const LengthVec survivor_at = at_fraction(start_positions[*si], bodies.position(*si), tau);
        for (std::size_t j = 0; j < bodies.size(); ++j) {
            if (j == *si || !collides(bodies, j)) {
                continue;
            }
            const space::Length reach = bodies.radius(*si) + bodies.radius(j);
            if (!(reach.value() > 0.0)) {
                continue;
            }
            const LengthVec other_at = at_fraction(start_positions[j], bodies.position(j), tau);
            const auto sigma = swept_contact_fraction(
                other_at - survivor_at, bodies.position(j) - bodies.position(*si), reach);
            if (sigma) {
                const double fraction = std::min(1.0, tau + *sigma * (1.0 - tau));
                contacts.push_back(make_contact(fraction, m.survivor, bodies.id(j)));
            }
        }
    }
    return merges;
}

} // namespace orrery::core
