#include "orrery/core/octree.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>

namespace orrery::core {

namespace {

constexpr std::uint32_t cells_per_axis = std::uint32_t{1} << Octree::max_depth;

// Spreads the low 21 bits of v so that bit k moves to bit 3k.
std::uint64_t spread_bits(std::uint32_t v) noexcept {
    std::uint64_t x = v & 0x1fffffU;
    x = (x | (x << 32U)) & 0x001f00000000ffffULL;
    x = (x | (x << 16U)) & 0x001f0000ff0000ffULL;
    x = (x | (x << 8U)) & 0x100f00f00f00f00fULL;
    x = (x | (x << 4U)) & 0x10c30c30c30c30c3ULL;
    x = (x | (x << 2U)) & 0x1249249249249249ULL;
    return x;
}

// Inverse of spread_bits.
std::uint32_t compact_bits(std::uint64_t x) noexcept {
    x &= 0x1249249249249249ULL;
    x = (x | (x >> 2U)) & 0x10c30c30c30c30c3ULL;
    x = (x | (x >> 4U)) & 0x100f00f00f00f00fULL;
    x = (x | (x >> 8U)) & 0x001f0000ff0000ffULL;
    x = (x | (x >> 16U)) & 0x001f00000000ffffULL;
    x = (x | (x >> 32U)) & 0x1fffffULL;
    return static_cast<std::uint32_t>(x);
}

// Cell index of a coordinate: floor((v - lo) * scale), clamped to the grid.
// scale is a power of two, so the product is exact and only the subtraction
// rounds; a value that rounds onto the upper face is clamped into the last
// cell.
std::uint32_t quantise(double v, double lo, double scale) noexcept {
    const double cell = std::floor((v - lo) * scale);
    if (!(cell > 0.0)) {
        return 0;
    }
    if (cell >= static_cast<double>(cells_per_axis - 1)) {
        return cells_per_axis - 1;
    }
    return static_cast<std::uint32_t>(cell);
}

} // namespace

std::uint64_t Octree::morton_key(std::uint32_t ix, std::uint32_t iy, std::uint32_t iz) noexcept {
    return spread_bits(ix) | (spread_bits(iy) << 1U) | (spread_bits(iz) << 2U);
}

void Octree::clear() noexcept {
    nodes_.clear();
    x_.clear();
    y_.clear();
    z_.clear();
    gm_.clear();
    index_.clear();
    keys_.clear();
    size_ = 0.0;
}

bool Octree::build(std::span<const Vec3<space::Length>> positions, std::span<const double> gm,
                   std::span<const std::uint32_t> source_index, std::size_t leaf_capacity) {
    if (leaf_capacity == 0) {
        throw std::invalid_argument("Octree::build: leaf capacity must be at least 1");
    }
    if (source_index.size() > std::size_t{std::numeric_limits<std::uint32_t>::max() / 2}) {
        throw std::length_error("Octree::build: too many sources");
    }
    clear();
    const std::size_t n = source_index.size();
    if (n == 0) {
        return true;
    }

    // Bounding box, reduced sequentially in input order.
    double lo_x = std::numeric_limits<double>::infinity();
    double lo_y = lo_x;
    double lo_z = lo_x;
    // The three upper bounds start from the same value on purpose.
    // cppcheck-suppress duplicateAssignExpression
    double hi_x = -lo_x;
    double hi_y = -lo_x;
    double hi_z = -lo_x;
    for (const std::uint32_t s : source_index) {
        const Vec3<space::Length>& p = positions[s];
        const double px = p.x.value();
        const double py = p.y.value();
        const double pz = p.z.value();
        if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
            return false;
        }
        lo_x = std::min(lo_x, px);
        lo_y = std::min(lo_y, py);
        lo_z = std::min(lo_z, pz);
        hi_x = std::max(hi_x, px);
        hi_y = std::max(hi_y, py);
        hi_z = std::max(hi_z, pz);
    }
    const double extent = std::max({hi_x - lo_x, hi_y - lo_y, hi_z - lo_z});
    if (!std::isfinite(extent)) {
        return false;
    }
    // Smallest power of two strictly above the extent: extent = f 2^e with
    // f in [0.5, 1), so 2^e > extent.
    int e = 1;
    if (extent > 0.0) {
        static_cast<void>(std::frexp(extent, &e));
    }
    const double size = std::ldexp(1.0, e);
    const double scale = std::ldexp(1.0, max_depth - e);
    if (!std::isfinite(size) || !(scale > 0.0) || !std::isfinite(scale)) {
        return false;
    }
    lo_x_ = lo_x;
    lo_y_ = lo_y;
    lo_z_ = lo_z;
    size_ = size;

    // Keys, computed independently per source.
    std::vector<std::pair<std::uint64_t, std::uint32_t>> order(n);
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n, 1024),
                      [&](const tbb::blocked_range<std::size_t>& range) {
                          for (std::size_t k = range.begin(); k != range.end(); ++k) {
                              const Vec3<space::Length>& p = positions[source_index[k]];
                              const std::uint64_t key =
                                  morton_key(quantise(p.x.value(), lo_x, scale),
                                             quantise(p.y.value(), lo_y, scale),
                                             quantise(p.z.value(), lo_z, scale));
                              order[k] = {key, static_cast<std::uint32_t>(k)};
                          }
                      });
    // (key, input position) pairs are unique: the result does not depend on
    // the sorting algorithm or on the number of threads.
    tbb::parallel_sort(order.begin(), order.end());

    x_.resize(n);
    y_.resize(n);
    z_.resize(n);
    gm_.resize(n);
    index_.resize(n);
    keys_.resize(n);
    for (std::size_t k = 0; k < n; ++k) {
        const std::uint32_t s = source_index[order[k].second];
        x_[k] = positions[s].x.value();
        y_[k] = positions[s].y.value();
        z_[k] = positions[s].z.value();
        gm_[k] = gm[s];
        index_[k] = s;
        keys_[k] = order[k].first;
    }
    nodes_.reserve(2 * (n / leaf_capacity + 1));
    static_cast<void>(build_node(0, static_cast<std::uint32_t>(n), 0, leaf_capacity));
    return true;
}

std::uint32_t Octree::build_node(std::uint32_t first, std::uint32_t count, int level,
                                 std::size_t leaf_capacity) {
    const auto self = static_cast<std::uint32_t>(nodes_.size());
    nodes_.emplace_back();
    {
        Node& node = nodes_[self];
        node.first = first;
        node.count = count;
        node.leaf = count <= leaf_capacity || level == max_depth;
        // Cell of this node: the key prefix shared by its sources.
        const auto shift = static_cast<unsigned>(3 * (max_depth - level));
        const std::uint64_t prefix = level == 0 ? 0 : (keys_[first] >> shift);
        const double side = std::ldexp(size_, -level);
        node.side = side;
        // Cell centre; consumed by the offset once the centre of mass is known.
        node.com_x = lo_x_ + (static_cast<double>(compact_bits(prefix)) + 0.5) * side;
        node.com_y = lo_y_ + (static_cast<double>(compact_bits(prefix >> 1U)) + 0.5) * side;
        node.com_z = lo_z_ + (static_cast<double>(compact_bits(prefix >> 2U)) + 0.5) * side;
    }
    const double centre_x = nodes_[self].com_x;
    const double centre_y = nodes_[self].com_y;
    const double centre_z = nodes_[self].com_z;

    if (nodes_[self].leaf) {
        leaf_moments(nodes_[self]);
    } else {
        // Children: the non-empty octants of the next level, in order.
        const auto child_shift = static_cast<unsigned>(3 * (max_depth - level - 1));
        std::uint32_t begin = first;
        const std::uint32_t end = first + count;
        while (begin < end) {
            const std::uint64_t octant = (keys_[begin] >> child_shift) & 7U;
            std::uint32_t stop = begin + 1;
            while (stop < end && ((keys_[stop] >> child_shift) & 7U) == octant) {
                ++stop;
            }
            static_cast<void>(build_node(begin, stop - begin, level + 1, leaf_capacity));
            begin = stop;
        }
        internal_moments(nodes_[self], self + 1);
    }
    Node& node = nodes_[self];
    node.next = static_cast<std::uint32_t>(nodes_.size());
    const double ox = node.com_x - centre_x;
    const double oy = node.com_y - centre_y;
    const double oz = node.com_z - centre_z;
    node.offset = std::sqrt(ox * ox + oy * oy + oz * oz);
    return self;
}

void Octree::leaf_moments(Node& node) const {
    const std::uint32_t end = node.first + node.count;
    double gm = 0.0;
    double mx = 0.0;
    double my = 0.0;
    double mz = 0.0;
    for (std::uint32_t k = node.first; k < end; ++k) {
        gm += gm_[k];
        mx += gm_[k] * x_[k];
        my += gm_[k] * y_[k];
        mz += gm_[k] * z_[k];
    }
    node.gm = gm;
    node.com_x = mx / gm;
    node.com_y = my / gm;
    node.com_z = mz / gm;
    std::array<double, 6> q{};
    for (std::uint32_t k = node.first; k < end; ++k) {
        const double dx = x_[k] - node.com_x;
        const double dy = y_[k] - node.com_y;
        const double dz = z_[k] - node.com_z;
        q[0] += gm_[k] * dx * dx;
        q[1] += gm_[k] * dx * dy;
        q[2] += gm_[k] * dx * dz;
        q[3] += gm_[k] * dy * dy;
        q[4] += gm_[k] * dy * dz;
        q[5] += gm_[k] * dz * dz;
    }
    node.second = q;
}

void Octree::internal_moments(Node& node, std::uint32_t first_child) const {
    // Children are consecutive subtrees starting right after the node.
    const auto end = static_cast<std::uint32_t>(nodes_.size());
    double gm = 0.0;
    double mx = 0.0;
    double my = 0.0;
    double mz = 0.0;
    for (std::uint32_t c = first_child; c < end; c = nodes_[c].next) {
        const Node& child = nodes_[c];
        gm += child.gm;
        mx += child.gm * child.com_x;
        my += child.gm * child.com_y;
        mz += child.gm * child.com_z;
    }
    node.gm = gm;
    node.com_x = mx / gm;
    node.com_y = my / gm;
    node.com_z = mz / gm;
    // Parallel-axis theorem: Q = sum_c (Q_c + G m_c d_c d_c^T), d_c = c_c - c.
    std::array<double, 6> q{};
    for (std::uint32_t c = first_child; c < end; c = nodes_[c].next) {
        const Node& child = nodes_[c];
        const double dx = child.com_x - node.com_x;
        const double dy = child.com_y - node.com_y;
        const double dz = child.com_z - node.com_z;
        q[0] += child.second[0] + child.gm * dx * dx;
        q[1] += child.second[1] + child.gm * dx * dy;
        q[2] += child.second[2] + child.gm * dx * dz;
        q[3] += child.second[3] + child.gm * dy * dy;
        q[4] += child.second[4] + child.gm * dy * dz;
        q[5] += child.second[5] + child.gm * dz * dz;
    }
    node.second = q;
}

} // namespace orrery::core
