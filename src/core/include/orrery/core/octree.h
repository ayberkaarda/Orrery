#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "orrery/core/quantity.h"
#include "orrery/core/vec3.h"

namespace orrery::core {

// Morton-ordered octree of point sources for the Barnes-Hut force model
// (space units: AU, and gravitational parameters G m in AU^3 day^-2).
//
// Build:
//   1. The bounding cube of the sources has its lower corner at the
//      component-wise minimum and a side equal to the smallest power of two
//      strictly above the largest extent (1 AU if every source coincides).
//   2. Each source is quantised to 21 bits per axis and the bits are
//      interleaved into a 63-bit Morton (Z-order) key, x in the lowest bit of
//      each triple.
//   3. Sources are sorted by (key, input position). The pair is unique, so the
//      sorted order is unique and independent of the sorting algorithm and of
//      the number of threads.
//   4. Nodes are created top-down in pre-order: a node whose range holds more
//      than `leaf_capacity` sources (and is above depth 21) is split into its
//      non-empty octants in octant order 0..7.
//   5. Moments are accumulated bottom-up in that fixed order: total G m,
//      centre of mass and second moments sum G m (x - c)_i (x - c)_j about it.
//
// Every value depends only on the source positions and masses and on their
// input order, never on scheduling, so a tree is reproducible bit for bit.
class Octree {
public:
    static constexpr int max_depth = 21;

    struct Node {
        // Centre of mass (AU) and total gravitational parameter G M.
        double com_x = 0.0;
        double com_y = 0.0;
        double com_z = 0.0;
        double gm = 0.0;
        // Second moments about the centre of mass, scaled by G:
        // xx, xy, xz, yy, yz, zz (AU^5 day^-2).
        std::array<double, 6> second{};
        // Side of the Morton cell (AU) and distance from the cell centre to
        // the centre of mass (AU); used by the opening criterion.
        double side = 0.0;
        double offset = 0.0;
        // Range of this node's sources in sorted order.
        std::uint32_t first = 0;
        std::uint32_t count = 0;
        // Pre-order index of the first node after this subtree.
        std::uint32_t next = 0;
        bool leaf = false;
    };

    // Builds the tree over the sources listed in `source_index` (indices into
    // `positions` and `gm`, any order; every gm must be positive). Returns
    // false, and leaves the tree empty, if a source position is not finite
    // or the bounding box overflows. leaf_capacity must be at least 1.
    bool build(std::span<const Vec3<space::Length>> positions, std::span<const double> gm,
               std::span<const std::uint32_t> source_index, std::size_t leaf_capacity);

    [[nodiscard]] bool empty() const noexcept { return nodes_.empty(); }
    [[nodiscard]] std::span<const Node> nodes() const noexcept { return nodes_; }

    // Sorted sources: position, G m and the index passed to build().
    [[nodiscard]] std::span<const double> x() const noexcept { return x_; }
    [[nodiscard]] std::span<const double> y() const noexcept { return y_; }
    [[nodiscard]] std::span<const double> z() const noexcept { return z_; }
    [[nodiscard]] std::span<const double> gm() const noexcept { return gm_; }
    [[nodiscard]] std::span<const std::uint32_t> index() const noexcept { return index_; }
    // Morton keys of the sorted sources (non-decreasing).
    [[nodiscard]] std::span<const std::uint64_t> keys() const noexcept { return keys_; }

    // Interleaves the low 21 bits of each coordinate (x in bit 0).
    [[nodiscard]] static std::uint64_t morton_key(std::uint32_t ix, std::uint32_t iy,
                                                  std::uint32_t iz) noexcept;

private:
    void clear() noexcept;
    std::uint32_t build_node(std::uint32_t first, std::uint32_t count, int level,
                             std::size_t leaf_capacity);
    void leaf_moments(Node& node) const;
    void internal_moments(Node& node, std::uint32_t first_child) const;

    double lo_x_ = 0.0;
    double lo_y_ = 0.0;
    double lo_z_ = 0.0;
    double size_ = 0.0;
    std::vector<Node> nodes_;
    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> z_;
    std::vector<double> gm_;
    std::vector<std::uint32_t> index_;
    std::vector<std::uint64_t> keys_;
};

} // namespace orrery::core
