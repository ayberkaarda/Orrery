#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "orrery/core/body_store.h"
#include "orrery/core/conservation.h"
#include "orrery/core/event.h"
#include "orrery/core/quantity.h"
#include "orrery/core/result.h"
#include "orrery/core/vec3.h"

namespace orrery::core {

// ---------------------------------------------------------------------------
// Swept geometry within one step.
//
// APPROXIMATION: linear relative motion within a step - between the start and
// end positions of a step the separation of two bodies is interpolated
// linearly, p(tau) = p0 + tau (p1 - p0), tau in [0, 1] - valid when the step
// is a small fraction of the encounter time (the relative path curves little
// within one step); a grazing contact on a strongly curved path can be missed
// or its time misestimated by up to one step. EventStepper applies the swept
// tests per accepted integration segment (each internal IAS15 step), not per
// requested output interval.
// ---------------------------------------------------------------------------

// Earliest tau in [0, 1] with |p0 + tau (p1 - p0)| <= reach, given the
// relative positions p0 (start of step) and p1 (end of step). 0 when the
// bodies already overlap at the start; empty when they never come within
// reach on the swept path. Scale-invariant and overflow-safe: the inputs are
// rescaled by a power of two before squaring. Free of catastrophic
// cancellation: the miss distance of the line is |p0 x p1| / |p1 - p0| with
// each cross-product component formed by fused multiply-add (a few ulps even
// when its two products nearly cancel), and the entry distance along the path
// is (|p0|^2 - R^2) / (approach + half chord), so the result is decided by the
// perpendicular distance to a few ulps instead of by a difference of nearly
// equal squares.
[[nodiscard]] std::optional<double> swept_contact_fraction(const Vec3<space::Length>& p0,
                                                           const Vec3<space::Length>& p1,
                                                           space::Length reach) noexcept;

struct ClosestApproach {
    double step_fraction = 0.0;
    space::Length distance{};
};

// Point of the swept segment p0 -> p1 closest to the origin.
[[nodiscard]] ClosestApproach swept_closest_approach(const Vec3<space::Length>& p0,
                                                     const Vec3<space::Length>& p1) noexcept;

// ---------------------------------------------------------------------------
// Inelastic merge.
// ---------------------------------------------------------------------------

struct MergeOutcome {
    BodyId survivor;
    BodyId absorbed;
    space::Mass survivor_mass_before{};
    space::Mass absorbed_mass{};
    Vec3<space::Length> position{};
    Vec3<space::Velocity> velocity{};
    space::Velocity relative_speed{};
    space::Energy dissipated_energy{};
    Vec3<space::AngularMomentum> internal_angular_momentum{};
};

enum class RadiusRule : std::uint8_t {
    // Equal-density merge: R = (R_1^3 + R_2^3)^(1/3).
    combine_volumes,
    // The survivor keeps its radius (horizon absorption).
    keep_survivor,
};

// Merges body `absorbed` into body `survivor` (indices of `bodies`) and
// removes the absorbed body. The survivor keeps its id, name and kind; its
// mass becomes m1 + m2, its position the centre of mass
//   R = r1 + f (r2 - r1),  f = m2 / (m1 + m2),
// and its velocity the centre-of-mass velocity V = v1 + f (v2 - v1), so total
// mass, linear momentum and the centre of mass are conserved to rounding.
// (Two massless bodies merge at their midpoint with the mean velocity.)
// Throws std::out_of_range for invalid indices and std::invalid_argument when
// they are equal. If a merged value is not finite (overflow) the store is left
// unchanged and the error is returned.
[[nodiscard]] Result<MergeOutcome, BodyError> merge_bodies(BodyStore& bodies, std::size_t survivor,
                                                           std::size_t absorbed, RadiusRule rule);

// Survivor of a collision: the heavier body; on equal masses the lower id.
// Returns the index of the survivor among {a, b}.
[[nodiscard]] std::size_t heavier_of(const BodyStore& bodies, std::size_t a, std::size_t b);

// ---------------------------------------------------------------------------
// Broad phase.
// ---------------------------------------------------------------------------

// A sphere swept from `start` to `end` during the step. Inactive spheres take
// no part in the broad phase.
struct SweptSphere {
    Vec3<space::Length> start{};
    Vec3<space::Length> end{};
    space::Length radius{};
    bool active = false;
};

// Every pair (i, j), i < j, of active spheres whose swept axis-aligned
// bounding boxes overlap (touching counts), in ascending (i, j) order.
//
// Loose octree (spec 3.5): the root cell is the bounding cube of the box
// centres; a node of size S keeps the boxes whose largest side exceeds S / 4
// and passes every other box to the child octant holding its centre, until a
// node holds at most 8 movable boxes (or 64 levels, or a cell near the
// rounding resolution of its coordinates). The loose bounds of a node - its
// cell widened by S / 2 on every side - contain every box stored in its
// subtree, so a query descends only into children whose loose bounds overlap
// the query box. Each box is queried from the root and tested against the
// boxes of higher index stored in the visited nodes. Box sizes are handled
// per level, so one large or distant box does not coarsen the cells of the
// others: for N boxes of comparable size the work is O(N log N). Children are
// built with stable partitions and visited in fixed octant order and the
// result is sorted: no hash containers, no iteration-order dependence.
// Non-finite boxes fall back to the exhaustive test.
[[nodiscard]] std::vector<std::pair<std::size_t, std::size_t>>
broad_phase_pairs(std::span<const SweptSphere> spheres);

// Work counters of one broad-phase call.
struct BroadPhaseStats {
    // Pairwise box-overlap tests performed.
    std::size_t box_tests = 0;
    // Box against loose-node-bounds tests performed.
    std::size_t node_tests = 0;
    // Octree nodes built.
    std::size_t nodes = 0;
};

// Same result as broad_phase_pairs(spheres); also fills `stats` if non-null.
[[nodiscard]] std::vector<std::pair<std::size_t, std::size_t>>
broad_phase_pairs(std::span<const SweptSphere> spheres, BroadPhaseStats* stats);

// ---------------------------------------------------------------------------
// Collision phase of a step.
// ---------------------------------------------------------------------------

// Detects and resolves the collisions of one step and appends one
// CollisionEvent per merge.
//
// Participants: every body that is neither a black hole (absorption is the
// horizon phase) nor a probe (impacts belong to the entry phases). A pair
// collides when the swept separation comes within R_i + R_j > 0 (two points
// never collide). `start_positions` holds the position of each body of
// `bodies` at t_start (same order); entries of removed bodies are erased so
// it stays aligned. Merges use the end-of-step state and
// RadiusRule::combine_volumes.
//
// Order: the earliest contact is resolved first, ties in ascending (lower id,
// higher id). The survivor's start position is then replaced by the start of
// the merged centre-of-mass path (merged_start_position), every pending
// contact of either participant is dropped, and the survivor - new mass,
// radius, position and velocity - is swept again against every other
// participant over the remaining fraction [tau, 1] of the step. This repeats
// until no contact is left, so a survivor that passes through a further body
// later in the step merges with it (chain collisions), and three or more
// bodies overlapping at once all merge. Every merge removes a body, so the
// loop ends. Returns the number of merges.
std::size_t resolve_collisions(BodyStore& bodies, std::vector<Vec3<space::Length>>& start_positions,
                               space::Time t_start, space::Time dt, EventLog& log);

// Earliest contact step fraction of the collision phase for bodies moving
// from `start_positions` to `end_positions` (one each per body of `bodies`,
// same order; kinds and radii from `bodies`), without changing anything;
// empty if resolve_collisions would merge nothing. Throws
// std::invalid_argument on a size mismatch.
[[nodiscard]] std::optional<double>
earliest_collision_fraction(const BodyStore& bodies,
                            std::span<const Vec3<space::Length>> start_positions,
                            std::span<const Vec3<space::Length>> end_positions);

// Start of the merged centre-of-mass path of a merge of `absorbed` into
// `survivor` (indices of `bodies`, masses before the merge): the start
// positions weighted as merge_bodies weights the end positions. With linear
// motion the merged body moves on the line from this point to its merged end
// position.
[[nodiscard]] Vec3<space::Length> merged_start_position(const BodyStore& bodies,
                                                        std::size_t survivor, std::size_t absorbed,
                                                        const Vec3<space::Length>& survivor_start,
                                                        const Vec3<space::Length>& absorbed_start);

} // namespace orrery::core
