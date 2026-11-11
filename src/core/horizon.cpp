#include "orrery/core/horizon.h"

#include <algorithm>
#include <stdexcept>
#include <tuple>
#include <variant>

#include "orrery/core/black_hole.h"
#include "orrery/core/collision.h"

namespace orrery::core {

namespace {

using LengthVec = Vec3<space::Length>;

bool is_black_hole(const BodyStore& bodies, std::size_t i) {
    return std::holds_alternative<BlackHole>(bodies.kind(i));
}

struct Crossing {
    double step_fraction = 0.0;
    BodyId first;
    BodyId second;
};

auto crossing_key(const Crossing& c) noexcept {
    return std::tie(c.step_fraction, c.first, c.second);
}

Crossing make_crossing(double fraction, BodyId a, BodyId b) noexcept {
    return a < b ? Crossing{fraction, a, b} : Crossing{fraction, b, a};
}

// Absorption threshold of a pair with at least one black hole.
space::Length threshold(const BodyStore& bodies, std::size_t i, std::size_t j) {
    space::Length r{};
    if (is_black_hole(bodies, i)) {
        r += schwarzschild_radius(bodies.mass(i));
    }
    if (is_black_hole(bodies, j)) {
        r += schwarzschild_radius(bodies.mass(j));
    }
    return r;
}

// Every pair with at least one black hole whose swept relative path comes
// within the threshold, for the motion start -> end over the whole step.
std::vector<Crossing> initial_crossings(const BodyStore& bodies, std::span<const LengthVec> start,
                                        std::span<const LengthVec> end) {
    const std::size_t n = bodies.size();
    std::vector<Crossing> crossings;
    for (std::size_t b = 0; b < n; ++b) {
        if (!is_black_hole(bodies, b)) {
            continue;
        }
        for (std::size_t j = 0; j < n; ++j) {
            // A pair of holes is visited once, from its lower index.
            if (j == b || (is_black_hole(bodies, j) && j < b)) {
                continue;
            }
            const auto tau = swept_contact_fraction(start[j] - start[b], end[j] - end[b],
                                                    threshold(bodies, b, j));
            if (tau) {
                crossings.push_back(make_crossing(*tau, bodies.id(b), bodies.id(j)));
            }
        }
    }
    return crossings;
}

LengthVec at_fraction(const LengthVec& start, const LengthVec& end, double tau) noexcept {
    return start + (end - start) * tau;
}

} // namespace

std::optional<double> earliest_horizon_fraction(const BodyStore& bodies,
                                                std::span<const LengthVec> start_positions,
                                                std::span<const LengthVec> end_positions) {
    if (start_positions.size() != bodies.size() || end_positions.size() != bodies.size()) {
        throw std::invalid_argument(
            "earliest_horizon_fraction: one start and end position per body is required");
    }
    std::optional<double> earliest;
    for (const Crossing& c : initial_crossings(bodies, start_positions, end_positions)) {
        if (!earliest || c.step_fraction < *earliest) {
            earliest = c.step_fraction;
        }
    }
    return earliest;
}

std::size_t absorb_horizon_crossings(BodyStore& bodies, std::vector<LengthVec>& start_positions,
                                     space::Time t_start, space::Time dt, EventLog& log) {
    if (start_positions.size() != bodies.size()) {
        throw std::invalid_argument(
            "absorb_horizon_crossings: one start position per body is required");
    }
    std::vector<LengthVec> end_positions(bodies.size());
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        end_positions[i] = bodies.position(i);
    }
    std::vector<Crossing> crossings = initial_crossings(bodies, start_positions, end_positions);

    std::size_t absorbed_count = 0;
    while (!crossings.empty()) {
        const auto next =
            std::ranges::min_element(crossings, [](const Crossing& a, const Crossing& b) {
                return crossing_key(a) < crossing_key(b);
            });
        const Crossing c = *next;
        crossings.erase(next);
        const auto ia = bodies.index_of(c.first);
        const auto ib = bodies.index_of(c.second);
        if (!ia || !ib) {
            continue;
        }
        std::size_t hole = 0;
        if (is_black_hole(bodies, *ia) && is_black_hole(bodies, *ib)) {
            hole = heavier_of(bodies, *ia, *ib);
        } else {
            hole = is_black_hole(bodies, *ia) ? *ia : *ib;
        }
        const std::size_t victim = hole == *ia ? *ib : *ia;
        const space::Length horizon = threshold(bodies, *ia, *ib);
        const LengthVec hole_start = merged_start_position(
            bodies, hole, victim, start_positions[hole], start_positions[victim]);
        const auto merged = merge_bodies(bodies, hole, victim, RadiusRule::keep_survivor);
        if (!merged) {
            continue;
        }
        start_positions[hole] = hole_start;
        start_positions.erase(start_positions.begin() + static_cast<std::ptrdiff_t>(victim));
        const MergeOutcome& m = merged.value();
        log.append(
            HorizonCrossingEvent{t_start + c.step_fraction * dt, c.step_fraction, m.survivor,
                                 m.absorbed, m.survivor_mass_before, m.absorbed_mass, horizon,
                                 schwarzschild_radius(m.survivor_mass_before + m.absorbed_mass),
                                 m.position, m.internal_angular_momentum});
        ++absorbed_count;

        // The hole moved to the merged centre-of-mass path and its horizon
        // grew: drop its pending crossings and sweep it again over the rest
        // of the step against every other body.
        std::erase_if(crossings, [&](const Crossing& x) {
            return x.first == m.survivor || x.second == m.survivor || x.first == m.absorbed ||
                   x.second == m.absorbed;
        });
        const auto hi = bodies.index_of(m.survivor);
        if (!hi) {
            continue;
        }
        const double tau = c.step_fraction;
        const LengthVec hole_at = at_fraction(start_positions[*hi], bodies.position(*hi), tau);
        for (std::size_t j = 0; j < bodies.size(); ++j) {
            if (j == *hi) {
                continue;
            }
            const LengthVec other_at = at_fraction(start_positions[j], bodies.position(j), tau);
            const auto sigma = swept_contact_fraction(other_at - hole_at,
                                                      bodies.position(j) - bodies.position(*hi),
                                                      threshold(bodies, *hi, j));
            if (sigma) {
                const double fraction = std::min(1.0, tau + *sigma * (1.0 - tau));
                crossings.push_back(make_crossing(fraction, m.survivor, bodies.id(j)));
            }
        }
    }
    return absorbed_count;
}

} // namespace orrery::core
