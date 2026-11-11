#include "orrery/core/tidal.h"

#include <algorithm>
#include <stdexcept>
#include <tuple>
#include <variant>
#include <vector>

#include "orrery/core/black_hole.h"
#include "orrery/core/collision.h"

namespace orrery::core {

namespace {

// Every tidal entry of the motion start -> end (unsorted).
std::vector<DisruptionEvent> tidal_entries(const BodyStore& bodies,
                                           std::span<const Vec3<space::Length>> start,
                                           std::span<const Vec3<space::Length>> end,
                                           space::Time t_start, space::Time dt) {
    const std::size_t n = bodies.size();
    std::vector<DisruptionEvent> found;
    for (std::size_t b = 0; b < n; ++b) {
        if (!std::holds_alternative<BlackHole>(bodies.kind(b))) {
            continue;
        }
        const space::Mass hole_mass = bodies.mass(b);
        const space::Length rs = schwarzschild_radius(hole_mass);
        for (std::size_t i = 0; i < n; ++i) {
            const BodyKind& kind = bodies.kind(i);
            if (std::holds_alternative<BlackHole>(kind) || std::holds_alternative<Probe>(kind)) {
                continue;
            }
            const auto rt = tidal_radius(bodies.radius(i), bodies.mass(i), hole_mass);
            if (!rt || *rt <= rs) {
                continue;
            }
            const Vec3<space::Length> p0 = start[i] - start[b];
            const Vec3<space::Length> p1 = end[i] - end[b];
            if (norm(p0) < *rt) {
                continue;
            }
            const auto tau = swept_contact_fraction(p0, p1, *rt);
            if (!tau) {
                continue;
            }
            const ClosestApproach closest = swept_closest_approach(p0, p1);
            found.push_back(DisruptionEvent{t_start + *tau * dt, *tau, bodies.id(i), bodies.id(b),
                                            *rt, closest.distance});
        }
    }
    return found;
}

} // namespace

void detect_tidal_disruptions(const BodyStore& bodies,
                              std::span<const Vec3<space::Length>> start_positions,
                              space::Time t_start, space::Time dt, EventLog& log) {
    if (start_positions.size() != bodies.size()) {
        throw std::invalid_argument(
            "detect_tidal_disruptions: one start position per body is required");
    }
    std::vector<Vec3<space::Length>> end_positions(bodies.size());
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        end_positions[i] = bodies.position(i);
    }
    std::vector<DisruptionEvent> found =
        tidal_entries(bodies, start_positions, end_positions, t_start, dt);
    std::ranges::sort(found, [](const DisruptionEvent& a, const DisruptionEvent& c) {
        return std::tie(a.step_fraction, a.body, a.black_hole) <
               std::tie(c.step_fraction, c.body, c.black_hole);
    });
    for (DisruptionEvent& e : found) {
        log.append(e);
    }
}

std::optional<double> earliest_tidal_fraction(const BodyStore& bodies,
                                              std::span<const Vec3<space::Length>> start_positions,
                                              std::span<const Vec3<space::Length>> end_positions) {
    if (start_positions.size() != bodies.size() || end_positions.size() != bodies.size()) {
        throw std::invalid_argument(
            "earliest_tidal_fraction: one start and end position per body is required");
    }
    std::optional<double> earliest;
    for (const DisruptionEvent& e : tidal_entries(bodies, start_positions, end_positions,
                                                  space::Time(0.0), space::Time(0.0))) {
        if (!earliest || e.step_fraction < *earliest) {
            earliest = e.step_fraction;
        }
    }
    return earliest;
}

} // namespace orrery::core
