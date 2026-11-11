#include "orrery/render/trails.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <utility>

#include <glm/geometric.hpp>

#include "orrery/core/units.h"

namespace orrery::render {

TrailRing::TrailRing(std::size_t capacity) : points_(capacity) {}

void TrailRing::push(const glm::dvec3& position) {
    const std::size_t cap = points_.size();
    if (cap == 0) {
        return;
    }
    if (size_ < cap) {
        points_[(start_ + size_) % cap] = position;
        ++size_;
    } else {
        points_[start_] = position;
        start_ = (start_ + 1) % cap;
    }
}

void TrailRing::clear() noexcept {
    start_ = 0;
    size_ = 0;
}

void TrailRing::set_capacity(std::size_t capacity) {
    if (capacity == points_.size()) {
        return;
    }
    const std::size_t keep = std::min(size_, capacity);
    std::vector<glm::dvec3> points(capacity);
    for (std::size_t i = 0; i < keep; ++i) {
        points[i] = at(size_ - keep + i);
    }
    points_ = std::move(points);
    start_ = 0;
    size_ = keep;
}

const glm::dvec3& TrailRing::at(std::size_t i) const {
    if (i >= size_) {
        throw std::out_of_range("TrailRing::at: index beyond size");
    }
    return points_[(start_ + i) % points_.size()];
}

TrailHistory::TrailHistory(TrailSettings settings) : settings_(settings) {}

void TrailHistory::set_settings(const TrailSettings& settings) {
    settings_ = settings;
    for (BodyTrail& trail : trails_) {
        trail.ring.set_capacity(settings_.length);
    }
}

const TrailRing* TrailHistory::find(core::BodyId id) const noexcept {
    const auto it = std::lower_bound(trails_.begin(), trails_.end(), id,
                                     [](const BodyTrail& t, core::BodyId v) { return t.id < v; });
    return it != trails_.end() && it->id == id ? &it->ring : nullptr;
}

void TrailHistory::record(const core::SimSnapshot& snapshot) {
    const double au = core::constants::astronomical_unit.value();
    const auto positions = snapshot.positions();
    const auto ids = snapshot.ids();

    // Reference for the angular spacing: the most massive body.
    glm::dvec3 reference{0.0};
    double heaviest = -1.0;
    for (std::size_t i = 0; i < snapshot.size(); ++i) {
        if (snapshot.masses()[i].value() > heaviest) {
            heaviest = snapshot.masses()[i].value();
            reference =
                glm::dvec3(positions[i].x.value(), positions[i].y.value(), positions[i].z.value()) *
                au;
        }
    }

    // Both sequences are sorted by id: merge them.
    std::vector<BodyTrail> next;
    next.reserve(ids.size());
    auto old = trails_.begin();
    for (std::size_t i = 0; i < ids.size(); ++i) {
        while (old != trails_.end() && old->id < ids[i]) {
            ++old; // body no longer present: its trail is dropped
        }
        BodyTrail trail{ids[i], TrailRing(settings_.length)};
        if (old != trails_.end() && old->id == ids[i]) {
            trail.ring = std::move(old->ring);
            ++old;
        }
        const glm::dvec3 world =
            glm::dvec3(positions[i].x.value(), positions[i].y.value(), positions[i].z.value()) * au;
        bool store = true;
        if (!trail.ring.empty() && settings_.min_step_angle > 0.0) {
            const double step = glm::length(world - trail.ring.newest());
            store = step >= settings_.min_step_angle * glm::length(world - reference);
        }
        if (store && std::isfinite(world.x) && std::isfinite(world.y) && std::isfinite(world.z)) {
            trail.ring.push(world);
        }
        next.push_back(std::move(trail));
    }
    trails_ = std::move(next);
}

void append_trail_segments(const TrailRing& ring, const glm::dvec3& head, const Camera& camera,
                           std::vector<TrailSegment>& out) {
    const std::size_t n = ring.size();
    if (n == 0 || ring.capacity() == 0) {
        return;
    }
    const double capacity = static_cast<double>(ring.capacity());
    // Point j (0 = oldest) has age n - j samples; the head has age 0.
    const auto fade = [&](std::size_t age) {
        return static_cast<float>(std::max(0.0, 1.0 - static_cast<double>(age) / capacity));
    };
    // Repeated world positions (a stationary body, or the newest sample
    // equal to the head) would give zero-length segments, which the shader
    // would still widen into visible, stacked quads: a run of equal points
    // counts once, with the fade of its first point so that the fade stays
    // continuous along the polyline.
    glm::dvec3 previous_world = ring.at(0);
    glm::vec3 previous = camera.to_render(previous_world);
    float previous_fade = fade(n);
    out.reserve(out.size() + n);
    for (std::size_t j = 1; j <= n; ++j) {
        const bool is_head = j == n;
        const glm::dvec3& world = is_head ? head : ring.at(j);
        if (world == previous_world) {
            continue;
        }
        const glm::vec3 current = camera.to_render(world);
        const float current_fade = fade(n - j);
        out.push_back({previous, previous_fade, current, current_fade});
        previous_world = world;
        previous = current;
        previous_fade = current_fade;
    }
}

} // namespace orrery::render
