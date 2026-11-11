#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include <glm/ext/vector_double3.hpp>
#include <glm/ext/vector_float3.hpp>

#include "orrery/core/body.h"
#include "orrery/core/sim_snapshot.h"
#include "orrery/render/camera.h"

// Orbit trails (docs/rendering.md, "Trails"): a fixed-length ring buffer of
// past world positions per body, kept in double precision (metres), and the
// conversion of a trail into camera-relative float segments each frame.
namespace orrery::render {

// Fixed-capacity ring of world positions [m], oldest to newest.
class TrailRing {
public:
    explicit TrailRing(std::size_t capacity = 0);

    // Appends a position; when full, the oldest one is dropped.
    void push(const glm::dvec3& position);
    void clear() noexcept;
    // Changes the capacity and keeps the newest min(size, capacity) points.
    void set_capacity(std::size_t capacity);

    [[nodiscard]] std::size_t size() const noexcept { return size_; }
    [[nodiscard]] std::size_t capacity() const noexcept { return points_.size(); }
    [[nodiscard]] bool empty() const noexcept { return size_ == 0; }
    // i = 0 is the oldest point; i must be below size().
    [[nodiscard]] const glm::dvec3& at(std::size_t i) const;
    [[nodiscard]] const glm::dvec3& newest() const { return at(size_ - 1); }

private:
    std::vector<glm::dvec3> points_;
    std::size_t start_ = 0; // index of the oldest point
    std::size_t size_ = 0;
};

struct TrailSettings {
    // Points kept per body; 0 disables recording.
    std::size_t length = 512;
    // A sample is stored only once the body has moved by at least
    // min_step_angle * (its distance to the most massive body) since the
    // last stored sample [rad]. Every trail then spans about the same
    // fraction of its orbit (length * min_step_angle radians), whatever the
    // period. 0 stores every recorded snapshot.
    double min_step_angle = 0.0;
};

struct BodyTrail {
    core::BodyId id;
    TrailRing ring;
};

// Trails of every body of a sequence of snapshots. Bodies appear when first
// seen and their trails are dropped when a snapshot no longer contains them
// (merged, absorbed or removed). Trails are kept sorted by id.
class TrailHistory {
public:
    explicit TrailHistory(TrailSettings settings = {});

    void record(const core::SimSnapshot& snapshot);
    void clear() noexcept { trails_.clear(); }

    // Applies a new length (keeping the newest points) and spacing.
    void set_settings(const TrailSettings& settings);
    [[nodiscard]] const TrailSettings& settings() const noexcept { return settings_; }

    [[nodiscard]] std::span<const BodyTrail> trails() const noexcept { return trails_; }
    [[nodiscard]] const TrailRing* find(core::BodyId id) const noexcept;

private:
    TrailSettings settings_;
    std::vector<BodyTrail> trails_;
};

// One trail segment in camera-relative render space, ready for the GPU.
// fade runs from 1 at the body to 0 at the end of a full-length trail.
struct TrailSegment {
    glm::vec3 p0{0.0F};
    float fade0 = 0.0F;
    glm::vec3 p1{0.0F};
    float fade1 = 0.0F;
};

// Converts a trail into segments from the oldest point to `head` (the body's
// current world position [m]). Positions are made camera-relative in double
// (Camera::to_render_double) and only then cast to float, so the segments are
// float-safe at any distance from the world origin. The fade of a point is
// 1 - age / capacity, with age counted in samples from the head.
void append_trail_segments(const TrailRing& ring, const glm::dvec3& head, const Camera& camera,
                           std::vector<TrailSegment>& out);

} // namespace orrery::render
