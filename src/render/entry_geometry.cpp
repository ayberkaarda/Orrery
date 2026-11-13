#include "orrery/render/entry_geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include <glm/geometric.hpp>
#include <glm/matrix.hpp>

#include "orrery/render/aero_view.h"

namespace orrery::render {

namespace {

constexpr double kDegree = std::numbers::pi / 180.0;

bool usable(const EntryTrackPoint& p) noexcept {
    return std::isfinite(p.latitude_deg) && std::isfinite(p.longitude_deg) &&
           std::isfinite(p.altitude_m);
}

// A unit vector perpendicular to `axis` (east of it, or x at the poles).
glm::dvec3 any_tangent(const glm::dvec3& axis) noexcept {
    glm::dvec3 east = glm::cross(glm::dvec3(0.0, 0.0, 1.0), axis);
    if (glm::length(east) < 1.0e-9) {
        east = glm::cross(glm::dvec3(0.0, 1.0, 0.0), axis);
    }
    return glm::normalize(east);
}

} // namespace

glm::dvec3 planet_direction(double latitude_deg, double longitude_deg) noexcept {
    const double lat = latitude_deg * kDegree;
    const double lon = longitude_deg * kDegree;
    return {std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
}

EntryFrame make_entry_frame(std::span<const EntryTrackPoint> track, const EntryGlobe& globe) {
    glm::dvec3 sum(0.0);
    const EntryTrackPoint* first = nullptr;
    const EntryTrackPoint* last = nullptr;
    for (const EntryTrackPoint& p : track) {
        if (!usable(p)) {
            continue;
        }
        sum += planet_direction(p.latitude_deg, p.longitude_deg);
        first = first == nullptr ? &p : first;
        last = &p;
    }
    glm::dvec3 up(1.0, 0.0, 0.0);
    if (glm::length(sum) > 1.0e-9) {
        up = glm::normalize(sum);
    }
    glm::dvec3 forward = any_tangent(up);
    if (first != nullptr && last != nullptr && first != last) {
        const glm::dvec3 d = planet_direction(last->latitude_deg, last->longitude_deg) -
                             planet_direction(first->latitude_deg, first->longitude_deg);
        const glm::dvec3 tangent = d - glm::dot(d, up) * up;
        if (glm::length(tangent) > 1.0e-9) {
            forward = glm::normalize(tangent);
        }
    }
    const glm::dvec3 side = glm::cross(up, forward);
    EntryFrame frame;
    frame.to_view = glm::transpose(glm::dmat3(forward, side, up));

    glm::dvec3 lo(std::numeric_limits<double>::infinity());
    glm::dvec3 hi(-std::numeric_limits<double>::infinity());
    bool any = false;
    for (const EntryTrackPoint& p : track) {
        if (!usable(p)) {
            continue;
        }
        for (const double altitude : {p.altitude_m, 0.0}) {
            const glm::dvec3 v =
                view_position(frame, globe, p.latitude_deg, p.longitude_deg, altitude);
            lo = glm::min(lo, v);
            hi = glm::max(hi, v);
        }
        any = true;
    }
    if (!any) {
        // A patch of ground around the view point.
        const double r = globe.radius_m;
        lo = glm::dvec3(-0.1 * r, -0.1 * r, r * 0.98);
        hi = glm::dvec3(0.1 * r, 0.1 * r, r + globe.atmosphere_top_m * globe.altitude_scale);
    }
    frame.bounds = {{lo.x, lo.y, lo.z}, {hi.x, hi.y, hi.z}};
    return frame;
}

glm::dvec3 view_position(const EntryFrame& frame, const EntryGlobe& globe, double latitude_deg,
                         double longitude_deg, double altitude_m) noexcept {
    const double radius = globe.radius_m + std::max(altitude_m, 0.0) * globe.altitude_scale;
    return frame.to_view * (planet_direction(latitude_deg, longitude_deg) * radius);
}

TurntableMode frame_entry(const EntryFrame& frame, double aspect, const CameraLens& lens,
                          double yaw, double pitch) {
    return frame_bounds(frame.bounds, aspect, lens, yaw, pitch);
}

double entry_map_position(const ScalarRange& range, double value) noexcept {
    return kEntryMapStart + (1.0 - kEntryMapStart) * range.normalize(value);
}

std::vector<EntryTrackPoint> decimate_track(std::span<const EntryTrackPoint> track,
                                            std::size_t max_points) {
    if (track.size() <= max_points || max_points < 3) {
        return {track.begin(), track.end()};
    }
    std::size_t peak = 0;
    for (std::size_t i = 1; i < track.size(); ++i) {
        if (track[i].value > track[peak].value) {
            peak = i;
        }
    }
    // ceil((n - 1) / (max - 2)) keeps at most max - 2 regular points, plus
    // the peak and the last point.
    const std::size_t step = (track.size() + max_points - 4) / (max_points - 2);
    std::vector<EntryTrackPoint> out;
    out.reserve(max_points);
    for (std::size_t i = 0; i + 1 < track.size(); i += step) {
        out.push_back(track[i]);
        if (peak > i && peak < i + step && peak + 1 < track.size()) {
            out.push_back(track[peak]);
        }
    }
    out.push_back(track.back());
    return out;
}

} // namespace orrery::render
