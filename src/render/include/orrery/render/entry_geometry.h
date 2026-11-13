#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include <glm/ext/matrix_double3x3.hpp>
#include <glm/ext/vector_double3.hpp>

#include "orrery/geom/triangle_mesh.h"
#include "orrery/render/aero_colormap.h"
#include "orrery/render/camera.h"

// Geometry of the entry view (docs/entry-workspace.md, "View"): plain double
// math, no OpenGL, so that framing and placement are tested without a GPU.
//
// The view frame is a rotation of the planet-fixed frame (x through
// latitude 0, longitude 0; z north) that puts the middle of the track on the
// +z axis and its direction of flight along +x. The planet centre stays at
// the origin, so world +z is the local vertical at the track, which is what
// the turntable camera orbits around. Altitudes are drawn exaggerated by
// EntryGlobe::altitude_scale (the view says so).
namespace orrery::render {

struct EntryTrackPoint {
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double altitude_m = 0.0;
    double value = 0.0; // the coloured quantity (g-load, heat flux)
};

struct EntryGlobe {
    double radius_m = 6'371'000.0;
    double atmosphere_top_m = 121'920.0;
    double altitude_scale = 5.0; // vertical exaggeration of altitudes (>= 1)
};

struct EntryFrame {
    glm::dmat3 to_view{1.0}; // planet-fixed -> view frame
    // Bounds of the track and its ground track in the view frame [m].
    geom::Bounds bounds;
};

// Unit vector of a latitude and longitude in the planet-fixed frame.
[[nodiscard]] glm::dvec3 planet_direction(double latitude_deg, double longitude_deg) noexcept;

// The view frame of `track` (a view of latitude 0, longitude 0 looking east
// when the track is empty or has no usable point). Non-finite points are
// ignored.
[[nodiscard]] EntryFrame make_entry_frame(std::span<const EntryTrackPoint> track,
                                          const EntryGlobe& globe);

// Position of a point at `altitude_m` (drawn exaggerated) in the view frame.
[[nodiscard]] glm::dvec3 view_position(const EntryFrame& frame, const EntryGlobe& globe,
                                       double latitude_deg, double longitude_deg,
                                       double altitude_m) noexcept;

// Turntable that frames the track from the side and above (yaw measured from
// the view +x axis, the direction of flight), with the planet's limb in the
// upper part of the image.
[[nodiscard]] TurntableMode frame_entry(const EntryFrame& frame, double aspect,
                                        const CameraLens& lens, double yaw = -1.25,
                                        double pitch = 0.42);

// Colour of the ribbon: the sequential map from `kEntryMapStart` to 1, so the
// lowest values stay visible against the dark sky and the night side.
inline constexpr double kEntryMapStart = 0.18;
[[nodiscard]] double entry_map_position(const ScalarRange& range, double value) noexcept;

// The track decimated to at most `max_points` points (both ends and the
// points of largest value kept), for uploads and the 2D map.
[[nodiscard]] std::vector<EntryTrackPoint> decimate_track(std::span<const EntryTrackPoint> track,
                                                          std::size_t max_points);

} // namespace orrery::render
