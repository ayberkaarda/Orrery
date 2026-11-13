#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include <glm/ext/vector_double3.hpp>

#include "orrery/core/result.h"
#include "orrery/render/aero_colormap.h"
#include "orrery/render/camera.h"
#include "orrery/render/entry_geometry.h"
#include "orrery/render/hdr_pipeline.h"

// Entry view (docs/entry-workspace.md, "View"): draws an atmospheric entry
// into the HDR scene target, like AeroView does for an aero result.
//
//   1. the planet: a lit sphere with a coastline-free graticule (10 deg)
//   2. the atmosphere shell: translucent, ray-marched in 16 samples through a
//      layer up to the entry interface (drawn exaggerated with the altitudes)
//   3. the ground track on the surface and a faint curtain from it up to the
//      trajectory, which shows the altitude
//   4. the trajectory ribbon, coloured by a quantity (g-load, heat flux) with
//      the sequential map, outlined so it reads over the planet and the sky
//   5. markers (start, peak, cursor) as screen-space discs
//
// Plain data in, no user-interface code: the legend and labels are drawn by
// the workspace. APPROXIMATION: the atmosphere is a single-colour
// exponential haze, not Rayleigh/Mie scattering; altitudes are exaggerated.
namespace orrery::render {

enum class EntryPlanetStyle : std::uint8_t { earth, mars };

// Recommended pipeline settings: exposure 1, 4x MSAA, the sky colour.
[[nodiscard]] HdrSettings entry_hdr_settings();

struct EntryMarker {
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double altitude_m = 0.0;
    glm::dvec3 color{1.0};  // display-referred linear
    double radius_px = 5.0; // at a 1080-row frame
};

struct EntryViewOptions {
    EntryPlanetStyle style = EntryPlanetStyle::earth;
    // Colour range of the ribbon's values (sequential map from
    // kEntryMapStart).
    ScalarRange range;
    bool ribbon = true;
    bool ground_track = true;
    bool curtain = true;
    bool atmosphere = true;
    bool graticule = true;
    double ribbon_width_px = 5.0; // at a 1080-row frame
    std::vector<EntryMarker> markers;
    // Direction towards the sun in the view frame (normalised on use).
    glm::dvec3 sun{-0.35, -0.55, 0.76};
    double exposure = 1.0;
};

struct EntryDrawStats {
    std::size_t planet_triangles = 0;
    std::size_t ribbon_segments = 0;
    std::size_t ground_segments = 0;
    std::size_t markers = 0;
};

class EntryView {
public:
    // Requires a current OpenGL 4.6 context; compiles the shaders and builds
    // the planet mesh.
    [[nodiscard]] static core::Result<EntryView, std::string> create();

    EntryView(EntryView&& other) noexcept;
    EntryView& operator=(EntryView&& other) noexcept;
    EntryView(const EntryView&) = delete;
    EntryView& operator=(const EntryView&) = delete;
    // Releases the GL objects: the creating context must be current.
    ~EntryView();

    // Replaces the track (an empty track removes it) and the globe it is
    // drawn on (planet, atmosphere and exaggeration; the view frame follows
    // the track). Points with a non-finite coordinate or value, and a globe
    // without a positive radius, atmosphere and exaggeration >= 1, are
    // rejected with the reason; the previous track then stays.
    [[nodiscard]] core::Result<std::monostate, std::string>
    set_track(std::vector<EntryTrackPoint> track, const EntryGlobe& globe);
    [[nodiscard]] const EntryFrame& frame() const noexcept;
    [[nodiscard]] const EntryGlobe& globe() const noexcept;
    [[nodiscard]] std::size_t track_size() const noexcept;

    // Draws into the bound HDR scene framebuffer.
    [[nodiscard]] core::Result<EntryDrawStats, std::string>
    draw(const Camera& camera, const FrameInfo& frame, const EntryViewOptions& options);

private:
    struct State;
    explicit EntryView(std::unique_ptr<State> state) noexcept;
    std::unique_ptr<State> state_;
};

} // namespace orrery::render
