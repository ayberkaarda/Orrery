#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <vector>

#include "orrery/core/sim_snapshot.h"
#include "orrery/render/camera.h"
#include "orrery/render/render_settings.h"
#include "orrery/render/trails.h"

// View policy shared by the window and the headless renderer (docs/app.md,
// "Space view"), so that both show a simulation the same way.
namespace orrery::app {

// Wall-clock length of one headless frame: 60 frames per second.
inline constexpr std::int64_t kFrameWallNs = 16'666'667;

// Vertical field of view of the space camera [deg].
inline constexpr double kSpaceFovDegrees = 40.0;

// Orbit camera that frames a system: around its dominant body (one holding
// at least half of the mass) or, without one, the fixed centre of mass;
// azimuth 72 deg, elevation 30 deg. The framed radius is the median distance
// of the other bodies from that centre, so a planetary system shows its inner
// and middle orbits instead of shrinking them to a dot around the star to fit
// the outermost one (the mouse wheel zooms out to the rest). A black hole
// alone is framed for its accretion disk instead: 12 deg above the disk
// plane, the disk (render::kDiskOuterRadius) spanning the view's width.
//
// APPROXIMATION: framing heuristic, not a physical quantity - it is valid
// for any distribution of bodies, only the composition varies.
[[nodiscard]] render::OrbitMode framing_orbit(const core::SimSnapshot& snapshot, double aspect);
[[nodiscard]] render::Camera framing_camera(const core::SimSnapshot& snapshot, double aspect);

// Body frames for the camera (orbit target) from a snapshot, which the
// returned function keeps alive.
[[nodiscard]] render::BodyFrameLookup
body_frames(std::shared_ptr<const core::SimSnapshot> snapshot);

// Trail recording settings: `settings.trail_length` samples per body, each
// trail spanning about 85 percent of its orbit.
[[nodiscard]] render::TrailSettings trail_settings(const render::RenderSettings& settings);

// Path of the running executable as the operating system reports it
// (GetModuleFileNameW on Windows, /proc/self/exe on Linux,
// _NSGetExecutablePath on macOS); nullopt where none of these works.
[[nodiscard]] std::optional<std::filesystem::path> running_executable();

// running_executable(), or argv0 made absolute when the system cannot say:
// argv[0] alone is unreliable (a bare name started through PATH resolves
// against the working directory).
[[nodiscard]] std::filesystem::path executable_path(const std::filesystem::path& argv0);

// Directories holding the bundled assets (experiments/, scenarios/), most
// specific first: `assets` next to the executable (a portable copy), then
// `../share/orrery/assets` from the executable's directory (an installed
// tree, docs/packaging.md), each when it exists, then the source tree's
// assets directory when the build recorded it (ORRERY_SOURCE_ASSET_FALLBACK).
[[nodiscard]] std::vector<std::filesystem::path>
asset_directories(const std::filesystem::path& executable);

} // namespace orrery::app
