#include "orrery/app/space_view.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <glm/geometric.hpp>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#elif defined(__APPLE__)
#include <mach-o/dyld.h>
#endif

#include "orrery/core/black_hole.h"
#include "orrery/core/body.h"
#include "orrery/core/schwarzschild_geodesic.h"
#include "orrery/core/units.h"
#include "orrery/render/black_hole.h"
#include "orrery/render/body_visuals.h"
#include "orrery/render/space_renderer.h"

namespace orrery::app {

namespace {

constexpr double kPi = 3.141592653589793;
constexpr double kDegree = kPi / 180.0;
constexpr double kFramingYaw = 72.0 * kDegree;
constexpr double kFramingPitch = 30.0 * kDegree;
// Fraction of the field of view left free around the framed radius.
constexpr double kFramingMargin = 1.1;
// A lone black hole is framed from just above its disk plane, with the disk
// filling the view (docs/app.md, "Space view").
constexpr double kBlackHolePitch = 12.0 * kDegree;
constexpr double kBlackHoleMargin = 1.08;

std::size_t heaviest_index(const core::SimSnapshot& snapshot) {
    std::size_t heaviest = 0;
    for (std::size_t i = 1; i < snapshot.size(); ++i) {
        if (snapshot.masses()[i].value() > snapshot.masses()[heaviest].value()) {
            heaviest = i;
        }
    }
    return heaviest;
}

// Impact parameter [M] of the highest image of the disk's far rim seen from
// far away: the light from the rim point behind the hole (R cos i behind it,
// R sin i above the line of sight) is bent over the hole, so the ray with
// impact parameter b reaches it when b - R cos i alpha(b) = R sin i, with
// the weak-field deflection alpha(b) = 4/b + 15 pi/(4 b^2) + 128/(3 b^3)
// (physics.md, "Null geodesics"). From a finite distance the image sits
// lower, so this bounds it.
double lensed_rim_height(double radius, double pitch) {
    const double behind = radius * std::cos(pitch);
    const double above = radius * std::sin(pitch);
    const auto excess = [&](double b) {
        const double alpha = 4.0 / b + 15.0 * kPi / (4.0 * b * b) + 128.0 / (3.0 * b * b * b);
        return b - behind * alpha - above;
    };
    // excess rises with b; it is negative at max(R sin i, b_c) and positive
    // at R + 4 (for the 30 M disk seen from 12 degrees, b = 15.5 M).
    double lo = std::max(above, core::geodesic::kCriticalImpactParameter);
    double hi = radius + 4.0;
    for (int i = 0; i < 60; ++i) {
        const double mid = 0.5 * (lo + hi);
        (excess(mid) < 0.0 ? lo : hi) = mid;
    }
    return hi;
}

// Camera distance [M] that frames the disk (radius R, seen from `pitch`
// above its plane) with kBlackHoleMargin in both directions:
// - width: a rim point at azimuth phi from the camera's side lies at
//   R sin phi / (D - R cos i cos phi) to the side; the widest, at
//   cos phi = R cos i / D, is R / sqrt(D^2 - R^2 cos^2 i);
// - below the hole: the near rim, R sin i / (D - R cos i);
// - above it: the lensed far rim, about lensed_rim_height / D.
// The camera always stays outside the disk (D >= margin R), and so outside
// the horizon.
double black_hole_distance(double tan_half_v, double usable_aspect) {
    const double r = render::kDiskOuterRadius;
    const double in_plane = r * std::cos(kBlackHolePitch);
    const double tan_half_h = tan_half_v * usable_aspect;
    const double wide = kBlackHoleMargin * r / tan_half_h;
    const double width = std::sqrt(in_plane * in_plane + wide * wide);
    const double near_rim =
        in_plane + kBlackHoleMargin * r * std::sin(kBlackHolePitch) / tan_half_v;
    const double far_rim = kBlackHoleMargin * lensed_rim_height(r, kBlackHolePitch) / tan_half_v;
    return std::max({width, near_rim, far_rim, kBlackHoleMargin * r});
}

} // namespace

render::OrbitMode framing_orbit(const core::SimSnapshot& snapshot, double aspect) {
    render::OrbitMode orbit;
    orbit.yaw = kFramingYaw;
    orbit.pitch = kFramingPitch;
    const double au = core::constants::astronomical_unit.value();
    if (snapshot.empty()) {
        orbit.focus = glm::dvec3(0.0);
        orbit.distance = 3.0 * au;
        return orbit;
    }
    const std::size_t heaviest = heaviest_index(snapshot);
    double total_mass = 0.0;
    glm::dvec3 weighted(0.0);
    for (std::size_t i = 0; i < snapshot.size(); ++i) {
        const double m = snapshot.masses()[i].value();
        total_mass += m;
        weighted += render::world_position(snapshot, i) * m;
    }
    const bool dominant = snapshot.masses()[heaviest].value() >= 0.5 * total_mass;
    glm::dvec3 centre = total_mass > 0.0 ? weighted / total_mass : glm::dvec3(0.0);
    if (dominant) {
        orbit.target = snapshot.ids()[heaviest];
        centre = render::world_position(snapshot, heaviest);
    }
    orbit.focus = centre;

    std::vector<double> distances;
    distances.reserve(snapshot.size());
    for (std::size_t i = 0; i < snapshot.size(); ++i) {
        if (dominant && i == heaviest) {
            continue;
        }
        const double d = glm::length(render::world_position(snapshot, i) - centre);
        if (std::isfinite(d) && d > 0.0) {
            distances.push_back(d);
        }
    }
    double radius = au;
    const double tan_half_v = std::tan(0.5 * kSpaceFovDegrees * kDegree);
    const double usable = render::Camera::usable_aspect(aspect);
    if (distances.empty() && snapshot.kind_tag(heaviest) == core::BodyKindTag::black_hole) {
        // The accretion disk drawn by the lensing pass reaches
        // kDiskOuterRadius G M / c^2; seen from 12 degrees above its plane it
        // is wide and flat, so its width sets the distance in most windows
        // and its height in very wide ones.
        const double rs =
            core::to_si(core::schwarzschild_radius(snapshot.masses()[heaviest])).value();
        orbit.pitch = kBlackHolePitch;
        orbit.distance = black_hole_distance(tan_half_v, usable) * 0.5 * rs;
        return orbit;
    }
    if (!distances.empty()) {
        std::sort(distances.begin(), distances.end());
        radius = distances[distances.size() / 2];
    } else {
        // A single body: frame it at a few radii.
        const render::BodyAppearance appearance = render::appearance_for(
            snapshot.names()[heaviest], snapshot.kinds()[heaviest], snapshot.masses()[heaviest]);
        radius = 4.0 * render::physical_radius(snapshot.radii()[heaviest], appearance);
    }
    const double tan_half = std::min(tan_half_v, tan_half_v * usable);
    orbit.distance = kFramingMargin * radius / tan_half;
    return orbit;
}

render::Camera framing_camera(const core::SimSnapshot& snapshot, double aspect) {
    render::Camera camera;
    render::CameraLens lens;
    lens.vertical_fov = kSpaceFovDegrees * kDegree;
    (void)camera.set_lens(lens);
    render::OrbitMode orbit = framing_orbit(snapshot, aspect);
    camera.set_mode(orbit);
    std::shared_ptr<const core::SimSnapshot> borrowed(std::shared_ptr<const core::SimSnapshot>{},
                                                      &snapshot);
    camera.update(0.0, render::CameraInput{}, body_frames(std::move(borrowed)));
    return camera;
}

render::BodyFrameLookup body_frames(std::shared_ptr<const core::SimSnapshot> snapshot) {
    return [snapshot = std::move(snapshot)](core::BodyId id) -> std::optional<render::BodyFrame> {
        if (!snapshot) {
            return std::nullopt;
        }
        const std::optional<std::size_t> index = snapshot->index_of(id);
        if (!index) {
            return std::nullopt;
        }
        const std::size_t i = *index;
        const double au = core::constants::astronomical_unit.value();
        const double au_per_day = au / core::constants::day.value();
        render::BodyFrame frame;
        frame.position = render::world_position(*snapshot, i);
        const auto& v = snapshot->velocities()[i];
        frame.velocity = glm::dvec3(v.x.value(), v.y.value(), v.z.value()) * au_per_day;
        const render::BodyAppearance appearance = render::appearance_for(
            snapshot->names()[i], snapshot->kinds()[i], snapshot->masses()[i]);
        frame.radius = render::physical_radius(snapshot->radii()[i], appearance);
        return frame;
    };
}

render::TrailSettings trail_settings(const render::RenderSettings& settings) {
    render::TrailSettings trails;
    trails.length = std::max<std::size_t>(settings.trail_length, 2);
    trails.min_step_angle = 2.0 * kPi * 0.85 / static_cast<double>(trails.length);
    return trails;
}

std::optional<std::filesystem::path> running_executable() {
#if defined(_WIN32)
    std::vector<wchar_t> buffer(MAX_PATH);
    while (buffer.size() <= 32768) {
        const DWORD length =
            GetModuleFileNameW(nullptr, buffer.data(), static_cast<DWORD>(buffer.size()));
        if (length == 0) {
            return std::nullopt;
        }
        if (length < buffer.size()) {
            return std::filesystem::path(std::wstring(buffer.data(), length));
        }
        buffer.resize(buffer.size() * 2);
    }
    return std::nullopt;
#elif defined(__APPLE__)
    std::uint32_t size = 0;
    (void)_NSGetExecutablePath(nullptr, &size);
    std::vector<char> buffer(static_cast<std::size_t>(size) + 1, '\0');
    if (_NSGetExecutablePath(buffer.data(), &size) != 0) {
        return std::nullopt;
    }
    std::error_code error;
    const std::filesystem::path path = std::filesystem::canonical(buffer.data(), error);
    return error ? std::nullopt : std::optional<std::filesystem::path>(path);
#elif defined(__linux__)
    std::error_code error;
    const std::filesystem::path path = std::filesystem::read_symlink("/proc/self/exe", error);
    return error ? std::nullopt : std::optional<std::filesystem::path>(path);
#else
    return std::nullopt;
#endif
}

std::filesystem::path executable_path(const std::filesystem::path& argv0) {
    if (std::optional<std::filesystem::path> path = running_executable()) {
        return *path;
    }
    std::error_code error;
    const std::filesystem::path absolute = std::filesystem::absolute(argv0, error);
    return error ? argv0 : absolute;
}

std::vector<std::filesystem::path> asset_directories(const std::filesystem::path& executable) {
    std::vector<std::filesystem::path> dirs;
    std::error_code error;
    if (!executable.empty()) {
        const std::filesystem::path bin =
            std::filesystem::absolute(executable, error).parent_path();
        if (!error) {
            // A portable copy keeps assets/ beside the executable; an installed
            // tree (bin/ and share/orrery/assets) keeps them under share/.
            for (const std::filesystem::path& candidate :
                 {bin / "assets", bin.parent_path() / "share" / "orrery" / "assets"}) {
                if (std::filesystem::is_directory(candidate, error)) {
                    dirs.push_back(candidate);
                }
            }
        }
    }
#if defined(ORRERY_SOURCE_ASSET_DIR)
    const std::filesystem::path source(ORRERY_SOURCE_ASSET_DIR);
    if (std::filesystem::is_directory(source, error) &&
        std::find(dirs.begin(), dirs.end(), source) == dirs.end()) {
        dirs.push_back(source);
    }
#endif
    return dirs;
}

} // namespace orrery::app
