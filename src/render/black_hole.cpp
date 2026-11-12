#include "orrery/render/black_hole.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>

#include "orrery/core/black_hole.h"
#include "orrery/core/units.h"
#include "orrery/render/body_visuals.h"
#include "orrery/render/space_renderer.h"

namespace orrery::render {

// APPROXIMATION: thin accretion disk around a non-rotating hole - zero
// thickness, in the plane normal to world +Z, from the ISCO (6 M) to 30 M,
// gas on circular geodesics, Newtonian-limit thin-disk temperature profile
// (no Novikov-Thorne correction factors), optically thick black-body
// emission without limb darkening or returning radiation, spin ignored
// (ADR-003) - the disk is a rendering model, no accretion is simulated -
// shape, inner edge, colour gradient and Doppler/redshift pattern of a thin,
// optically thick disk around a Schwarzschild hole; luminosity is scaled for
// display.
core::geodesic::DiskModel lensing_disk() noexcept {
    core::geodesic::DiskModel disk;
    disk.axis = {0.0, 0.0, 1.0};
    disk.inner_radius = core::geodesic::kIscoRadius;
    disk.outer_radius = kDiskOuterRadius;
    return disk;
}

double shadow_angular_radius(double observer_radius) noexcept {
    const double r = observer_radius;
    if (!(r > core::geodesic::kHorizonRadius)) {
        return std::numbers::pi;
    }
    const double s = core::geodesic::kCriticalImpactParameter * std::sqrt(1.0 - 2.0 / r) / r;
    if (r >= core::geodesic::kPhotonSphereRadius) {
        return std::asin(std::min(s, 1.0));
    }
    // Inside the photon sphere the escape cone points outwards: the shadow
    // fills the rest of the sky.
    return std::numbers::pi - std::asin(std::min(s, 1.0));
}

double einstein_angle(double observer_radius) noexcept {
    return observer_radius > 0.0 ? std::sqrt(4.0 / observer_radius) : std::numbers::pi;
}

// APPROXIMATION: one lens, bodies unlensed - only the hole with the largest
// Einstein angle bends the sky, and bodies are drawn unbent in the normal
// passes (hidden by the shadow and the disk through depth) - each body would
// need its own lensed images; two comparable holes in one view are rare in
// the bundled scenarios - exact for a single hole and the sky at infinity;
// bodies within a few Einstein radii of the hole appear at their unlensed
// positions.
std::optional<LensingTarget> select_lensed_black_hole(const core::SimSnapshot& snapshot,
                                                      const Camera& camera, const FrameInfo& frame,
                                                      const RenderSettings& settings) {
    if (!settings.black_hole_lensing || frame.height <= 0) {
        return std::nullopt;
    }
    const double pps = pixel_scale(camera.lens().vertical_fov, frame.height);
    std::optional<LensingTarget> best;
    for (std::size_t i = 0; i < snapshot.size(); ++i) {
        if (snapshot.kind_tag(i) != core::BodyKindTag::black_hole) {
            continue;
        }
        const core::space::Mass mass = snapshot.masses()[i];
        const double rs = core::to_si(core::schwarzschild_radius(mass)).value();
        if (!(rs > 0.0) || !std::isfinite(rs)) {
            continue;
        }
        LensingTarget target;
        target.index = i;
        target.id = snapshot.ids()[i];
        target.mass_length = 0.5 * rs;
        target.hole_render = camera.to_render_double(world_position(snapshot, i));
        const double distance = glm::length(target.hole_render);
        if (!std::isfinite(distance)) {
            continue;
        }
        target.observer_radius = distance / target.mass_length;
        target.einstein_radius_px =
            std::tan(std::min(einstein_angle(target.observer_radius), 1.5)) * pps;
        target.shadow_radius_px =
            std::tan(std::min(shadow_angular_radius(target.observer_radius), 1.5)) * pps;
        if (target.einstein_radius_px < kLensingMinEinsteinPx) {
            continue;
        }
        if (!best || target.einstein_radius_px > best->einstein_radius_px) {
            best = target;
        }
    }
    return best;
}

TraceGrid trace_grid(const FrameInfo& frame) noexcept {
    return {std::max(frame.width, 1) / 2 + 1, std::max(frame.height, 1) / 2 + 1};
}

glm::dvec2 trace_sample_pixel(int i, int j) noexcept {
    return {2.0 * i + 1.0, 2.0 * j + 1.0};
}

glm::dvec3 pixel_ray(const Camera& camera, const FrameInfo& frame, const glm::dvec2& window_pixel) {
    const double aspect = Camera::usable_aspect(frame.aspect());
    const double tan_y = std::tan(0.5 * camera.lens().vertical_fov);
    const double ndc_x = window_pixel.x / frame.width * 2.0 - 1.0;
    const double ndc_y = window_pixel.y / frame.height * 2.0 - 1.0;
    const glm::dvec3 local{ndc_x * tan_y * aspect, ndc_y * tan_y, -1.0};
    return glm::normalize(camera.orientation() * local);
}

} // namespace orrery::render
