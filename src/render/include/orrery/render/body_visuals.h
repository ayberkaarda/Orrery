#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

#include <glm/ext/vector_double3.hpp>

#include "orrery/core/body.h"
#include "orrery/core/quantity.h"

// CPU-side visual policy for bodies (docs/rendering.md, "Bodies"): how big a
// body is drawn, with which mesh level, in which colours, lit by what. Pure
// functions without OpenGL so that they are unit-tested on every platform.
namespace orrery::render {

// Body radius policy.
//   true_scale: bodies are drawn at their physical radius. At solar-system
//               scale almost every body is then far below one pixel.
//   clamped:    the physical radius, enlarged where necessary so that the
//               body covers at least a minimum screen radius (per kind, see
//               min_screen_radius_px). Close up the physical radius wins, so
//               the clamp never shrinks or distorts a resolved body.
enum class ScaleMode : std::uint8_t { true_scale, clamped };

// Procedural surface model selected per body in the body shader.
enum class SurfaceStyle : std::uint8_t {
    uniform = 0,
    rocky = 1,       // mottled regolith (Mercury, Moon, asteroids)
    cloudy = 2,      // featureless cloud deck with faint banding (Venus)
    terrestrial = 3, // oceans, land, ice caps and clouds (Earth)
    desert = 4,      // ochre surface with dark albedo features and caps (Mars)
    banded = 5,      // gas giant belts and zones (Jupiter, Saturn)
    ice_giant = 6,   // nearly uniform with soft bands (Uranus, Neptune)
    star = 7,        // emissive photosphere with limb darkening
    black_hole = 8,  // dark disc (shadow) placeholder
    probe = 9,       // small emissive marker
};

struct BodyAppearance {
    double nominal_radius = 0.0; // [m], used when the body has no physical radius
    glm::dvec3 albedo{0.5};      // linear diffuse reflectance (primary colour)
    glm::dvec3 secondary{0.5};   // linear secondary colour (bands, land, features)
    SurfaceStyle style = SurfaceStyle::uniform;
    double specular = 0.0;               // specular weight in [0, 1]
    glm::dvec3 emission{0.0};            // scene-linear emitted radiance (stars, probes)
    double temperature = 0.0;            // effective temperature [K], stars only
    double luminosity = 0.0;             // [L_sun], stars only
    glm::dvec3 spin_axis{0.0, 0.0, 1.0}; // world (ecliptic J2000) unit vector
};

// Visual appearance of a body. Known solar-system bodies (the first word of
// the name, case-insensitive: "Earth", "Jupiter system", ...) get their mean
// radius, colours and obliquity from a built-in table; other bodies get a
// per-kind default.
[[nodiscard]] BodyAppearance appearance_for(std::string_view name, const core::BodyKind& kind,
                                            core::space::Mass mass);

// Radius to draw [m]: the physical radius when it is positive, otherwise the
// appearance's nominal radius.
[[nodiscard]] double physical_radius(core::space::Length radius,
                                     const BodyAppearance& appearance) noexcept;

// Minimum screen radius in pixels per kind for ScaleMode::clamped, times
// `scale` (a user setting, clamped to [0.25, 8]).
[[nodiscard]] double min_screen_radius_px(core::BodyKindTag kind, double scale) noexcept;

// Pixels per unit of tan(angle) at the image centre: (height / 2) / tan(fov_y / 2).
[[nodiscard]] double pixel_scale(double vertical_fov, int viewport_height) noexcept;

// Angular size in pixels of a sphere of radius r at centre distance d:
// tan(asin(r / d)) * pixel_scale, its exact projected radius on the optical
// axis. Infinite when the eye is inside the sphere. This is the measure of
// the minimum-size policy (visual_radius) and the marker weight; screen
// bounds off the axis come from project_sphere.
[[nodiscard]] double projected_radius_px(double radius, double distance,
                                         double pixels_per_tangent) noexcept;

// Screen bounds of a sphere under the perspective projection, from its
// camera-space centre: `distance` from the eye and `depth` along the view
// axis (positive in front).
//   radius_px:    largest screen extent, the semi-major axis of the projected
//                 ellipse, r sqrt(d^2 - r^2) / (z^2 - r^2) * pixel_scale; it
//                 equals projected_radius_px on the axis and grows towards
//                 the image edges (for labels and glare sprites).
//   lod_scale_px: bound on the image displacement per unit of relative
//                 radial mesh error, r (d + r) / (z - r)^2 * pixel_scale: the
//                 largest magnification of the projection over the sphere.
//                 At least radius_px, and equal to it for small spheres.
// Both are infinite when the sphere reaches the plane of the eye (z <= r).
struct ProjectedSphere {
    double radius_px = 0.0;
    double lod_scale_px = 0.0;
};
[[nodiscard]] ProjectedSphere project_sphere(double radius, double distance, double depth,
                                             double pixels_per_tangent) noexcept;

// Radius to draw so that the sphere covers at least `min_px` pixels of
// angular size (ScaleMode::clamped) or exactly the physical radius
// (true_scale). The policy is angular on purpose: a marker keeps its size
// wherever it is on screen, and off the axis its drawn ellipse is larger.
[[nodiscard]] double visual_radius(double radius, double distance, double pixels_per_tangent,
                                   double min_px, ScaleMode mode) noexcept;

// Mesh levels used for bodies: LOD i is the icosphere of subdivision i.
// Every point of a level lies between the unit sphere and the sphere of the
// level's closest face plane, so its outline deviates from the true outline
// by at most face_depth * lod_scale_px, where face_depth (1 - distance of the
// closest face plane) is measured from the generated mesh together with the
// longest edge (whose sagitta 1 - cos(theta / 2) is smaller). select_lod picks
// the coarsest level whose bound stays within a quarter pixel:
// lod_max_radius_px(i) = 0.25 / face_depth_i is the largest lod_scale_px it
// accepts. The result is shifted by `bias` levels and clamped to
// [0, kLodCount - 1].
inline constexpr std::size_t kLodCount = 7;
[[nodiscard]] double lod_max_radius_px(std::size_t lod) noexcept;
[[nodiscard]] std::size_t select_lod(double projected_px, int bias) noexcept;

// Conservative frustum test of a sphere in render space (camera at the
// origin), given the camera -> world rotation as the three camera axes.
struct ViewFrustum {
    glm::dvec3 right{1.0, 0.0, 0.0};
    glm::dvec3 up{0.0, 1.0, 0.0};
    glm::dvec3 back{0.0, 0.0, 1.0}; // the camera looks along -back
    double tan_half_fov_y = 0.4;
    double aspect = 1.0;
};
[[nodiscard]] bool sphere_in_frustum(const ViewFrustum& frustum, const glm::dvec3& centre,
                                     double radius) noexcept;

// Irradiance at distance d from a star of luminosity L [L_sun], relative to
// the irradiance at 1 AU from the Sun: L (1 AU / d)^exponent. exponent = 2 is
// the physical inverse-square law; see docs/rendering.md for the display
// default.
[[nodiscard]] double relative_irradiance(double luminosity, double distance_m,
                                         double exponent) noexcept;

// Linear sRGB colour of a black body of the given temperature [K], normalised
// to unit luminance (Y = 1). Valid from 1667 K to 25000 K; outside, the
// temperature is clamped.
[[nodiscard]] glm::dvec3 blackbody_linear_srgb(double kelvin) noexcept;

} // namespace orrery::render
