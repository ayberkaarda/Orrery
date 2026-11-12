#include "orrery/render/body_visuals.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <limits>
#include <string>
#include <variant>

#include <glm/geometric.hpp>

#include "orrery/core/black_hole.h"
#include "orrery/core/units.h"

namespace orrery::render {

namespace {

constexpr double kPi = 3.141592653589793;
constexpr double kDegree = kPi / 180.0;
// Mean obliquity of the ecliptic at J2000 (IAU 2006), for pole directions
// given in equatorial coordinates.
constexpr double kObliquityJ2000 = 23.4392911 * kDegree;
constexpr double kSunRadius = 6.957e8;     // nominal solar radius [m] (IAU 2015 B3)
constexpr double kSunTemperature = 5772.0; // nominal effective temperature [K] (IAU 2015 B3)
constexpr double kEarthMassInSolar = 3.003489614915041e-6;

// IAU pole (right ascension, declination of the north pole, ICRF J2000) in
// equatorial coordinates, rotated into the ecliptic frame of the physics.
glm::dvec3 pole_from_equatorial(double ra_degrees, double dec_degrees) noexcept {
    const double ra = ra_degrees * kDegree;
    const double dec = dec_degrees * kDegree;
    const glm::dvec3 equatorial{std::cos(dec) * std::cos(ra), std::cos(dec) * std::sin(ra),
                                std::sin(dec)};
    const double c = std::cos(kObliquityJ2000);
    const double s = std::sin(kObliquityJ2000);
    return glm::normalize(glm::dvec3{equatorial.x, c * equatorial.y + s * equatorial.z,
                                     -s * equatorial.y + c * equatorial.z});
}

struct KnownBody {
    std::string_view key;
    double radius_km = 0.0;
    glm::dvec3 albedo;
    glm::dvec3 secondary;
    SurfaceStyle style;
    double specular = 0.0;
    double pole_ra = 0.0;
    double pole_dec = 0.0;
};

// APPROXIMATION: body colours - hand-chosen linear albedo pairs for the
// procedural surface models, not measured spectra - display only; radii are
// the IAU/NASA mean radii and pole directions the IAU WGCCRE 2015 J2000
// values (precession ignored), valid for display at any epoch.
const std::array<KnownBody, 11> kKnownBodies{{
    {"mercury",
     2439.7,
     {0.30, 0.28, 0.26},
     {0.17, 0.16, 0.15},
     SurfaceStyle::rocky,
     0.05,
     281.0103,
     61.4155},
    {"venus",
     6051.8,
     {0.82, 0.71, 0.50},
     {0.68, 0.55, 0.36},
     SurfaceStyle::cloudy,
     0.05,
     272.76,
     67.16},
    {"earth",
     6371.0,
     {0.020, 0.055, 0.150},
     {0.120, 0.135, 0.065},
     SurfaceStyle::terrestrial,
     0.35,
     0.0,
     90.0},
    {"moon",
     1737.4,
     {0.22, 0.21, 0.20},
     {0.10, 0.10, 0.10},
     SurfaceStyle::rocky,
     0.03,
     269.9949,
     66.5392},
    {"mars",
     3389.5,
     {0.46, 0.21, 0.09},
     {0.22, 0.11, 0.06},
     SurfaceStyle::desert,
     0.04,
     317.681,
     52.887},
    {"jupiter",
     69911.0,
     {0.80, 0.70, 0.55},
     {0.52, 0.33, 0.20},
     SurfaceStyle::banded,
     0.04,
     268.056595,
     64.495303},
    {"saturn",
     58232.0,
     {0.80, 0.70, 0.50},
     {0.60, 0.48, 0.30},
     SurfaceStyle::banded,
     0.04,
     40.589,
     83.537},
    {"uranus",
     25362.0,
     {0.55, 0.76, 0.80},
     {0.46, 0.66, 0.73},
     SurfaceStyle::ice_giant,
     0.06,
     257.311,
     -15.175},
    {"neptune",
     24622.0,
     {0.24, 0.40, 0.80},
     {0.15, 0.27, 0.62},
     SurfaceStyle::ice_giant,
     0.06,
     299.36,
     43.46},
    {"pluto",
     1188.3,
     {0.60, 0.50, 0.40},
     {0.35, 0.25, 0.18},
     SurfaceStyle::rocky,
     0.03,
     132.993,
     -6.163},
    {"ceres",
     469.7,
     {0.12, 0.12, 0.11},
     {0.08, 0.08, 0.08},
     SurfaceStyle::rocky,
     0.02,
     291.418,
     66.764},
}};

std::string first_word_lower(std::string_view name) {
    std::string word;
    for (const char c : name) {
        if (std::isspace(static_cast<unsigned char>(c)) != 0) {
            if (!word.empty()) {
                break;
            }
            continue;
        }
        word.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
    return word;
}

double mass_kg(core::space::Mass mass) noexcept {
    return core::to_si(mass).value();
}

// Sphere radius [m] of a body of the given mass and bulk density.
double radius_from_density(core::space::Mass mass, double density) noexcept {
    const double kg = mass_kg(mass);
    if (!(kg > 0.0) || !std::isfinite(kg)) {
        return 0.0;
    }
    return std::cbrt(3.0 * kg / (4.0 * kPi * density));
}

BodyAppearance star_appearance(core::space::Mass mass) {
    BodyAppearance a;
    a.style = SurfaceStyle::star;
    const double m = std::clamp(mass.value(), 0.08, 150.0);
    // APPROXIMATION: generic star from its mass - main-sequence relations
    // R ~ M^0.8, L ~ M^3.5, T = T_sun (L / R^2)^(1/4) - only for stars that
    // the built-in table does not know; within a factor of about two for
    // main-sequence stars of 0.1 to 50 solar masses, wrong for giants and
    // remnants.
    a.nominal_radius = kSunRadius * std::pow(m, 0.8);
    a.luminosity = std::pow(m, 3.5);
    a.temperature = std::clamp(kSunTemperature * std::pow(a.luminosity / std::pow(m, 1.6), 0.25),
                               2400.0, 40000.0);
    return a;
}

} // namespace

BodyAppearance appearance_for(std::string_view name, const core::BodyKind& kind,
                              core::space::Mass mass) {
    const core::BodyKindTag tag = core::tag_of(kind);
    BodyAppearance a;
    switch (tag) {
        case core::BodyKindTag::star:
            a = star_appearance(mass);
            if (first_word_lower(name) == "sun") {
                a.nominal_radius = kSunRadius;
                a.temperature = kSunTemperature;
                a.luminosity = 1.0;
                a.spin_axis = pole_from_equatorial(286.13, 63.87);
            } else if (first_word_lower(name) == "s2") {
                // S2 (S0-2) near Sgr A*: a B0-2.5 V star, T_eff = 28,500 K and
                // R = 5.53 R_sun (Habibi et al. 2017, ApJ 847, 120). Scenarios
                // enter it as a test star of zero mass, which the generic
                // relations would draw as a cool red dwarf.
                constexpr double kS2Radius = 5.53;
                constexpr double kS2Temperature = 28500.0;
                a.nominal_radius = kS2Radius * kSunRadius;
                a.temperature = kS2Temperature;
                a.luminosity =
                    kS2Radius * kS2Radius * std::pow(kS2Temperature / kSunTemperature, 4.0);
            }
            // Photosphere radiance: well above every lit surface, so that the
            // star stays the brightest object after tone mapping.
            a.albedo = blackbody_linear_srgb(a.temperature);
            a.emission = a.albedo * 14.0;
            return a;
        case core::BodyKindTag::black_hole: {
            a.style = SurfaceStyle::black_hole;
            // Shadow radius of a Schwarzschild black hole, sqrt(27) / 2 r_s:
            // the dark disc a distant observer sees.
            const double rs = core::to_si(core::schwarzschild_radius(mass)).value();
            a.nominal_radius = std::sqrt(27.0) / 2.0 * rs;
            a.albedo = glm::dvec3(0.0);
            a.secondary = glm::dvec3(1.0, 0.62, 0.30); // ring tint
            return a;
        }
        case core::BodyKindTag::probe:
            a.style = SurfaceStyle::probe;
            a.nominal_radius = 10.0;
            a.albedo = glm::dvec3(0.55, 0.85, 0.95);
            a.emission = glm::dvec3(0.35, 0.85, 1.0) * 1.6;
            return a;
        case core::BodyKindTag::planet:
        case core::BodyKindTag::moon:
        case core::BodyKindTag::asteroid:
            break;
    }

    const std::string key = first_word_lower(name);
    for (const KnownBody& known : kKnownBodies) {
        if (known.key == key) {
            a.nominal_radius = known.radius_km * 1.0e3;
            a.albedo = known.albedo;
            a.secondary = known.secondary;
            a.style = known.style;
            a.specular = known.specular;
            a.spin_axis = pole_from_equatorial(known.pole_ra, known.pole_dec);
            return a;
        }
    }

    // APPROXIMATION: unknown bodies get a radius from their mass and a bulk
    // density per kind (rocky planet 5500, gas giant above 10 Earth masses
    // 1300, moon 3000, asteroid 2000 kg/m^3) - the scenario format carries
    // no radius - display size only, within a factor of about two for
    // solar-system-like bodies.
    switch (tag) {
        case core::BodyKindTag::planet: {
            const bool giant = mass.value() > 10.0 * kEarthMassInSolar;
            a.nominal_radius = radius_from_density(mass, giant ? 1300.0 : 5500.0);
            a.style = giant ? SurfaceStyle::banded : SurfaceStyle::rocky;
            a.albedo = giant ? glm::dvec3(0.70, 0.62, 0.50) : glm::dvec3(0.32, 0.30, 0.28);
            a.secondary = giant ? glm::dvec3(0.48, 0.38, 0.28) : glm::dvec3(0.18, 0.17, 0.16);
            a.specular = 0.04;
            if (a.nominal_radius <= 0.0) {
                a.nominal_radius = 3.0e6;
            }
            break;
        }
        case core::BodyKindTag::moon:
            a.nominal_radius = radius_from_density(mass, 3000.0);
            a.style = SurfaceStyle::rocky;
            a.albedo = glm::dvec3(0.30, 0.29, 0.27);
            a.secondary = glm::dvec3(0.16, 0.15, 0.14);
            a.specular = 0.03;
            if (a.nominal_radius <= 0.0) {
                a.nominal_radius = 5.0e5;
            }
            break;
        default:
            a.nominal_radius = radius_from_density(mass, 2000.0);
            a.style = SurfaceStyle::rocky;
            a.albedo = glm::dvec3(0.20, 0.19, 0.17);
            a.secondary = glm::dvec3(0.11, 0.10, 0.09);
            a.specular = 0.02;
            if (a.nominal_radius <= 0.0) {
                a.nominal_radius = 5.0e4;
            }
            break;
    }
    return a;
}

double physical_radius(core::space::Length radius, const BodyAppearance& appearance) noexcept {
    const double metres = core::to_si(radius).value();
    return metres > 0.0 && std::isfinite(metres) ? metres : appearance.nominal_radius;
}

double min_screen_radius_px(core::BodyKindTag kind, double scale) noexcept {
    const double s = std::isfinite(scale) ? std::clamp(scale, 0.25, 8.0) : 1.0;
    double px = 2.0;
    switch (kind) {
        case core::BodyKindTag::star:
            px = 5.0;
            break;
        case core::BodyKindTag::planet:
            px = 3.0;
            break;
        case core::BodyKindTag::moon:
            px = 2.0;
            break;
        case core::BodyKindTag::asteroid:
            px = 1.25;
            break;
        case core::BodyKindTag::black_hole:
            px = 4.0;
            break;
        case core::BodyKindTag::probe:
            px = 2.5;
            break;
    }
    return px * s;
}

double pixel_scale(double vertical_fov, int viewport_height) noexcept {
    const double height = std::max(viewport_height, 1);
    return 0.5 * height / std::tan(0.5 * vertical_fov);
}

double projected_radius_px(double radius, double distance, double pixels_per_tangent) noexcept {
    if (!(distance > radius)) {
        return std::numeric_limits<double>::infinity();
    }
    // tan(asin(r / d)) = r / sqrt(d^2 - r^2), formed without squaring overflow.
    const double ratio = radius / distance;
    return ratio / std::sqrt((1.0 - ratio) * (1.0 + ratio)) * pixels_per_tangent;
}

// APPROXIMATION: minimum screen size - in ScaleMode::clamped a body smaller
// than its per-kind minimum screen radius is drawn enlarged to that radius
// (and shaded as a marker with a readable night side) - positions stay exact,
// sizes of unresolved bodies are symbolic; ScaleMode::true_scale is exact.
double visual_radius(double radius, double distance, double pixels_per_tangent, double min_px,
                     ScaleMode mode) noexcept {
    if (mode == ScaleMode::true_scale || !(distance > 0.0) || !(min_px > 0.0)) {
        return radius;
    }
    // Radius whose projection is exactly min_px: solving
    // r / sqrt(d^2 - r^2) * s = m gives r = d m / sqrt(s^2 + m^2).
    const double clamped = distance * min_px / std::hypot(pixels_per_tangent, min_px);
    return std::max(radius, clamped);
}

ProjectedSphere project_sphere(double radius, double distance, double depth,
                               double pixels_per_tangent) noexcept {
    ProjectedSphere out;
    if (!(radius >= 0.0) || !std::isfinite(radius) || !std::isfinite(distance) ||
        !(pixels_per_tangent > 0.0)) {
        out.radius_px = std::numeric_limits<double>::infinity();
        out.lod_scale_px = out.radius_px;
        return out;
    }
    if (radius == 0.0) {
        return out;
    }
    if (!(depth > radius)) {
        // The sphere reaches the plane of the eye: its projection is unbounded.
        out.radius_px = std::numeric_limits<double>::infinity();
        out.lod_scale_px = out.radius_px;
        return out;
    }
    // The distance is at least the depth; rounding of the two may disagree.
    const double d = std::max(distance, depth);
    // Semi-major axis of the projected ellipse: r sqrt(d^2 - r^2) / (z^2 - r^2),
    // with the differences of squares factored for accuracy near the eye
    // plane.
    const double z_squares = (depth - radius) * (depth + radius);
    out.radius_px =
        radius * std::sqrt((d - radius) * (d + radius)) / z_squares * pixels_per_tangent;
    // Worst local magnification of the projection over the sphere: a
    // displacement s at point p moves the image by at most s |p| / z_p^2,
    // with |p| <= d + r and z_p >= z - r.
    const double near_depth = depth - radius;
    out.lod_scale_px = radius * (d + radius) / (near_depth * near_depth) * pixels_per_tangent;
    return out;
}

namespace {

// Measured from make_icosphere(lod) (vertices as stored, in float), rounded
// up; tests/render/test_body_visuals.cpp measures every level again.
//   max_edge_angle: largest angle subtended by an edge [rad]
//   face_depth:     1 - (smallest distance of a face plane from the centre)
// A face seen edge-on at the silhouette pulls the outline in by its full
// depth, which exceeds the sagitta 1 - cos(max_edge_angle / 2) of the longest
// edge (by about a third); the thresholds use the larger of the two.
struct LodGeometry {
    double max_edge_angle;
    double face_depth;
};
constexpr std::array<LodGeometry, kLodCount - 1> kLodGeometry{{
    {1.1071488, 0.20534556},
    {0.62831855, 0.065827636},
    {0.32636623, 0.017753078},
    {0.16483374, 0.0045283776},
    {0.082627480, 0.0011379102},
    {0.041340217, 0.00028484961},
}};

} // namespace

double lod_max_radius_px(std::size_t lod) noexcept {
    if (lod + 1 >= kLodCount) {
        return std::numeric_limits<double>::infinity();
    }
    constexpr double kMaxErrorPx = 0.25;
    const LodGeometry& g = kLodGeometry[lod];
    const double edge_sagitta = 1.0 - std::cos(0.5 * g.max_edge_angle);
    return kMaxErrorPx / std::max(g.face_depth, edge_sagitta);
}

std::size_t select_lod(double projected_px, int bias) noexcept {
    std::size_t lod = kLodCount - 1;
    if (std::isfinite(projected_px)) {
        for (std::size_t i = 0; i < kLodCount; ++i) {
            if (projected_px <= lod_max_radius_px(i)) {
                lod = i;
                break;
            }
        }
    }
    const int shifted = static_cast<int>(lod) + std::clamp(bias, -8, 8);
    return static_cast<std::size_t>(std::clamp(shifted, 0, static_cast<int>(kLodCount) - 1));
}

bool sphere_in_frustum(const ViewFrustum& frustum, const glm::dvec3& centre,
                       double radius) noexcept {
    const double x = glm::dot(centre, frustum.right);
    const double y = glm::dot(centre, frustum.up);
    const double depth = -glm::dot(centre, frustum.back);
    if (depth < -radius) {
        return false; // entirely behind the eye
    }
    const double ty = frustum.tan_half_fov_y;
    const double tx = ty * frustum.aspect;
    // Signed distances to the four side planes through the eye.
    const double top = (y - depth * ty) / std::sqrt(1.0 + ty * ty);
    const double bottom = (-y - depth * ty) / std::sqrt(1.0 + ty * ty);
    const double right = (x - depth * tx) / std::sqrt(1.0 + tx * tx);
    const double left = (-x - depth * tx) / std::sqrt(1.0 + tx * tx);
    return top <= radius && bottom <= radius && right <= radius && left <= radius;
}

double relative_irradiance(double luminosity, double distance_m, double exponent) noexcept {
    const double au = core::constants::astronomical_unit.value();
    const double d = std::max(distance_m, 1.0);
    return luminosity * std::pow(au / d, exponent);
}

glm::dvec3 blackbody_linear_srgb(double kelvin) noexcept {
    // APPROXIMATION: black-body chromaticity - cubic fits of the Planckian
    // locus (Kim et al. 2002, as used for CIE 1931 2-degree xy) instead of
    // integrating the Planck spectrum against the colour matching functions -
    // within about 1e-3 in xy from 1667 K to 25000 K.
    const double t = std::clamp(std::isfinite(kelvin) ? kelvin : 5772.0, 1667.0, 25000.0);
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double x = t <= 4000.0
                         ? -0.2661239e9 / t3 - 0.2343589e6 / t2 + 0.8776956e3 / t + 0.179910
                         : -3.0258469e9 / t3 + 2.1070379e6 / t2 + 0.2226347e3 / t + 0.240390;
    const double x2 = x * x;
    const double x3 = x2 * x;
    double y = 0.0;
    if (t <= 2222.0) {
        y = -1.1063814 * x3 - 1.34811020 * x2 + 2.18555832 * x - 0.20219683;
    } else if (t <= 4000.0) {
        y = -0.9549476 * x3 - 1.37418593 * x2 + 2.09137015 * x - 0.16748867;
    } else {
        y = 3.0817580 * x3 - 5.87338670 * x2 + 3.75112997 * x - 0.37001483;
    }
    const double big_x = x / y;
    const double big_z = (1.0 - x - y) / y;
    glm::dvec3 rgb{3.2404542 * big_x - 1.5371385 - 0.4985314 * big_z,
                   -0.9692660 * big_x + 1.8760108 + 0.0415560 * big_z,
                   0.0556434 * big_x - 0.2040259 + 1.0572252 * big_z};
    rgb = glm::max(rgb, glm::dvec3(0.0));
    const double luminance = 0.2126 * rgb.r + 0.7152 * rgb.g + 0.0722 * rgb.b;
    return luminance > 0.0 ? rgb / luminance : glm::dvec3(1.0);
}

} // namespace orrery::render
