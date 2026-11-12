#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <glm/geometric.hpp>

#include "orrery/core/sim_snapshot.h"
#include "orrery/core/units.h"
#include "orrery/render/body_visuals.h"
#include "orrery/render/camera.h"
#include "orrery/render/icosphere.h"
#include "orrery/render/render_settings.h"
#include "orrery/render/space_renderer.h"

using namespace orrery;
using namespace orrery::render;
using Catch::Approx;

namespace {

constexpr double kAu = 149597870700.0;

double luminance_of(const glm::dvec3& c) {
    return 0.2126 * c.r + 0.7152 * c.g + 0.0722 * c.b;
}

std::shared_ptr<const core::SimSnapshot> sun_earth_snapshot(double earth_x_au) {
    core::BodyStore bodies;
    REQUIRE(bodies.add({"Sun", core::Star{}, core::space::Mass(1.0), {}, {}}));
    REQUIRE(bodies.add(
        {"Earth",
         core::Planet{},
         core::space::Mass(3.0e-6),
         {core::space::Length(earth_x_au), core::space::Length(0.0), core::space::Length(0.0)},
         {}}));
    return core::SimSnapshot::capture(core::SimState(core::space::Time(0.0), std::move(bodies)));
}

constexpr double kPi = 3.141592653589793;
constexpr double kDegree = kPi / 180.0;

// Rotation by `angle` about the unit `axis` (Rodrigues).
struct Rotation {
    glm::dvec3 axis{0.0, 0.0, 1.0};
    double angle = 0.0;

    [[nodiscard]] glm::dvec3 apply(const glm::dvec3& v) const {
        const double c = std::cos(angle);
        const double s = std::sin(angle);
        return v * c + glm::cross(axis, v) * s + axis * glm::dot(axis, v) * (1.0 - c);
    }

    // Rotation taking the direction `from` onto the direction `to`.
    static Rotation between(const glm::dvec3& from, const glm::dvec3& to) {
        const glm::dvec3 a = glm::normalize(from);
        const glm::dvec3 b = glm::normalize(to);
        const glm::dvec3 axis = glm::cross(a, b);
        const double s = glm::length(axis);
        if (s < 1.0e-12) {
            if (glm::dot(a, b) > 0.0) {
                return {};
            }
            const glm::dvec3 helper =
                std::abs(a.x) < 0.9 ? glm::dvec3(1.0, 0.0, 0.0) : glm::dvec3(0.0, 1.0, 0.0);
            return {glm::normalize(glm::cross(a, helper)), kPi};
        }
        return {axis / s, std::atan2(s, glm::dot(a, b))};
    }
};

// Fixed, arbitrary orientations (deterministic on every platform).
std::vector<Rotation> arbitrary_rotations() {
    return {
        {glm::normalize(glm::dvec3(0.3, -0.8, 0.5)), 0.71},
        {glm::normalize(glm::dvec3(-0.9, 0.1, 0.4)), 2.3},
        {glm::normalize(glm::dvec3(0.2, 0.2, -1.0)), 4.1},
        {glm::normalize(glm::dvec3(1.0, 0.6, 0.05)), 5.6},
    };
}

// Geometry measured from a generated mesh (independent of the LOD code).
struct MeshBounds {
    double max_edge_angle = 0.0;    // largest angle subtended by an edge [rad]
    double min_face_distance = 1.0; // closest face plane to the centre (unit sphere)
    glm::dvec3 longest_edge_midpoint{1.0, 0.0, 0.0};
    glm::dvec3 deepest_face_normal{1.0, 0.0, 0.0};
};

MeshBounds measure_mesh(const IcosphereMesh& mesh) {
    MeshBounds bounds;
    for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const std::array<glm::dvec3, 3> v{glm::dvec3(mesh.positions[mesh.indices[t]]),
                                          glm::dvec3(mesh.positions[mesh.indices[t + 1]]),
                                          glm::dvec3(mesh.positions[mesh.indices[t + 2]])};
        for (std::size_t e = 0; e < 3; ++e) {
            const glm::dvec3& a = v[e];
            const glm::dvec3& b = v[(e + 1) % 3];
            const double angle = std::atan2(glm::length(glm::cross(a, b)), glm::dot(a, b));
            if (angle > bounds.max_edge_angle) {
                bounds.max_edge_angle = angle;
                bounds.longest_edge_midpoint = glm::normalize(a + b);
            }
        }
        const glm::dvec3 normal = glm::normalize(glm::cross(v[1] - v[0], v[2] - v[0]));
        const double distance = std::abs(glm::dot(normal, v[0]));
        if (distance < bounds.min_face_distance) {
            bounds.min_face_distance = distance;
            bounds.deepest_face_normal = normal;
        }
    }
    return bounds;
}

// Support of the exact perspective projection of a sphere (centre c in camera
// space, the camera looks along -Z) in the unit screen direction m: the
// largest m . (x, y) / -z over the sphere, from the plane through the eye
// that touches it.
double sphere_support(const glm::dvec3& c, double r, const glm::dvec2& m) {
    const double a = m.x * c.x + m.y * c.y;
    const double z = -c.z;
    return (a * z + r * std::sqrt(a * a + z * z - r * r)) / ((z - r) * (z + r));
}

// Largest gap [px] between the projected outline of the sphere and that of
// the mesh (rotated, scaled to radius r, centred at c), over 2048 screen
// directions: for convex outlines the largest support difference is their
// Hausdorff distance.
double silhouette_error_px(const IcosphereMesh& mesh, const Rotation& rotation, const glm::dvec3& c,
                           double r, double pps) {
    std::vector<glm::dvec2> projected;
    projected.reserve(mesh.positions.size());
    for (const glm::vec3& p : mesh.positions) {
        const glm::dvec3 v = c + r * rotation.apply(glm::dvec3(p));
        projected.emplace_back(v.x / -v.z, v.y / -v.z);
    }
    double worst = 0.0;
    constexpr int kDirections = 2048;
    for (int k = 0; k < kDirections; ++k) {
        const double angle = 2.0 * kPi * k / kDirections;
        const glm::dvec2 m{std::cos(angle), std::sin(angle)};
        double polygon = -std::numeric_limits<double>::infinity();
        for (const glm::dvec2& q : projected) {
            polygon = std::max(polygon, glm::dot(m, q));
        }
        worst = std::max(worst, sphere_support(c, r, m) - polygon);
    }
    return worst * pps;
}

// Free camera at the world origin looking along -Z (render space == world).
Camera free_camera(double vertical_fov) {
    Camera camera;
    CameraLens lens;
    lens.vertical_fov = vertical_fov;
    REQUIRE(camera.set_lens(lens).has_value());
    camera.set_mode(FreeMode{});
    camera.update(0.0, CameraInput{}, BodyFrameLookup{});
    return camera;
}

// One body at `position` [m] with an explicit physical radius [m].
std::shared_ptr<const core::SimSnapshot> single_body_snapshot(const core::BodyKind& kind,
                                                              const char* name,
                                                              const glm::dvec3& position,
                                                              double radius) {
    core::BodyStore bodies;
    REQUIRE(
        bodies.add({name,
                    kind,
                    core::space::Mass(1.0e-9),
                    {core::space::Length(position.x / kAu), core::space::Length(position.y / kAu),
                     core::space::Length(position.z / kAu)},
                    {}}));
    REQUIRE(bodies.set_radius(0, core::space::Length(radius / kAu)));
    return core::SimSnapshot::capture(core::SimState(core::space::Time(0.0), std::move(bodies)));
}

Camera orbit_camera(const glm::dvec3& focus, double distance, double yaw, double pitch) {
    Camera camera;
    OrbitMode orbit;
    orbit.focus = focus;
    orbit.distance = distance;
    orbit.yaw = yaw;
    orbit.pitch = pitch;
    camera.set_mode(orbit);
    camera.update(0.0, CameraInput{}, BodyFrameLookup{});
    return camera;
}

} // namespace

TEST_CASE("LOD selection is monotonic in the projected size and honours the bias",
          "[render][lod]") {
    CHECK(std::isinf(lod_max_radius_px(kLodCount - 1)));
    for (std::size_t lod = 0; lod + 2 < kLodCount; ++lod) {
        CHECK(lod_max_radius_px(lod + 1) > lod_max_radius_px(lod));
    }
    const double between_2_and_3 = 0.5 * (lod_max_radius_px(1) + lod_max_radius_px(2));
    CHECK(select_lod(0.5, 0) == 0);
    CHECK(select_lod(lod_max_radius_px(0) * 1.01, 0) == 1);
    CHECK(select_lod(between_2_and_3, 0) == 2);
    CHECK(select_lod(1.0e5, 0) == kLodCount - 1);
    CHECK(select_lod(std::numeric_limits<double>::infinity(), 0) == kLodCount - 1);
    // Bias shifts and clamps.
    CHECK(select_lod(between_2_and_3, 1) == 3);
    CHECK(select_lod(between_2_and_3, -5) == 0);
    CHECK(select_lod(between_2_and_3, 50) == kLodCount - 1);
    std::size_t previous = 0;
    double px = 0.1;
    while (px < 5000.0) {
        const std::size_t lod = select_lod(px, 0);
        CHECK(lod >= previous);
        previous = lod;
        px *= 1.3;
    }
}

TEST_CASE("Every LOD threshold keeps the measured silhouette error within a quarter pixel",
          "[render][lod]") {
    for (std::size_t lod = 0; lod + 1 < kLodCount; ++lod) {
        const IcosphereMesh mesh = make_icosphere(static_cast<int>(lod));
        const MeshBounds bounds = measure_mesh(mesh);
        const double threshold = lod_max_radius_px(lod);
        INFO("LOD " << lod << ": threshold " << threshold << " px, longest edge "
                    << bounds.max_edge_angle << " rad, closest face plane "
                    << bounds.min_face_distance);
        // The selection boundary is the advertised threshold.
        CHECK(select_lod(threshold * (1.0 - 1.0e-9), 0) == lod);
        CHECK(select_lod(threshold * (1.0 + 1.0e-6), 0) == lod + 1);
        // The sagitta of the longest edge must stay within a quarter pixel,
        // and so must the deepest face: the mesh lies between the spheres of
        // radius min_face_distance and 1, and a face seen edge-on at the
        // silhouette pulls the outline in by the full depth of its plane.
        CHECK(threshold * (1.0 - std::cos(0.5 * bounds.max_edge_angle)) <= 0.25);
        CHECK(threshold * (1.0 - bounds.min_face_distance) <= 0.25);

        // The projected outline itself, for a unit sphere on the optical axis
        // whose projected radius is exactly the threshold, in the worst
        // orientation (deepest face and longest edge at the silhouette) and
        // in a set of arbitrary ones.
        const double distance = 1.0e4;
        const double pps = threshold / projected_radius_px(1.0, distance, 1.0);
        const glm::dvec3 centre{0.0, 0.0, -distance};
        const double tilt = std::asin(1.0 / distance);
        const glm::dvec3 silhouette_normal{std::cos(tilt), 0.0, std::sin(tilt)};
        std::vector<Rotation> rotations{
            Rotation::between(bounds.deepest_face_normal, silhouette_normal),
            Rotation::between(bounds.longest_edge_midpoint, silhouette_normal),
        };
        for (const Rotation& r : arbitrary_rotations()) {
            rotations.push_back(r);
        }
        double worst = 0.0;
        for (const Rotation& r : rotations) {
            worst = std::max(worst, silhouette_error_px(mesh, r, centre, 1.0, pps));
        }
        INFO("measured outline error " << worst << " px");
        CHECK(worst <= 0.25);
        // The worst orientation really reaches the face-depth bound.
        CHECK(worst >= 0.9 * threshold * (1.0 - bounds.min_face_distance));
    }
}

TEST_CASE("Projected sphere bounds and LOD follow the off-axis perspective ellipse",
          "[render][lod][prepare]") {
    struct Case {
        int width;
        int height;
        double fov_degrees;
        glm::dvec3 centre; // camera space [m], the camera looks along -Z
        double radius;     // [m]
    };
    const std::vector<Case> cases{
        {1080, 1080, 90.0, {90.0, 0.0, -100.0}, 1.5},  // near the right edge
        {1080, 1080, 90.0, {0.0, 0.0, -100.0}, 1.5},   // on the optical axis
        {1080, 1080, 90.0, {70.0, 70.0, -100.0}, 1.5}, // towards a corner
        {1920, 1080, 120.0, {-150.0, 60.0, -100.0}, 4.0},
        {600, 1200, 100.0, {20.0, -105.0, -100.0}, 2.0},
        {1280, 720, 60.0, {40.0, 30.0, -100.0}, 3.0},
        {1080, 1080, 90.0, {6.0, 3.0, -14.0}, 9.0}, // large and close
    };
    for (const Case& c : cases) {
        INFO("viewport " << c.width << "x" << c.height << ", fov " << c.fov_degrees
                         << " deg, centre (" << c.centre.x << ", " << c.centre.y << ", "
                         << c.centre.z << ") m, radius " << c.radius << " m");
        const Camera camera = free_camera(c.fov_degrees * kDegree);
        const FrameInfo frame{c.width, c.height};
        const double pps = pixel_scale(c.fov_degrees * kDegree, c.height);
        // Largest screen extent of the exact projection: half the widest of
        // the projected widths over all screen directions.
        double expected = 0.0;
        for (int k = 0; k < 3600; ++k) {
            const double angle = kPi * k / 3600.0;
            const glm::dvec2 m{std::cos(angle), std::sin(angle)};
            expected = std::max(expected, 0.5 * (sphere_support(c.centre, c.radius, m) +
                                                 sphere_support(c.centre, c.radius, -m)));
        }
        expected *= pps;

        RenderSettings settings;
        settings.scale_mode = ScaleMode::true_scale;
        const auto planet = single_body_snapshot(core::Planet{}, "Rock", c.centre, c.radius);
        const PreparedBodies prepared = prepare_bodies(*planet, camera, frame, settings);
        REQUIRE(prepared.screen.size() == 1);
        const BodyScreenInfo& info = prepared.screen[0];
        REQUIRE(info.visible);
        CHECK(info.visual_radius == Approx(c.radius).epsilon(1e-12));
        CHECK(info.radius_px == Approx(expected).epsilon(1e-6));
        // The chosen mesh keeps the outline within a quarter pixel at this
        // exact off-axis geometry.
        const IcosphereMesh mesh = make_icosphere(static_cast<int>(info.lod));
        const MeshBounds bounds = measure_mesh(mesh);
        const glm::dvec3 towards_eye = glm::normalize(-c.centre);
        std::vector<Rotation> rotations{
            Rotation::between(bounds.deepest_face_normal,
                              glm::normalize(glm::cross(towards_eye, glm::dvec3(0.0, 1.0, 0.0)))),
            Rotation::between(bounds.deepest_face_normal, glm::dvec3(1.0, 0.0, 0.0)),
        };
        for (const Rotation& r : arbitrary_rotations()) {
            rotations.push_back(r);
        }
        double worst = 0.0;
        for (const Rotation& r : rotations) {
            worst = std::max(worst, silhouette_error_px(mesh, r, c.centre, c.radius, pps));
        }
        INFO("LOD " << info.lod << ", measured outline error " << worst << " px");
        CHECK(worst <= 0.25);

        // Star glare sprites are sized from the same bound.
        const auto star = single_body_snapshot(core::Star{}, "Vega", c.centre, c.radius);
        const PreparedBodies lit = prepare_bodies(*star, camera, frame, settings);
        REQUIRE(lit.glows.size() == 1);
        CHECK(lit.glows[0].color_disc.w == Approx(expected).epsilon(1e-5));
    }
}

TEST_CASE("The clamped scale mode enlarges only bodies below the minimum screen size",
          "[render][scale]") {
    const double pps = pixel_scale(0.8726646259971648, 1080); // 50 degrees, 1080 rows
    CHECK(pps == Approx(540.0 / std::tan(0.4363323129985824)));
    const double earth = 6.371e6;

    // At 1 AU the Earth is far below a pixel: clamped to the minimum.
    const double far = kAu;
    CHECK(projected_radius_px(earth, far, pps) < 0.1);
    const double clamped = visual_radius(earth, far, pps, 3.0, ScaleMode::clamped);
    CHECK(projected_radius_px(clamped, far, pps) == Approx(3.0).epsilon(1e-12));
    CHECK(visual_radius(earth, far, pps, 3.0, ScaleMode::true_scale) == earth);

    // Close up the true radius wins and is not changed.
    const double near = 5.0e7;
    CHECK(projected_radius_px(earth, near, pps) > 3.0);
    CHECK(visual_radius(earth, near, pps, 3.0, ScaleMode::clamped) == earth);

    // The eye inside the sphere projects to infinity.
    CHECK(std::isinf(projected_radius_px(earth, earth * 0.5, pps)));
    // Per-kind minimums, scaled and clamped by the user factor.
    CHECK(min_screen_radius_px(core::BodyKindTag::star, 1.0) >
          min_screen_radius_px(core::BodyKindTag::planet, 1.0));
    CHECK(min_screen_radius_px(core::BodyKindTag::planet, 2.0) ==
          2.0 * min_screen_radius_px(core::BodyKindTag::planet, 1.0));
    CHECK(min_screen_radius_px(core::BodyKindTag::moon, 100.0) ==
          8.0 * min_screen_radius_px(core::BodyKindTag::moon, 1.0));
}

TEST_CASE("Sphere frustum culling keeps visible spheres and rejects hidden ones",
          "[render][culling]") {
    ViewFrustum f; // camera looks along -Z
    f.tan_half_fov_y = std::tan(0.4);
    f.aspect = 16.0 / 9.0;
    CHECK(sphere_in_frustum(f, {0.0, 0.0, -10.0}, 1.0));
    CHECK_FALSE(sphere_in_frustum(f, {0.0, 0.0, 10.0}, 1.0)); // behind
    CHECK(sphere_in_frustum(f, {0.0, 0.0, 0.5}, 1.0));        // around the eye
    CHECK_FALSE(sphere_in_frustum(f, {0.0, 100.0, -10.0}, 1.0));
    CHECK(sphere_in_frustum(f, {0.0, 4.3, -10.0}, 0.5)); // straddles the top plane
    CHECK_FALSE(sphere_in_frustum(f, {-100.0, 0.0, -10.0}, 1.0));
    CHECK(sphere_in_frustum(f, {-7.0, 0.0, -10.0}, 1.0)); // inside the wide horizontal field
}

TEST_CASE("Body appearance uses known radii and colours and falls back per kind",
          "[render][appearance]") {
    const auto earth = appearance_for("Earth", core::Planet{}, core::space::Mass(3.0e-6));
    CHECK(earth.nominal_radius == Approx(6.371e6));
    CHECK(earth.style == SurfaceStyle::terrestrial);
    // Earth's pole is 23.44 degrees from the ecliptic pole.
    CHECK(std::acos(earth.spin_axis.z) == Approx(23.4392911 * 3.141592653589793 / 180.0));
    const auto jupiter =
        appearance_for("Jupiter system", core::Planet{}, core::space::Mass(9.5e-4));
    CHECK(jupiter.nominal_radius == Approx(6.9911e7));
    CHECK(jupiter.style == SurfaceStyle::banded);
    CHECK(appearance_for("MOON", core::Moon{}, core::space::Mass(3.7e-8)).nominal_radius ==
          Approx(1.7374e6));

    const auto sun = appearance_for("Sun", core::Star{}, core::space::Mass(1.0));
    CHECK(sun.style == SurfaceStyle::star);
    CHECK(sun.temperature == 5772.0);
    CHECK(sun.luminosity == 1.0);
    CHECK(luminance_of(sun.emission) > 10.0);

    // Unknown bodies: radius from mass and density.
    const auto rock = appearance_for("Rock", core::Planet{}, core::space::Mass(3.0e-6));
    CHECK(rock.nominal_radius == Approx(6.37e6).epsilon(0.05)); // Earth mass at 5500 kg/m^3
    const auto hole = appearance_for("Gate", core::BlackHole{0.0}, core::space::Mass(4.3e6));
    const double rs = 2.0 * 1.32712440041e20 * 4.3e6 / (299792458.0 * 299792458.0);
    CHECK(hole.nominal_radius == Approx(std::sqrt(27.0) / 2.0 * rs).epsilon(1e-6));
    CHECK(hole.style == SurfaceStyle::black_hole);
    CHECK(appearance_for("Probe-1", core::Probe{}, core::space::Mass(1e-27)).style ==
          SurfaceStyle::probe);
    CHECK(physical_radius(core::space::Length(1.0e-5), earth) == Approx(1.0e-5 * kAu));
    CHECK(physical_radius(core::space::Length(0.0), earth) == earth.nominal_radius);
}

TEST_CASE("Black-body colours run from red to blue with unit luminance", "[render][appearance]") {
    const glm::dvec3 cool = blackbody_linear_srgb(3000.0);
    const glm::dvec3 sun = blackbody_linear_srgb(5772.0);
    const glm::dvec3 hot = blackbody_linear_srgb(20000.0);
    for (const glm::dvec3& c : {cool, sun, hot}) {
        CHECK(luminance_of(c) == Approx(1.0).epsilon(1e-9));
    }
    CHECK(cool.r > cool.b);
    CHECK(hot.b > hot.r);
    CHECK(cool.r / cool.b > sun.r / sun.b);
    CHECK(sun.r / sun.b > hot.r / hot.b);
    // The Sun is close to white.
    CHECK(sun.r / sun.b < 1.6);
    CHECK(sun.r / sun.b > 1.0);
}

TEST_CASE("Irradiance falls off with the configured exponent", "[render][lighting]") {
    CHECK(relative_irradiance(1.0, kAu, 2.0) == Approx(1.0));
    CHECK(relative_irradiance(1.0, 4.0 * kAu, 2.0) == Approx(1.0 / 16.0));
    CHECK(relative_irradiance(1.0, 4.0 * kAu, 0.5) == Approx(0.5));
    CHECK(relative_irradiance(3.0, 4.0 * kAu, 0.0) == Approx(3.0));
}

TEST_CASE("Render settings are clamped into their documented ranges", "[render][settings]") {
    RenderSettings s;
    s.min_radius_scale = 100.0;
    s.lod_bias = -9;
    s.trail_length = 1;
    s.exposure = std::numeric_limits<double>::quiet_NaN();
    s.light_falloff_exponent = 7.0;
    const RenderSettings c = sanitized(s);
    CHECK(c.min_radius_scale == 8.0);
    CHECK(c.lod_bias == -3);
    CHECK(c.trail_length == 16);
    CHECK(c.exposure == RenderSettings{}.exposure);
    CHECK(c.light_falloff_exponent == 2.0);
    CHECK(sanitized(RenderSettings{}) == RenderSettings{});
}

TEST_CASE("Frame preparation lights planets from the star and picks LODs by size",
          "[render][prepare]") {
    const auto snapshot = sun_earth_snapshot(1.0);
    const FrameInfo frame{1280, 720};
    RenderSettings settings;

    // Far view of the whole system: both bodies clamped, coarse LODs.
    const Camera far = orbit_camera({0.0, 0.0, 0.0}, 3.0 * kAu, 0.3, 0.5);
    const PreparedBodies wide = prepare_bodies(*snapshot, far, frame, settings);
    REQUIRE(wide.instances.size() == 2);
    REQUIRE(wide.screen.size() == 2);
    CHECK(wide.screen[0].radius_px == Approx(min_screen_radius_px(core::BodyKindTag::star, 1.0)));
    // The clamp is angular (min_px at the image centre); off the optical axis
    // the projected ellipse is stretched, so the Earth's largest screen
    // extent is at least the minimum.
    const double planet_min = min_screen_radius_px(core::BodyKindTag::planet, 1.0);
    CHECK(wide.screen[1].radius_px >= planet_min * (1.0 - 1.0e-9));
    CHECK(wide.screen[1].radius_px < planet_min * 1.2);
    CHECK(wide.screen[1].lod <= 1);
    CHECK(wide.glows.size() == 1);
    // The Earth instance is lit from the Sun (towards -X).
    const BodyInstance* earth = nullptr;
    for (const BodyInstance& inst : wide.instances) {
        if (inst.albedo_style.w == static_cast<float>(SurfaceStyle::terrestrial)) {
            earth = &inst;
        }
    }
    REQUIRE(earth != nullptr);
    CHECK(earth->light_seed.x == Approx(-1.0).margin(1e-6));
    CHECK(earth->irradiance.g > 0.0F);
    // The projected Sun sits at the image centre (the orbit focus).
    CHECK(wide.screen[0].pixel.x == Approx(640.0).margin(0.5));
    CHECK(wide.screen[0].pixel.y == Approx(360.0).margin(0.5));

    // True scale: the Earth's true radius at 3 AU is far below a pixel.
    settings.scale_mode = ScaleMode::true_scale;
    const PreparedBodies tiny = prepare_bodies(*snapshot, far, frame, settings);
    CHECK(tiny.screen[1].radius_px < 0.05);
    CHECK(tiny.screen[1].visual_radius == Approx(6.371e6));

    // Close-up of the Earth: true radius, fine LOD; the Sun behind the camera is culled.
    const glm::dvec3 earth_world{kAu, 0.0, 0.0};
    const Camera close = orbit_camera(earth_world, 2.0e7, 3.141592653589793, 0.0);
    settings.scale_mode = ScaleMode::clamped;
    const PreparedBodies near = prepare_bodies(*snapshot, close, frame, settings);
    CHECK(near.culled == 1);
    REQUIRE(near.instances.size() == 1);
    CHECK(near.screen[1].visual_radius == Approx(6.371e6));
    CHECK(near.screen[1].lod >= 4);
    CHECK_FALSE(near.screen[0].visible);
}

TEST_CASE("Frame preparation is float-safe far from the world origin", "[render][prepare]") {
    // A Sun-Earth pair 1e4 AU from the origin, seen from 3e7 m: the render
    // position of the Earth is exact to float rounding.
    const auto snapshot = sun_earth_snapshot(1.0e4);
    const glm::dvec3 earth_world{1.0e4 * kAu, 0.0, 0.0};
    const Camera camera = orbit_camera(earth_world, 3.0e7, 0.7, 0.2);
    const PreparedBodies prepared = prepare_bodies(*snapshot, camera, FrameInfo{800, 600}, {});
    const glm::dvec3 expected = camera.to_render_double(earth_world);
    const BodyInstance& inst = prepared.instances.back();
    const glm::dvec3 got(inst.centre_radius.x, inst.centre_radius.y, inst.centre_radius.z);
    CHECK(glm::length(got - expected) / glm::length(expected) < 1.0e-6);
    CHECK(glm::length(expected) == Approx(3.0e7).epsilon(1e-9));
}

TEST_CASE("A star coincident with a planet gives finite instance data",
          "[render][prepare][lighting]") {
    // The planet is larger than the star, so its surface is what the camera
    // sees. Offsets: exactly coincident, sub-metre, and so small that the
    // squared length of the difference underflows.
    for (const double offset_au : {0.0, 1.0e-12, 1.0e-200, -1.0e-300}) {
        INFO("planet offset " << offset_au << " AU");
        core::BodyStore bodies;
        REQUIRE(bodies.add({"Sun", core::Star{}, core::space::Mass(1.0), {}, {}}));
        REQUIRE(bodies.add({"Rock",
                            core::Planet{},
                            core::space::Mass(3.0e-6),
                            {core::space::Length(offset_au), core::space::Length(0.5 * offset_au),
                             core::space::Length(0.0)},
                            {}}));
        REQUIRE(bodies.set_radius(0, core::space::Length(1.0e-4)));
        REQUIRE(bodies.set_radius(1, core::space::Length(2.0e-4)));
        const auto snapshot =
            core::SimSnapshot::capture(core::SimState(core::space::Time(0.0), std::move(bodies)));
        const Camera camera = orbit_camera({0.0, 0.0, 0.0}, 0.01 * kAu, 0.4, 0.3);
        const PreparedBodies prepared =
            prepare_bodies(*snapshot, camera, FrameInfo{640, 480}, RenderSettings{});
        REQUIRE(prepared.instances.size() == 2);
        for (const BodyInstance& inst : prepared.instances) {
            for (const glm::vec4& field :
                 {inst.centre_radius, inst.albedo_style, inst.secondary_specular, inst.light_seed,
                  inst.irradiance, inst.emission, inst.axis}) {
                for (int k = 0; k < 4; ++k) {
                    CHECK(std::isfinite(field[k]));
                }
            }
            const glm::dvec3 light(inst.light_seed);
            CHECK(glm::length(light) == Approx(1.0).margin(1e-6));
        }
        for (const GlowInstance& glow : prepared.glows) {
            for (int k = 0; k < 4; ++k) {
                CHECK(std::isfinite(glow.centre_kind[k]));
                CHECK(std::isfinite(glow.color_disc[k]));
            }
        }
    }
}
