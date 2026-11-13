#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <glm/geometric.hpp>

#include "../gl_test_support.h"
#include "orrery/aero/lbm/lbm2d.h"
#include "orrery/aero/lbm/solid_mask.h"
#include "orrery/aero/panel_method.h"
#include "orrery/geom/naca.h"
#include "orrery/geom/primitives.h"
#include "orrery/render/aero_view.h"
#include "orrery/render/png.h"
#include "orrery/render/ssim.h"

// Golden snapshots of the aero view (docs/aero-visualisation.md): the
// pressure coefficient on a NACA 0012 wing at 4 degrees from the 3D panel
// method, and three consecutive frames of the vortex street behind a
// circular cylinder at Re 150 from the 2D lattice Boltzmann solver (CPU).
// References live in tests/render/golden/aero/ and are written only with
//   ORRERY_UPDATE_GOLDEN=1 orrery_test_render "[golden]"
// (review the images before committing them).

using namespace orrery;
using namespace orrery::render;
using orrery::render::test::environment_flag;
using orrery::render::test::renderer_or_skip;

namespace {

constexpr double kSsimThreshold = 0.98;
constexpr double kDegree = 3.141592653589793 / 180.0;

std::filesystem::path output_path(const std::string& name) {
    const std::filesystem::path dir = std::filesystem::path(ORRERY_RENDER_OUTPUT_DIR) / "aero";
    std::filesystem::create_directories(dir);
    return dir / (name + "_actual.png");
}

// Compares with (or, with ORRERY_UPDATE_GOLDEN=1, writes) the golden image.
void check_golden(const ImageRgba8& image, const std::string& name) {
    const std::filesystem::path actual = output_path(name);
    REQUIRE(write_png(actual, image).has_value());
    const std::filesystem::path golden =
        std::filesystem::path(ORRERY_RENDER_GOLDEN_DIR) / "aero" / (name + ".png");
    if (environment_flag("ORRERY_UPDATE_GOLDEN")) {
        std::filesystem::create_directories(golden.parent_path());
        REQUIRE(write_png(golden, image).has_value());
        WARN("ORRERY_UPDATE_GOLDEN=1: wrote the golden reference " << golden.string()
                                                                   << "; review it before "
                                                                      "committing");
        return;
    }
    if (!std::filesystem::exists(golden)) {
        FAIL("golden reference " << golden.string()
                                 << " is missing; generate it with ORRERY_UPDATE_GOLDEN=1 "
                                    "orrery_test_render \"[golden]\" and review it");
    }
    auto reference = read_png(golden);
    REQUIRE(reference.has_value());
    auto score = ssim(reference.value(), image);
    REQUIRE(score.has_value());
    std::cout << "measured: aero golden " << name << " SSIM " << score.value().value << "\n";
    INFO("SSIM R " << score.value().channel[0] << " G " << score.value().channel[1] << " B "
                   << score.value().channel[2] << "; actual image: " << actual.string());
    CHECK(score.value().value > kSsimThreshold);
}

struct Pixel {
    double r = 0.0;
    double g = 0.0;
    double b = 0.0;
};

Pixel pixel_at(const ImageRgba8& image, double x, double y) {
    const auto px = static_cast<std::uint32_t>(std::clamp(x, 0.0, image.width - 1.0));
    const auto py = static_cast<std::uint32_t>(std::clamp(y, 0.0, image.height - 1.0));
    const std::size_t o = image.offset(px, py);
    return {static_cast<double>(image.pixels[o]), static_cast<double>(image.pixels[o + 1]),
            static_cast<double>(image.pixels[o + 2])};
}

// Pixel position of a world point (x right, y down).
glm::dvec2 project(const Camera& camera, const FrameInfo& frame, const glm::dvec3& world) {
    const glm::vec4 clip =
        camera.view_projection_matrix(frame.aspect()) * glm::vec4(camera.to_render(world), 1.0F);
    const double x = clip.x / clip.w;
    const double y = clip.y / clip.w;
    return {(x * 0.5 + 0.5) * frame.width, (0.5 - y * 0.5) * frame.height};
}

Camera turntable_camera(const TurntableMode& mode, double fov_degrees) {
    Camera camera;
    CameraLens lens;
    lens.vertical_fov = fov_degrees * kDegree;
    lens.near_plane = 0.01;
    REQUIRE(camera.set_lens(lens).has_value());
    camera.set_mode(mode);
    camera.update(0.0, CameraInput{}, BodyFrameLookup{});
    return camera;
}

// NACA 0012 rectangular wing, aspect ratio 4, at 4 degrees.
struct WingCase {
    geom::NacaWing wing;
    aero::FlowConditions conditions;
    aero::AeroResult result;
};

WingCase solve_wing() {
    WingCase c;
    const geom::Naca4 airfoil = *geom::Naca4::from_designation("0012");
    c.wing = geom::naca_wing(airfoil, 1.0, 4.0, 24, 20);
    aero::AeroGeometry geometry;
    geometry.mesh = c.wing.mesh;
    geometry.trailing_edge = c.wing.trailing_edge;
    geometry.reference = {4.0, 1.0, {0.25, 0.0, 0.0}};
    c.conditions.speed = 30.0;
    c.conditions.angle_of_attack = 4.0 * kDegree;
    aero::PanelSolver3D solver;
    solver.setup(geometry, c.conditions);
    while (!solver.step()) {
    }
    c.result = solver.result();
    return c;
}

} // namespace

TEST_CASE("Golden snapshot: NACA 0012 wing Cp at 4 degrees with legend",
          "[render][aero][gl][golden]") {
    constexpr int kWidth = 1280;
    constexpr int kHeight = 720;
    OffscreenRenderer renderer = renderer_or_skip(kWidth, kHeight, aero_hdr_settings());
    auto created = AeroView::create();
    INFO((created ? std::string() : created.error()));
    REQUIRE(created.has_value());
    AeroView view = std::move(created).value();

    const WingCase c = solve_wing();
    INFO("CL " << c.result.cl << " CD " << c.result.cd);
    std::cout << "measured: NACA 0012 wing, " << c.wing.mesh.triangles.size() << " panels: CL "
              << c.result.cl << " CD " << c.result.cd << "\n";
    CHECK(c.result.cl > 0.2);
    auto surface = surface_from_result(c.wing.mesh, c.result);
    REQUIRE(surface.has_value());
    REQUIRE(view.set_surface(surface.value()).has_value());
    view.set_forces(forces_from_result(c.result, c.conditions, {0.25, 0.0, 0.0}));

    AeroViewOptions options;
    options.forces.enabled = true;
    options.forces.length_per_coefficient = 4.0;
    const FrameInfo frame{kWidth, kHeight};
    CameraLens lens;
    lens.vertical_fov = 30.0 * kDegree;
    TurntableMode mode =
        frame_bounds(view.scene_bounds(), frame.aspect(), lens, 147.0 * kDegree, 30.0 * kDegree);
    mode.focus += glm::dvec3(0.15, 0.0, 0.22);
    mode.distance *= 0.74;
    const Camera camera = turntable_camera(mode, 30.0);

    const std::vector<Legend> legends = view.legends(options);
    REQUIRE(legends.size() == 1);
    CHECK(legends[0].title == "Cp");
    CHECK(legends[0].range.min == -legends[0].range.max);
    CHECK(legends[0].ticks.size() >= 5);

    std::optional<AeroDrawStats> stats;
    auto rendered = renderer.render(
        [&](const FrameInfo& f) {
            auto drawn = view.draw(camera, f, options);
            INFO((drawn ? std::string() : drawn.error()));
            REQUIRE(drawn.has_value());
            stats = drawn.value();
        },
        [&](const FrameInfo& f) { view.draw_overlay(f, options); });
    INFO((rendered ? std::string() : rendered.error()));
    REQUIRE(rendered.has_value());
    const ImageRgba8& image = rendered.value();
    REQUIRE(stats.has_value());
    CHECK(stats->surface_triangles == c.wing.mesh.triangles.size());
    CHECK(stats->force_arrows >= 1);

    // The suction peak on the upper surface near the leading edge is blue,
    // the stagnation line on the lower leading edge red-orange.
    const glm::dvec2 suction = project(camera, frame, {0.03, 0.4, 0.04});
    const Pixel s = pixel_at(image, suction.x, suction.y);
    INFO("suction pixel " << s.r << " " << s.g << " " << s.b << " at " << suction.x << " "
                          << suction.y);
    CHECK(s.b > s.r + 30.0);
    // The legend panel in the lower right corner is lighter than the backdrop.
    const Pixel legend = pixel_at(image, kWidth - 60.0, kHeight - 60.0);
    const Pixel corner = pixel_at(image, 10.0, 10.0);
    CHECK(legend.r + legend.g + legend.b > corner.r + corner.g + corner.b);

    check_golden(image, "naca0012_cp_aoa4");
}

TEST_CASE("Golden snapshots: cylinder vortex street at Re 150, three frames",
          "[render][aero][gl][golden]") {
    constexpr int kWidth = 1100;
    constexpr int kHeight = 440;
    OffscreenRenderer renderer = renderer_or_skip(kWidth, kHeight, aero_hdr_settings());
    auto created = AeroView::create();
    INFO((created ? std::string() : created.error()));
    REQUIRE(created.has_value());
    AeroView view = std::move(created).value();

    // D = 16 cells, inlet speed 0.05 (Ma 0.087), Re = U D / nu = 150, a
    // 20 D x 8 D channel with free-slip sides, the cylinder 4 D from the inlet.
    constexpr double kDiameter = 16.0;
    constexpr double kSpeed = 0.05;
    constexpr double kReynolds = 150.0;
    aero::lbm::Lbm2DConfig config;
    config.nx = static_cast<std::size_t>(20.0 * kDiameter);
    config.ny = static_cast<std::size_t>(8.0 * kDiameter);
    config.tau = aero::lbm::tau_of_viscosity(kSpeed * kDiameter / kReynolds);
    config.x_boundary = aero::lbm::XBoundary::inflow_outflow;
    config.outlet = aero::lbm::Outlet::convective;
    config.bottom = {aero::lbm::WallKind::slip, 0.0};
    config.top = {aero::lbm::WallKind::slip, 0.0};
    config.inlet_ux = kSpeed;
    config.smagorinsky = {true, 0.1};
    const double cx = 4.0 * kDiameter;
    const double cy = 0.5 * static_cast<double>(config.ny - 1) + 0.5;
    auto solver = aero::lbm::make_cpu_solver(
        config, aero::lbm::circle_mask(config.nx, config.ny, cx, cy, 0.5 * kDiameter));
    INFO((solver ? std::string() : solver.error()));
    REQUIRE(solver.has_value());
    // A transverse bump behind the cylinder starts the shedding early.
    const auto start = [&](std::size_t x, std::size_t y) {
        const double dx = static_cast<double>(x) - (cx + kDiameter);
        const double dy = static_cast<double>(y) - cy;
        return aero::lbm::d2q9::Moments{
            1.0, kSpeed, 0.15 * kSpeed * std::exp(-(dx * dx + dy * dy) / (kDiameter * kDiameter))};
    };
    REQUIRE(solver.value()->initialize(start).has_value());
    const auto started = std::chrono::steady_clock::now();
    REQUIRE(solver.value()->step(16000).has_value());

    // World units: D = 1 m, U = 1 m/s; the cylinder centre at the origin.
    LatticeMapping mapping;
    mapping.spacing = 1.0 / kDiameter;
    mapping.velocity_scale = 1.0 / kSpeed;
    mapping.reference_speed = kSpeed;
    mapping.origin = {-cx / kDiameter, -cy / kDiameter, 0.0};

    AeroSurface cylinder;
    cylinder.mesh = geom::cylinder_profile(0.5, 0.24).mesh({96, 0.05});
    for (geom::Vector3& v : cylinder.mesh.vertices) {
        v.z -= 0.12;
    }
    REQUIRE(view.set_surface(cylinder).has_value());

    AeroViewOptions options;
    SectionPlane plane;
    plane.axis = Axis::z;
    plane.position = 0.0;
    plane.quantity = FieldQuantity::vorticity;
    plane.scale = {ScaleMode::fixed, -2.5, 2.5};
    options.sections.push_back(plane);
    // Particle traces over the vorticity: released along the inlet and
    // carried through the street.
    options.particles.enabled = true;
    options.particles.count = 2200;
    options.particles.lifetime = 22.0;
    options.particles.trace_points = 6;
    options.particles.opacity = 0.34;
    options.particles.width_px = 1.0;
    options.particles.color = {0.80, 0.83, 0.88};
    options.streamlines.seeds = seed_line({-3.9, -3.9, 0.0}, {-3.9, 3.9, 0.0}, 150);

    TurntableMode mode;
    mode.focus = {6.0, 0.0, 0.0};
    mode.yaw = -90.0 * kDegree;
    mode.pitch = 89.9 * kDegree;
    const FrameInfo frame{kWidth, kHeight};
    const double fov = 16.0;
    // Fit the 20 D wide channel (plus a margin) horizontally.
    const double half_width = 10.4;
    mode.distance = half_width / (std::tan(0.5 * fov * kDegree) * frame.aspect());
    const Camera camera = turntable_camera(mode, fov);

    constexpr std::size_t kStepsPerFrame = 300; // about 1/6 of a shedding period
    for (int f = 0; f < 3; ++f) {
        if (f > 0) {
            REQUIRE(solver.value()->step(kStepsPerFrame).has_value());
        }
        auto macro = solver.value()->macroscopic();
        REQUIRE(macro.has_value());
        REQUIRE(view.set_flow(flow_field_from(macro.value(), mapping)).has_value());
        // Particle time per frame: the lattice time between frames in world
        // units (dt = spacing / velocity_scale per step), in small advances.
        const double frame_time =
            static_cast<double>(kStepsPerFrame) * mapping.spacing / mapping.velocity_scale;
        const int advances = f == 0 ? 120 : 4;
        for (int a = 0; a < advances; ++a) {
            REQUIRE(view.advance(frame_time / 4.0, options).has_value());
        }
        std::optional<AeroDrawStats> stats;
        auto rendered = renderer.render(
            [&](const FrameInfo& fi) {
                auto drawn = view.draw(camera, fi, options);
                INFO((drawn ? std::string() : drawn.error()));
                REQUIRE(drawn.has_value());
                stats = drawn.value();
            },
            [&](const FrameInfo& fi) { view.draw_overlay(fi, options); });
        INFO((rendered ? std::string() : rendered.error()));
        REQUIRE(rendered.has_value());
        REQUIRE(stats.has_value());
        CHECK(stats->section_planes == 1);
        CHECK(stats->particles == options.particles.count);
        // Vortices of both signs in the wake: blue and red regions behind the
        // cylinder, none upstream.
        const ImageRgba8& image = rendered.value();
        std::size_t blue = 0;
        std::size_t red = 0;
        for (double x = 2.0; x < 14.0; x += 0.1) {
            for (double y = -3.0; y < 3.0; y += 0.1) {
                const glm::dvec2 p = project(camera, frame, {x, y, 0.0});
                const Pixel q = pixel_at(image, p.x, p.y);
                blue += q.b > q.r + 30.0 ? 1U : 0U;
                red += q.r > q.b + 30.0 ? 1U : 0U;
            }
        }
        INFO("frame " << f << " blue " << blue << " red " << red);
        CHECK(blue > 200);
        CHECK(red > 200);
        check_golden(image, "cylinder_vortex_street_" + std::to_string(f));
    }
    std::cout << "measured: cylinder vortex street (16900 LBM steps, 3 frames) "
              << std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count()
              << " s\n";
}

TEST_CASE("Aero view draws 3D sections, streamlines and the Q iso-surface", "[render][aero][gl]") {
    constexpr int kWidth = 960;
    constexpr int kHeight = 540;
    OffscreenRenderer renderer = renderer_or_skip(kWidth, kHeight, aero_hdr_settings());
    auto created = AeroView::create();
    INFO((created ? std::string() : created.error()));
    REQUIRE(created.has_value());
    AeroView view = std::move(created).value();

    // An analytic wake: a uniform stream along +x with two counter-rotating
    // Lamb-Oseen vortices trailing from the tips of the NACA wing.
    const WingCase c = solve_wing();
    auto surface = surface_from_result(c.wing.mesh, c.result);
    REQUIRE(surface.has_value());
    REQUIRE(view.set_surface(surface.value()).has_value());
    view.set_forces(forces_from_result(c.result, c.conditions, {0.25, 0.0, 0.0}));
    FlowField field;
    GridGeometry& g = field.velocity.grid;
    g.dims = {72, 48, 24};
    g.spacing = 0.125;
    g.origin = {-1.0, -3.0, -1.5};
    field.velocity.values.reserve(g.node_count());
    for (std::size_t k = 0; k < g.dims[2]; ++k) {
        for (std::size_t j = 0; j < g.dims[1]; ++j) {
            for (std::size_t i = 0; i < g.dims[0]; ++i) {
                const glm::dvec3 p = g.node_position(i, j, k);
                glm::dvec3 u(1.0, 0.0, 0.0);
                for (const double side : {-1.0, 1.0}) {
                    const double dy = p.y - side * 1.9;
                    const double dz = p.z - 0.05 - 0.02 * p.x;
                    const double r2 = dy * dy + dz * dz + 1.0e-12;
                    const double core = 0.09;
                    const double ramp = 1.0 / (1.0 + std::exp(-6.0 * (p.x - 1.2)));
                    const double v = -side * 0.35 * ramp * (1.0 - std::exp(-r2 / core)) / r2;
                    u += glm::dvec3(0.0, -v * dz, v * dy);
                }
                field.velocity.values.push_back(u);
            }
        }
    }
    REQUIRE(view.set_flow(field).has_value());

    AeroViewOptions options;
    options.forces.enabled = true;
    options.forces.length_per_coefficient = 4.0;
    SectionPlane plane;
    plane.axis = Axis::x;
    plane.position = 6.5;
    plane.quantity = FieldQuantity::vorticity;
    plane.opacity = 0.9;
    options.sections.push_back(plane);
    options.iso.enabled = true;
    options.iso.level = 0.1;
    options.streamlines.enabled = true;
    options.streamlines.seeds = seed_line({-0.9, 1.5, 0.2}, {-0.9, 2.3, -0.1}, 10);
    for (const glm::dvec3& s : seed_line({-0.9, -2.3, -0.1}, {-0.9, -1.5, 0.2}, 10)) {
        options.streamlines.seeds.push_back(s);
    }
    options.streamlines.settings.max_points = 600;
    options.streamlines.settings.step = 0.3;

    const FrameInfo frame{kWidth, kHeight};
    CameraLens lens;
    lens.vertical_fov = 30.0 * kDegree;
    TurntableMode mode =
        frame_bounds(view.scene_bounds(), frame.aspect(), lens, 140.0 * kDegree, 28.0 * kDegree);
    mode.distance *= 0.8;
    const Camera camera = turntable_camera(mode, 30.0);
    std::optional<AeroDrawStats> stats;
    auto rendered = renderer.render(
        [&](const FrameInfo& f) {
            auto drawn = view.draw(camera, f, options);
            INFO((drawn ? std::string() : drawn.error()));
            REQUIRE(drawn.has_value());
            stats = drawn.value();
        },
        [&](const FrameInfo& f) { view.draw_overlay(f, options); });
    INFO((rendered ? std::string() : rendered.error()));
    REQUIRE(rendered.has_value());
    REQUIRE(write_png(output_path("wing_wake_3d"), rendered.value()).has_value());
    REQUIRE(stats.has_value());
    CHECK(stats->section_planes == 1);
    CHECK(stats->iso_triangles > 500);
    CHECK(stats->streamlines == 20);
    CHECK(stats->streamline_points > 2000);
    const std::vector<Legend> legends = view.legends(options);
    REQUIRE(legends.size() == 3); // Cp, omega, |u|
    CHECK(legends[1].colormap == Colormap::diverging_dark);
    CHECK(legends[2].colormap == Colormap::sequential);
}
