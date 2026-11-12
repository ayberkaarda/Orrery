#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>

#include "black_hole_test_support.h"
#include "orrery/core/schwarzschild_geodesic.h"
#include "orrery/render/black_hole.h"
#include "orrery/render/body_visuals.h"
#include "orrery/render/space_renderer.h"
#include "orrery/render/ui_context.h"
#include "orrery/render/ui_panels.h"

// Rendering of the Phase 5 black-hole pass (docs/rendering.md, "Black
// hole"): the GPU geodesics against the double-precision reference, the
// shadow size, golden snapshots of the Einstein ring and of the disk with its
// Doppler asymmetry, the environment cube map against the unlensed sky,
// occlusion of bodies, finiteness and the 1080p frame time. All tagged [gl].

using namespace orrery;
using namespace orrery::render;
using orrery::render::test::renderer_or_skip;
namespace geo = orrery::core::geodesic;
using Catch::Approx;

namespace {

geo::GeoVec geo_of(const glm::dvec3& v) {
    return {v.x, v.y, v.z};
}

glm::dvec3 glm_of(const geo::GeoVec& v) {
    return {v.x, v.y, v.z};
}

double angle_between(const glm::dvec3& a, const glm::dvec3& b) {
    return std::atan2(glm::length(glm::cross(a, b)), glm::dot(a, b));
}

// Camera position relative to the hole in units of M.
geo::GeoVec camera_in_mass_units(const LensingTarget& target) {
    return geo_of(-target.hole_render / target.mass_length);
}

// Image position (x right, y down, origin top-left) of a world point [m].
glm::dvec2 project(const Camera& camera, const FrameInfo& frame, const glm::dvec3& world) {
    const glm::dvec3 rel = camera.to_render_double(world);
    const glm::dquat& q = camera.orientation();
    const glm::dvec3 right = q * glm::dvec3(1.0, 0.0, 0.0);
    const glm::dvec3 up = q * glm::dvec3(0.0, 1.0, 0.0);
    const glm::dvec3 back = q * glm::dvec3(0.0, 0.0, 1.0);
    const double depth = -glm::dot(rel, back);
    const double tan_y = std::tan(0.5 * camera.lens().vertical_fov);
    const double ndc_x = glm::dot(rel, right) / (depth * tan_y * frame.aspect());
    const double ndc_y = glm::dot(rel, up) / (depth * tan_y);
    return {0.5 * (ndc_x + 1.0) * frame.width, 0.5 * (1.0 - ndc_y) * frame.height};
}

struct BoxStats {
    double luma = 0.0;
    double red = 0.0;
    double blue = 0.0;
};

BoxStats box_stats(const ImageRgba8& image, const glm::dvec2& centre, int half) {
    BoxStats s;
    int n = 0;
    for (int dy = -half; dy <= half; ++dy) {
        for (int dx = -half; dx <= half; ++dx) {
            const auto x = static_cast<std::uint32_t>(std::lround(centre.x) + dx);
            const auto y = static_cast<std::uint32_t>(std::lround(centre.y) + dy);
            const std::size_t o = image.offset(x, y);
            s.luma += test::luma(image, x, y);
            s.red += image.pixels[o];
            s.blue += image.pixels[o + 2];
            ++n;
        }
    }
    s.luma /= n;
    s.red /= n;
    s.blue /= n;
    return s;
}

std::size_t count_non_finite(const std::vector<float>& values) {
    return static_cast<std::size_t>(
        std::count_if(values.begin(), values.end(), [](float v) { return !std::isfinite(v); }));
}

// A bright point-like star exactly behind the hole as seen from the camera.
StarPoint star_behind(const glm::dvec3& direction) {
    StarPoint star;
    star.direction = glm::vec3(glm::normalize(direction));
    star.magnitude = -4.5F;
    star.temperature = 9500.0F;
    star.color = glm::vec3(blackbody_linear_srgb(9500.0));
    return star;
}

// Free camera at `position` [m] looking along `direction`, world +Z up (or
// +Y when looking along Z).
Camera looking_camera(const glm::dvec3& position, const glm::dvec3& direction, double fov_degrees) {
    Camera camera;
    CameraLens lens;
    lens.vertical_fov = fov_degrees * test::kPi / 180.0;
    REQUIRE(camera.set_lens(lens).has_value());
    const glm::dvec3 forward = glm::normalize(direction);
    const glm::dvec3 up =
        std::abs(forward.z) > 0.9 ? glm::dvec3(0.0, 1.0, 0.0) : glm::dvec3(0.0, 0.0, 1.0);
    FreeMode free;
    free.position = position;
    free.orientation = glm::quatLookAtRH(forward, up);
    camera.set_mode(free);
    camera.update(0.0, CameraInput{}, BodyFrameLookup{});
    return camera;
}

double hdr_luminance_sum(const std::vector<float>& rgba) {
    double sum = 0.0;
    for (std::size_t i = 0; i + 3 < rgba.size(); i += 4) {
        sum += 0.2126 * rgba[i] + 0.7152 * rgba[i + 1] + 0.0722 * rgba[i + 2];
    }
    return sum;
}

// Azimuth and elevation of a world direction.
std::pair<double, double> azimuth_elevation(const glm::dvec3& d) {
    const glm::dvec3 n = glm::normalize(d);
    return {std::atan2(n.y, n.x), std::asin(n.z)};
}

// Screen angle of the primary Einstein ring of a source exactly behind the
// hole, from the CPU geodesics: the ray whose escape direction points away
// from the camera along the camera-hole line.
double einstein_ring_angle(double radius) {
    const geo::GeoVec camera{radius, 0.0, 0.0};
    const geo::DiskModel no_disk{{0.0, 0.0, 1.0}, 1.0e9, 2.0e9};
    const auto bend = [&](double theta) {
        const geo::RayTrace trace =
            geo::trace_ray(camera, {-std::cos(theta), std::sin(theta), 0.0}, no_disk);
        REQUIRE(trace.fate == geo::RayFate::escaped);
        // Signed angle of the escape direction from -x, positive towards +y.
        return std::atan2(trace.escape_direction.y, -trace.escape_direction.x);
    };
    double lo = 1.2 * shadow_angular_radius(radius);
    double hi = 1.0;
    REQUIRE(bend(lo) < 0.0);
    REQUIRE(bend(hi) > 0.0);
    for (int i = 0; i < 60; ++i) {
        const double mid = 0.5 * (lo + hi);
        (bend(mid) < 0.0 ? lo : hi) = mid;
    }
    return 0.5 * (lo + hi);
}

} // namespace

TEST_CASE("GPU geodesics match the double-precision reference ray by ray",
          "[render][gl][black_hole]") {
    OffscreenRenderer renderer = renderer_or_skip(320, 180);
    StarfieldParams sky;
    sky.count = 1000;
    auto space = SpaceRenderer::create(sky);
    REQUIRE(space.has_value());
    const auto hole = test::lone_hole();
    const FrameInfo frame{320, 180};
    const Camera camera = test::hole_camera(40.0, 0.4, 0.12, 60.0);
    const SpaceView view{hole.get(), nullptr, &camera, RenderSettings{}};
    renderer.context().make_current();
    auto traced = space.value().trace_geometry(view, frame);
    REQUIRE(traced.has_value());
    const GeodesicTraceImage& image = traced.value();
    const TraceGrid grid = trace_grid(frame);
    REQUIRE(image.width == grid.width);
    REQUIRE(image.height == grid.height);
    const std::optional<LensingTarget> target =
        select_lensed_black_hole(*hole, camera, frame, RenderSettings{});
    REQUIRE(target.has_value());
    const geo::GeoVec origin = camera_in_mass_units(*target);
    const geo::DiskModel disk = lensing_disk();

    std::size_t compared = 0;
    std::size_t fate_mismatches = 0;
    std::size_t escaped = 0;
    std::size_t hits = 0;
    double worst_direction = 0.0;
    double worst_b = 0.0;
    double worst_radius = 0.0;
    double worst_g = 0.0;
    std::vector<double> direction_errors;
    for (int j = 0; j < image.height; j += 2) {
        for (int i = 0; i < image.width; i += 2) {
            const glm::dvec3 ray = pixel_ray(camera, frame, trace_sample_pixel(i, j));
            const geo::RayTrace cpu = geo::trace_ray(origin, geo_of(ray), disk);
            const std::size_t k =
                static_cast<std::size_t>(j) * static_cast<std::size_t>(image.width) +
                static_cast<std::size_t>(i);
            const glm::vec4 gpu_disk = image.disk[k];
            const glm::vec4 gpu_sky = image.sky[k];
            // Close to the critical curve the ray map is chaotic: a relative
            // change of 1e-2 in b changes the winding; skip that band.
            if (std::abs(cpu.impact_parameter / geo::kCriticalImpactParameter - 1.0) < 0.01) {
                continue;
            }
            ++compared;
            worst_b = std::max(
                worst_b, std::abs(static_cast<double>(gpu_sky.w) / cpu.impact_parameter - 1.0));
            const auto gpu_fate = static_cast<int>(std::lround(gpu_disk.w));
            if (gpu_fate != static_cast<int>(cpu.fate)) {
                ++fate_mismatches;
                continue;
            }
            if (cpu.fate == geo::RayFate::escaped) {
                ++escaped;
                const double error =
                    angle_between(glm::dvec3(gpu_sky), glm_of(cpu.escape_direction));
                direction_errors.push_back(error);
                worst_direction = std::max(worst_direction, error);
            }
            const auto first = std::find_if(cpu.crossings.begin(), cpu.crossings.end(),
                                            [](const geo::PlaneCrossing& c) { return c.on_disk; });
            const auto on_disk = static_cast<int>(
                std::count_if(cpu.crossings.begin(), cpu.crossings.end(),
                              [](const geo::PlaneCrossing& c) { return c.on_disk; }));
            CHECK(static_cast<int>(std::lround(gpu_disk.z)) == on_disk);
            if (first != cpu.crossings.end()) {
                ++hits;
                worst_radius = std::max(
                    worst_radius, std::abs(static_cast<double>(gpu_disk.x) / first->radius - 1.0));
                worst_g = std::max(worst_g, std::abs(static_cast<double>(gpu_disk.y) - first->g));
            }
        }
    }
    std::sort(direction_errors.begin(), direction_errors.end());
    const double median =
        direction_errors.empty() ? 0.0 : direction_errors[direction_errors.size() / 2];
    WARN("GPU vs CPU over " << compared << " rays (" << escaped << " escaped, " << hits
                            << " on the disk): fate mismatches " << fate_mismatches
                            << ", escape direction median " << median << " rad, worst "
                            << worst_direction << " rad; impact parameter " << worst_b
                            << " relative; first hit radius " << worst_radius << " relative, g "
                            << worst_g);
    CHECK(compared > 3000);
    CHECK(escaped > 1000);
    CHECK(hits > 300);
    CHECK(fate_mismatches <= compared / 500);
    CHECK(median < 2.0e-5);
    CHECK(worst_direction < 2.0e-3); // a fifth of a trace sample at this field of view
    CHECK(worst_b < 1.0e-5);
    CHECK(worst_radius < 1.0e-3);
    CHECK(worst_g < 1.0e-3);
}

TEST_CASE("The shadow has the projected radius of 3 sqrt(3) M", "[render][gl][black_hole]") {
    const FrameInfo frame{400, 400};
    OffscreenRenderer renderer = renderer_or_skip(frame.width, frame.height);
    auto space = SpaceRenderer::create();
    REQUIRE(space.has_value());
    const auto hole = test::lone_hole();
    for (const double radius : {12.0, 30.0, 80.0}) {
        INFO("camera at " << radius << " M");
        const Camera camera = test::hole_camera(radius, 1.1, 0.4, 70.0);
        const SpaceView view{hole.get(), nullptr, &camera, RenderSettings{}};
        renderer.context().make_current();
        auto traced = space.value().trace_geometry(view, frame);
        REQUIRE(traced.has_value());
        std::size_t captured = 0;
        for (const glm::vec4& d : traced.value().disk) {
            captured += std::lround(d.w) == 1 ? 1U : 0U;
        }
        // Each trace sample stands for 2 x 2 pixels.
        const double measured = std::sqrt(4.0 * static_cast<double>(captured) / test::kPi);
        const double expected =
            std::tan(shadow_angular_radius(radius)) * pixel_scale(camera.lens().vertical_fov, 400);
        WARN("shadow radius at " << radius << " M: " << measured << " px, expected " << expected
                                 << " px");
        CHECK(measured == Approx(expected).margin(1.0 + 0.01 * expected));
    }
}

TEST_CASE("Golden snapshot: Einstein ring and shadow of a star behind the hole",
          "[render][gl][golden][black_hole]") {
    const FrameInfo frame{640, 360};
    HdrSettings hdr;
    hdr.samples = 4;
    OffscreenRenderer renderer = renderer_or_skip(frame.width, frame.height, hdr);
    // The camera looks at the hole along the galactic plane, so that the
    // Milky Way runs across the view and, lensed, around the shadow; the star
    // sits exactly behind the hole.
    const glm::dvec3 behind = glm::transpose(ecliptic_to_galactic()) * glm::dvec3(0.0, 1.0, 0.0);
    StarfieldParams with_star;
    with_star.count = 0;
    with_star.extra_stars.push_back(star_behind(behind));
    StarfieldParams without_star;
    without_star.count = 0;
    auto space = SpaceRenderer::create(with_star);
    REQUIRE(space.has_value());
    auto empty_space = SpaceRenderer::create(without_star);
    REQUIRE(empty_space.has_value());
    const auto hole = test::lone_hole();
    const auto [azimuth, elevation] = azimuth_elevation(-behind);
    const double radius = 60.0;
    const Camera camera = test::hole_camera(radius, azimuth, elevation, 40.0);
    RenderSettings settings;
    settings.accretion_disk = false;
    settings.milky_way_brightness = 3.0;

    SpaceDrawStats stats;
    auto ring = renderer.render([&](const FrameInfo& f) {
        stats = space.value().draw(SpaceView{hole.get(), nullptr, &camera, settings}, f);
    });
    REQUIRE(ring.has_value());
    CHECK(stats.lensing);
    CHECK(stats.bodies_drawn == 0);
    CHECK(count_non_finite(renderer.pipeline().read_hdr()) == 0);
    CHECK(renderer.context().debug_error_count() == 0);
    auto background = renderer.render([&](const FrameInfo& f) {
        (void)empty_space.value().draw(SpaceView{hole.get(), nullptr, &camera, settings}, f);
    });
    REQUIRE(background.has_value());
    const ImageRgba8& a = ring.value();
    const ImageRgba8& b = background.value();

    // The star's light alone: a closed ring at the radius the CPU geodesics
    // predict for a source exactly behind the hole.
    const double pps = pixel_scale(camera.lens().vertical_fov, frame.height);
    const double ring_px = std::tan(einstein_ring_angle(radius)) * pps;
    const double shadow_px = std::tan(shadow_angular_radius(radius)) * pps;
    const glm::dvec2 centre{0.5 * frame.width, 0.5 * frame.height};
    double weight = 0.0;
    double weighted_radius = 0.0;
    std::array<double, 8> sectors{};
    for (std::uint32_t y = 0; y < a.height; ++y) {
        for (std::uint32_t x = 0; x < a.width; ++x) {
            const double diff = test::luma(a, x, y) - test::luma(b, x, y);
            const glm::dvec2 offset = glm::dvec2(x + 0.5, y + 0.5) - centre;
            const double r = glm::length(offset);
            if (diff > 8.0 && r > 1.3 * shadow_px) {
                weight += diff;
                weighted_radius += diff * r;
                const double angle = std::atan2(offset.y, offset.x) + test::kPi;
                sectors[std::min<std::size_t>(
                    7, static_cast<std::size_t>(angle / (test::kPi / 4.0)))] += diff;
            }
        }
    }
    REQUIRE(weight > 0.0);
    const double measured_ring = weighted_radius / weight;
    WARN("Einstein ring radius " << measured_ring << " px (geodesic prediction " << ring_px
                                 << " px), shadow radius " << shadow_px << " px");
    CHECK(measured_ring == Approx(ring_px).margin(2.0));
    for (const double sector : sectors) {
        CHECK(sector > 0.05 * weight / 8.0); // closed all around
    }

    // The shadow is black against the lensed Milky Way around it.
    double inside = 0.0;
    double around = 0.0;
    std::size_t n_inside = 0;
    std::size_t n_around = 0;
    for (std::uint32_t y = 0; y < b.height; ++y) {
        for (std::uint32_t x = 0; x < b.width; ++x) {
            const double r = glm::length(glm::dvec2(x + 0.5, y + 0.5) - centre);
            if (r < 0.85 * shadow_px) {
                inside += test::luma(b, x, y);
                ++n_inside;
            } else if (r > 1.15 * shadow_px && r < 2.0 * shadow_px) {
                around += test::luma(b, x, y);
                ++n_around;
            }
        }
    }
    inside /= static_cast<double>(n_inside);
    around /= static_cast<double>(n_around);
    CHECK(inside < 1.0);
    CHECK(around > 4.0);
    test::check_black_hole_golden(a, "black_hole_einstein_ring");
}

TEST_CASE("Golden snapshot: thin disk with Doppler beaming and gravitational redshift",
          "[render][gl][golden][black_hole]") {
    const FrameInfo frame{640, 360};
    HdrSettings hdr;
    hdr.samples = 4;
    OffscreenRenderer renderer = renderer_or_skip(frame.width, frame.height, hdr);
    auto space = SpaceRenderer::create();
    REQUIRE(space.has_value());
    const auto hole = test::lone_hole();
    const double azimuth = 72.0 * test::kPi / 180.0;
    const double elevation = 9.0 * test::kPi / 180.0;
    const Camera camera = test::hole_camera(80.0, azimuth, elevation, 40.0);
    const RenderSettings settings;
    SpaceDrawStats stats;
    auto image = renderer.render([&](const FrameInfo& f) {
        stats = space.value().draw(SpaceView{hole.get(), nullptr, &camera, settings}, f);
    });
    REQUIRE(image.has_value());
    CHECK(stats.lensing);
    CHECK(count_non_finite(renderer.pipeline().read_hdr()) == 0);
    CHECK(renderer.context().debug_error_count() == 0);

    // The gas orbits counter-clockwise about +z: at azimuth phi it moves
    // along (-sin phi, cos phi, 0). The side moving towards the camera is
    // brighter (Doppler beaming) and bluer (higher observed temperature).
    const double m = test::hole_mass_length();
    const glm::dvec3 to_camera{std::cos(azimuth), std::sin(azimuth), 0.0};
    BoxStats approaching;
    BoxStats receding;
    for (const double r : {12.0, 18.0}) {
        for (const double side : {-1.0, 1.0}) {
            const double phi = azimuth + side * 0.5 * test::kPi;
            const glm::dvec3 velocity{-std::sin(phi), std::cos(phi), 0.0};
            const glm::dvec3 point = r * m * glm::dvec3(std::cos(phi), std::sin(phi), 0.0);
            const BoxStats s = box_stats(image.value(), project(camera, frame, point), 5);
            BoxStats& into = glm::dot(velocity, to_camera) > 0.0 ? approaching : receding;
            into.luma += s.luma;
            into.red += s.red;
            into.blue += s.blue;
        }
    }
    WARN("disk luma approaching " << approaching.luma / 2.0 << ", receding " << receding.luma / 2.0
                                  << "; blue/red approaching " << approaching.blue / approaching.red
                                  << ", receding " << receding.blue / receding.red);
    CHECK(approaching.luma > 2.0 * receding.luma);
    CHECK(receding.luma > 10.0); // the receding side stays visible, dimmer and redder
    CHECK(approaching.blue / approaching.red > receding.blue / receding.red + 0.1);
    test::check_black_hole_golden(image.value(), "black_hole_disk_doppler");
}

TEST_CASE("Through the environment cube map a weak lens shows the unlensed sky",
          "[render][gl][black_hole]") {
    // At 1440 x 810 over 40 degrees a pixel subtends about the reference
    // pixel angle of the cube map's stars (see the composite shader), so the
    // two skies must agree in brightness as well as in position.
    const FrameInfo frame{1440, 810};
    OffscreenRenderer renderer = renderer_or_skip(frame.width, frame.height);
    auto space = SpaceRenderer::create();
    REQUIRE(space.has_value());
    const auto hole = test::lone_hole();
    // The hole far behind the camera: selected (Einstein angle 1.1 px) but
    // bending the visible sky by less than 0.03 px.
    const glm::dvec3 gc = test::galactic_centre_direction();
    const auto [azimuth, elevation] = azimuth_elevation(gc);
    Camera camera = test::hole_camera(4.0e6, azimuth, elevation, 40.0);
    // Turn the view around: look away from the hole, towards the galactic centre.
    FreeMode free;
    free.position = 4.0e6 * test::hole_mass_length() * gc;
    free.orientation = camera.orientation() * glm::angleAxis(test::kPi, glm::dvec3(0.0, 1.0, 0.0));
    camera.set_mode(free);
    camera.update(0.0, CameraInput{}, BodyFrameLookup{});

    const auto render = [&](RenderSettings settings, bool lensing, bool* lensed) {
        settings.black_hole_lensing = lensing;
        auto image = renderer.render([&](const FrameInfo& f) {
            const SpaceDrawStats stats =
                space.value().draw(SpaceView{hole.get(), nullptr, &camera, settings}, f);
            *lensed = stats.lensing;
        });
        REQUIRE(image.has_value());
        return std::move(image).value();
    };
    bool lensed = false;
    RenderSettings band;
    band.star_brightness = 0.0;
    const ImageRgba8 band_lensed = render(band, true, &lensed);
    REQUIRE(lensed);
    const ImageRgba8 band_plain = render(band, false, &lensed);
    REQUIRE_FALSE(lensed);
    auto score = ssim(band_plain, band_lensed);
    REQUIRE(score.has_value());
    double sum_lensed = 0.0;
    double sum_plain = 0.0;
    for (std::uint32_t y = 0; y < band_plain.height; ++y) {
        for (std::uint32_t x = 0; x < band_plain.width; ++x) {
            sum_lensed += test::luma(band_lensed, x, y);
            sum_plain += test::luma(band_plain, x, y);
        }
    }
    WARN("Milky Way through the cube map: SSIM " << score.value().value << ", mean luma "
                                                 << sum_lensed / (1440.0 * 810.0) << " vs "
                                                 << sum_plain / (1440.0 * 810.0));
    CHECK(score.value().value > 0.97);
    CHECK(sum_lensed / sum_plain == Approx(1.0).margin(0.04));

    RenderSettings stars;
    stars.milky_way_brightness = 0.0;
    const ImageRgba8 stars_lensed = render(stars, true, &lensed);
    const ImageRgba8 stars_plain = render(stars, false, &lensed);
    double flux_lensed = 0.0;
    double flux_plain = 0.0;
    std::uint32_t brightest_lensed = 0;
    std::uint32_t brightest_plain = 0;
    const std::uint32_t w = stars_plain.width;
    for (std::uint32_t i = 0; i < w * stars_plain.height; ++i) {
        const double l = test::luma(stars_lensed, i % w, i / w);
        const double p = test::luma(stars_plain, i % w, i / w);
        flux_lensed += l;
        flux_plain += p;
        if (l > test::luma(stars_lensed, brightest_lensed % w, brightest_lensed / w)) {
            brightest_lensed = i;
        }
        if (p > test::luma(stars_plain, brightest_plain % w, brightest_plain / w)) {
            brightest_plain = i;
        }
    }
    WARN("starfield through the cube map: summed luma " << flux_lensed << " vs " << flux_plain);
    CHECK(flux_lensed / flux_plain == Approx(1.0).margin(0.3));
    const std::uint32_t lensed_row = brightest_lensed / w;
    const std::uint32_t plain_row = brightest_plain / w;
    const std::uint32_t lensed_column = brightest_lensed % w;
    const std::uint32_t plain_column = brightest_plain % w;
    const double dx = static_cast<double>(lensed_column) - static_cast<double>(plain_column);
    const double dy = static_cast<double>(lensed_row) - static_cast<double>(plain_row);
    CHECK(std::hypot(dx, dy) < 2.0);
}

TEST_CASE("Shadow and disk hide bodies behind the hole but not in front",
          "[render][gl][black_hole]") {
    const FrameInfo frame{400, 300};
    OffscreenRenderer renderer = renderer_or_skip(frame.width, frame.height);
    auto space = SpaceRenderer::create();
    REQUIRE(space.has_value());
    const double m = test::hole_mass_length();
    const double radius = 60.0;
    const double azimuth = 0.0;
    const double elevation = 0.25;
    const glm::dvec3 to_camera{std::cos(elevation) * std::cos(azimuth),
                               std::cos(elevation) * std::sin(azimuth), std::sin(elevation)};
    const glm::dvec3 side = glm::normalize(glm::cross(to_camera, glm::dvec3(0.0, 0.0, 1.0)));
    // Behind the hole on the line of sight, and in front of it off to one side.
    const glm::dvec3 behind = -40.0 * m * to_camera;
    const glm::dvec3 front = 25.0 * m * to_camera + 12.0 * m * side;
    const auto au = core::constants::astronomical_unit.value();
    core::BodyStore bodies;
    REQUIRE(
        bodies.add({"Hole", core::BlackHole{0.0}, core::space::Mass(test::kHoleMassMsun), {}, {}}));
    const auto planet = [&](const char* name, const glm::dvec3& p) {
        REQUIRE(bodies.add({name,
                            core::Planet{},
                            core::space::Mass(1.0e-6),
                            {core::space::Length(p.x / au), core::space::Length(p.y / au),
                             core::space::Length(p.z / au)},
                            {}}));
    };
    planet("Far", behind);
    planet("Near", front);
    REQUIRE(bodies.set_radius(1, core::space::Length(2.5 * m / au)));
    REQUIRE(bodies.set_radius(2, core::space::Length(2.5 * m / au)));
    const auto snapshot =
        core::SimSnapshot::capture(core::SimState(core::space::Time(0.0), std::move(bodies)));
    const Camera camera = test::hole_camera(radius, azimuth, elevation, 40.0);
    RenderSettings settings;
    settings.show_starfield = false;
    SpaceDrawStats stats;
    auto image = renderer.render([&](const FrameInfo& f) {
        stats = space.value().draw(SpaceView{snapshot.get(), nullptr, &camera, settings}, f);
    });
    REQUIRE(image.has_value());
    CHECK(stats.lensing);
    CHECK(stats.bodies_drawn == 2);
    const glm::dvec2 far_px = project(camera, frame, behind);
    const glm::dvec2 near_px = project(camera, frame, front);
    CHECK(box_stats(image.value(), far_px, 2).luma < 2.0);
    CHECK(box_stats(image.value(), near_px, 2).luma > 20.0);
    CHECK(renderer.context().debug_error_count() == 0);
}

// Review fix: a static camera just outside the horizon sees the disk
// blueshifted by g > 10 (the observer's own shift 1 / sqrt(1 - 2 / r) is
// about 14 at r = 2.01 M). With a 2000 K disk the black-body radiance at
// g T is some 10^5 times the radiance at T, beyond the largest half float:
// the HDR target must still hold finite values.
TEST_CASE("A camera just outside the horizon renders a finite HDR image",
          "[render][gl][black_hole]") {
    const FrameInfo frame{320, 320};
    OffscreenRenderer renderer = renderer_or_skip(frame.width, frame.height);
    auto space = SpaceRenderer::create();
    REQUIRE(space.has_value());
    const auto hole = test::lone_hole();
    // On the disk axis, looking outwards: the sky and the disk lie inside
    // the escape cone of about 0.18 rad around +Z.
    const Camera camera = looking_camera(glm::dvec3(0.0, 0.0, 2.01 * test::hole_mass_length()),
                                         glm::dvec3(0.0, 0.0, 1.0), 40.0);
    RenderSettings settings;
    settings.disk_temperature = 2000.0;
    const SpaceView view{hole.get(), nullptr, &camera, settings};
    renderer.context().make_current();
    auto traced = space.value().trace_geometry(view, frame);
    REQUIRE(traced.has_value());
    double max_g = 0.0;
    std::size_t on_disk = 0;
    for (const glm::vec4& d : traced.value().disk) {
        if (d.z > 0.5F) {
            ++on_disk;
            max_g = std::max(max_g, static_cast<double>(d.y));
        }
    }
    WARN("near-horizon camera: " << on_disk << " samples on the disk, largest g " << max_g);
    REQUIRE(on_disk > 50);
    REQUIRE(max_g > 10.0);

    SpaceDrawStats stats;
    auto image = renderer.render([&](const FrameInfo& f) {
        stats = space.value().draw(SpaceView{hole.get(), nullptr, &camera, settings}, f);
    });
    REQUIRE(image.has_value());
    CHECK(stats.lensing);
    const std::vector<float> hdr = renderer.pipeline().read_hdr();
    CHECK(count_non_finite(hdr) == 0);
    // No value reaches the largest half float either (some drivers saturate
    // an out-of-range write there instead of storing infinity), while the
    // disk is still far above the tone-mapping shoulder.
    const float brightest = *std::max_element(hdr.begin(), hdr.end());
    WARN("near-horizon camera: brightest HDR value " << brightest);
    CHECK(brightest < 65504.0F);
    CHECK(brightest > 100.0F);
    std::size_t lit = 0;
    for (std::uint32_t y = 0; y < image.value().height; ++y) {
        for (std::uint32_t x = 0; x < image.value().width; ++x) {
            lit += test::luma(image.value(), x, y) > 200.0 ? 1U : 0U;
        }
    }
    CHECK(lit > 100);
    CHECK(renderer.context().debug_error_count() == 0);
}

// Review fix: a star whose sprite straddles an edge or a corner of the
// environment cube map keeps its flux. Each star is looked at through a
// weak lens (as in the cube map test above) and its summed HDR luminance
// compared with that of the same star at the centre of a face.
TEST_CASE("Stars keep their flux across cube map edges and corners", "[render][gl][black_hole]") {
    const FrameInfo frame{320, 320};
    OffscreenRenderer renderer = renderer_or_skip(frame.width, frame.height);
    const auto hole = test::lone_hole();
    const double m = test::hole_mass_length();
    RenderSettings settings;
    settings.milky_way_brightness = 0.0;
    // One face texel at a face edge is (2 / N) / 2^(3/4) = 5.8e-4 rad; the
    // seam stars sit a quarter of a texel from the edge (or corner), their
    // sprites spread over several texels on both sides.
    const double in = 1.0 - 0.25 * 2.0 / 2048.0;
    struct Probe {
        const char* name;
        glm::dvec3 direction;
    };
    const std::array<Probe, 5> probes{{{"face centre", {1.0, 0.0, 0.0}},
                                       {"edge +X/+Y", {1.0, in, 0.3}},
                                       {"edge +Y/+Z", {-0.2, in, 1.0}},
                                       {"corner +X+Y+Z", {1.0, in, in}},
                                       {"corner -X-Y+Z", {-1.0, -in, in}}}};
    double reference = 0.0;
    for (const Probe& probe : probes) {
        INFO(probe.name);
        const glm::dvec3 direction = glm::normalize(probe.direction);
        StarfieldParams params;
        params.count = 0;
        StarPoint star = star_behind(direction);
        star.magnitude = 0.0F;
        params.extra_stars.push_back(star);
        auto space = SpaceRenderer::create(params);
        REQUIRE(space.has_value());
        // The hole far behind the camera bends the view by under 0.01 px.
        const Camera camera = looking_camera(4.0e6 * m * direction, direction, 10.0);
        bool lensed = false;
        auto image = renderer.render([&](const FrameInfo& f) {
            lensed =
                space.value().draw(SpaceView{hole.get(), nullptr, &camera, settings}, f).lensing;
        });
        REQUIRE(image.has_value());
        REQUIRE(lensed);
        const double flux = hdr_luminance_sum(renderer.pipeline().read_hdr());
        if (reference == 0.0) {
            reference = flux;
            REQUIRE(reference > 0.0);
            continue;
        }
        WARN(probe.name << ": star flux " << flux / reference << " of the face-centred star");
        CHECK(flux / reference == Approx(1.0).margin(0.1));
    }
    CHECK(renderer.context().debug_error_count() == 0);
}

TEST_CASE("Black hole with disk renders a 1080p frame at 30 fps or more",
          "[render][gl][perf][black_hole]") {
    HdrSettings hdr;
    hdr.samples = 4;
    OffscreenRenderer renderer = renderer_or_skip(1920, 1080, hdr);
    auto space = SpaceRenderer::create();
    REQUIRE(space.has_value());
    const auto hole = test::lone_hole();
    const Camera camera = test::hole_camera(80.0, 1.25, 0.16, 40.0);
    const SpaceView view{hole.get(), nullptr, &camera, RenderSettings{}};
    renderer.context().make_current();
    HdrPipeline& pipeline = renderer.pipeline();
    const GpuTimer timer = GpuTimer::create();
    std::vector<double> gpu_ms;
    for (int frame = 0; frame < 65; ++frame) {
        timer.begin();
        pipeline.begin_scene();
        const SpaceDrawStats stats = space.value().draw(view, pipeline.frame());
        pipeline.resolve();
        timer.end();
        REQUIRE(stats.lensing);
        const double ms = static_cast<double>(timer.elapsed_ns()) * 1.0e-6;
        if (frame >= 5) { // warm-up: environment cube map, shader caches
            gpu_ms.push_back(ms);
        }
    }
    std::sort(gpu_ms.begin(), gpu_ms.end());
    const double median = gpu_ms[gpu_ms.size() / 2];
    const double p95 = gpu_ms[gpu_ms.size() * 95 / 100];
    WARN("black hole, 1920x1080, 4x MSAA, 512 steps, disk on, on "
         << renderer.context().info().renderer << ": GPU median " << median << " ms ("
         << 1000.0 / median << " fps), p95 " << p95 << " ms");
    CHECK(p95 < 1000.0 / 30.0);
    CHECK(renderer.context().debug_error_count() == 0);
}

TEST_CASE("Render Settings offer the black-hole controls when the scene holds a hole",
          "[render][gl][ui][black_hole]") {
    constexpr int kWidth = 1600;
    constexpr int kHeight = 900;
    HdrSettings hdr;
    hdr.samples = 4;
    OffscreenRenderer renderer = renderer_or_skip(kWidth, kHeight, hdr);
    auto space = SpaceRenderer::create();
    REQUIRE(space.has_value());
    auto created = UiContext::create(renderer.context());
    REQUIRE(created.has_value());
    UiContext ui = std::move(created).value();
    const auto hole = test::lone_hole();
    const Camera camera =
        test::hole_camera(51.0, 72.0 * test::kPi / 180.0, 12.0 * test::kPi / 180.0, 40.0);
    const RenderSettings settings;
    const auto draw_scene = [&](const FrameInfo& f) {
        (void)space.value().draw(SpaceView{hole.get(), nullptr, &camera, settings}, f);
    };
    const auto render_with = [&](bool black_hole) {
        const auto draw_ui = [&](const FrameInfo& f) {
            UiFrameInput input;
            input.width = f.width;
            input.height = f.height;
            ui.begin_frame(input);
            (void)ui.dockspace();
            (void)draw_body_inspector(make_body_inspector_data(*hole, hole->ids()[0]));
            RenderSettingsContext context;
            context.black_hole = black_hole;
            (void)draw_render_settings(settings, context);
            ui.end_frame();
        };
        // The first frames settle the dock layout.
        for (int i = 0; i < 2; ++i) {
            REQUIRE(renderer.render(draw_scene, draw_ui).has_value());
        }
        auto image = renderer.render(draw_scene, draw_ui);
        REQUIRE(image.has_value());
        return std::move(image).value();
    };
    const ImageRgba8 plain = render_with(false);
    const ImageRgba8 with_section = render_with(true);
    const std::optional<UiRect> panel = ui.window_rect(kRenderSettingsWindow);
    REQUIRE(panel.has_value());
    CHECK(ui.window_docked(kRenderSettingsWindow));
    // The section adds five labelled rows of text and controls.
    const auto text_pixels = [&](const ImageRgba8& image) {
        std::size_t n = 0;
        for (auto y = static_cast<std::uint32_t>(panel->y);
             y < static_cast<std::uint32_t>(panel->y + panel->height) && y < image.height; ++y) {
            for (auto x = static_cast<std::uint32_t>(panel->x);
                 x < static_cast<std::uint32_t>(panel->x + panel->width) && x < image.width; ++x) {
                n += test::luma(image, x, y) > 150.0 ? 1U : 0U;
            }
        }
        return n;
    };
    CHECK(text_pixels(with_section) > text_pixels(plain) + 400);
    const std::filesystem::path output_dir = ORRERY_RENDER_OUTPUT_DIR;
    std::filesystem::create_directories(output_dir);
    REQUIRE(write_png(output_dir / "ui_black_hole_workspace_actual.png", with_section).has_value());
    CHECK(renderer.context().debug_error_count() == 0);
}

// Review renders (hidden: not part of the test run). Writes
// bh_gallery_<name>.png under the render output directory:
//   orrery_test_render "[black_hole_gallery]"
TEST_CASE("Black hole review renders", "[.][black_hole_gallery]") {
    HdrSettings hdr;
    hdr.samples = 4;
    OffscreenRenderer renderer = renderer_or_skip(1920, 1080, hdr);
    auto space = SpaceRenderer::create();
    REQUIRE(space.has_value());
    const auto hole = test::lone_hole();
    struct Shot {
        const char* name;
        double radius;
        double azimuth_deg;
        double elevation_deg;
        double temperature;
        double brightness;
    };
    const std::array<Shot, 10> shots{{{"framing", 51.0, 72.0, 12.0, 0.0, 0.0},
                                      {"close", 30.0, 40.0, 6.0, 0.0, 0.0},
                                      {"edge_on", 60.0, 72.0, 1.5, 0.0, 0.0},
                                      {"high", 70.0, 72.0, 55.0, 0.0, 0.0},
                                      {"far", 400.0, 72.0, 12.0, 0.0, 0.0},
                                      {"distant", 1.2e5, 20.0, -5.0, 0.0, 0.0},
                                      {"t4000", 60.0, 72.0, 1.5, 4000.0, 1.0},
                                      {"t4500b2", 60.0, 72.0, 1.5, 4500.0, 2.0},
                                      {"t5000", 60.0, 72.0, 1.5, 5000.0, 1.0},
                                      {"t5000b05", 60.0, 72.0, 1.5, 5000.0, 0.5}}};
    const std::filesystem::path output_dir = ORRERY_RENDER_OUTPUT_DIR;
    std::filesystem::create_directories(output_dir);
    for (const Shot& shot : shots) {
        const Camera camera = test::hole_camera(shot.radius, shot.azimuth_deg * test::kPi / 180.0,
                                                shot.elevation_deg * test::kPi / 180.0, 40.0);
        RenderSettings settings;
        if (shot.temperature > 0.0) {
            settings.disk_temperature = shot.temperature;
            settings.disk_brightness = shot.brightness;
        }
        auto image = renderer.render([&](const FrameInfo& f) {
            (void)space.value().draw(SpaceView{hole.get(), nullptr, &camera, settings}, f);
        });
        REQUIRE(image.has_value());
        REQUIRE(
            write_png(output_dir / (std::string("bh_gallery_") + shot.name + ".png"), image.value())
                .has_value());
    }
}
