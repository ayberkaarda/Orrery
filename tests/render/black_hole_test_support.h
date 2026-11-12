#pragma once

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>

#include <glm/geometric.hpp>

#include "gl_test_support.h"
#include "orrery/core/black_hole.h"
#include "orrery/core/body_store.h"
#include "orrery/core/sim_snapshot.h"
#include "orrery/core/units.h"
#include "orrery/render/camera.h"
#include "orrery/render/png.h"
#include "orrery/render/ssim.h"
#include "orrery/render/starfield.h"

// Shared scenes of the Phase 5 black-hole tests: a lone Schwarzschild hole at
// the world origin and cameras placed in units of its gravitational radius
// M = G M / c^2.
namespace orrery::render::test {

inline constexpr double kHoleMassMsun = 4.3e6;
inline constexpr double kPi = 3.141592653589793;

// G M / c^2 of the test hole [m].
inline double hole_mass_length() {
    return 0.5 * core::to_si(core::schwarzschild_radius(core::space::Mass(kHoleMassMsun))).value();
}

inline std::shared_ptr<const core::SimSnapshot> lone_hole() {
    core::BodyStore bodies;
    REQUIRE(bodies.add({"Hole", core::BlackHole{0.0}, core::space::Mass(kHoleMassMsun), {}, {}}));
    return core::SimSnapshot::capture(core::SimState(core::space::Time(0.0), std::move(bodies)));
}

// Camera at `radius` M from the hole, in the direction (azimuth, elevation)
// from it, looking at the hole.
inline Camera hole_camera(double radius, double azimuth, double elevation, double fov_degrees) {
    Camera camera;
    CameraLens lens;
    lens.vertical_fov = fov_degrees * kPi / 180.0;
    REQUIRE(camera.set_lens(lens).has_value());
    OrbitMode orbit;
    orbit.focus = glm::dvec3(0.0);
    orbit.distance = radius * hole_mass_length();
    orbit.yaw = azimuth;
    orbit.pitch = elevation;
    camera.set_mode(orbit);
    camera.update(0.0, CameraInput{}, BodyFrameLookup{});
    return camera;
}

// Direction of the galactic centre in world (ecliptic) axes.
inline glm::dvec3 galactic_centre_direction() {
    return glm::transpose(ecliptic_to_galactic()) * glm::dvec3(1.0, 0.0, 0.0);
}

inline double luma(const ImageRgba8& image, std::uint32_t x, std::uint32_t y) {
    const std::size_t o = image.offset(x, y);
    return 0.2126 * image.pixels[o] + 0.7152 * image.pixels[o + 1] + 0.0722 * image.pixels[o + 2];
}

// Writes the actual image under render_output, then compares it with the
// golden reference (SSIM > 0.98) or, with ORRERY_UPDATE_GOLDEN=1, writes it.
inline void check_black_hole_golden(const ImageRgba8& image, const std::string& name) {
    const std::filesystem::path output_dir = ORRERY_RENDER_OUTPUT_DIR;
    std::filesystem::create_directories(output_dir);
    const std::filesystem::path actual_path = output_dir / (name + "_actual.png");
    REQUIRE(write_png(actual_path, image).has_value());
    const std::filesystem::path golden_path =
        std::filesystem::path(ORRERY_RENDER_GOLDEN_DIR) / (name + ".png");
    if (environment_flag("ORRERY_UPDATE_GOLDEN")) {
        REQUIRE(write_png(golden_path, image).has_value());
        WARN("ORRERY_UPDATE_GOLDEN=1: wrote the golden reference " << golden_path.string()
                                                                   << "; review it before "
                                                                      "committing");
        return;
    }
    if (!std::filesystem::exists(golden_path)) {
        FAIL("golden reference " << golden_path.string()
                                 << " is missing; generate it with ORRERY_UPDATE_GOLDEN=1 "
                                    "orrery_test_render \"[golden]\" and review it");
    }
    auto golden = read_png(golden_path);
    REQUIRE(golden.has_value());
    auto score = ssim(golden.value(), image);
    REQUIRE(score.has_value());
    INFO("actual image: " << actual_path.string());
    CHECK(score.value().value > 0.98);
}

} // namespace orrery::render::test
