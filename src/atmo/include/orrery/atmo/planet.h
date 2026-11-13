#pragma once

#include <memory>
#include <string>

#include "orrery/atmo/atmosphere.h"
#include "orrery/core/quantity.h"

namespace orrery::atmo {

// The body that carries an atmosphere, as the entry bridge sees it (SI).
//
// The body is a sphere of `radius` (altitude 0) with point-mass gravity GM,
// rotating about the +z axis of the planet-centred inertial frame at
// `rotation_rate` (rad/s; its body-fixed frame coincides with the inertial
// frame at time 0, see docs/atmosphere.md "Frames").
// APPROXIMATION (E4): spherical body with point-mass gravity - J2 is 1e-3
// of g during entry - altitudes are above the mean sphere, not geodetic
// heights (up to about 14 km apart on Earth); no terrain.
struct Planet {
    std::string name;
    core::si::GravitationalParameter gm{};
    core::si::Length radius{};
    // Sidereal rotation rate, rad/s (counter-clockwise about +z).
    core::si::Frequency rotation_rate{};
    // Sutton-Graves constant of the atmosphere's gas, kg^0.5 / m.
    double sutton_graves_k = 0.0;
    // Conventional entry interface altitude (atmospheric entry and skip-out
    // events are reported where the trajectory crosses it).
    core::si::Length entry_interface_altitude{};
    // Shared, immutable model; never null for the presets.
    std::shared_ptr<const IAtmosphere> atmosphere;

    // Earth: GM 3.986004418e14 m^3/s^2 (IERS 2010), mean radius 6371.0 km,
    // rotation 7.292115e-5 rad/s, Sutton-Graves k 1.7415e-4, entry interface
    // 121.92 km (400 000 ft), US Standard Atmosphere 1976.
    [[nodiscard]] static Planet earth();
    // Mars: GM 4.282837e13 m^3/s^2, mean radius 3389.5 km, rotation
    // 7.088218e-5 rad/s, Sutton-Graves k 1.9027e-4 (CO2), entry interface
    // 125 km, MarsAtmosphere.
    [[nodiscard]] static Planet mars();
};

} // namespace orrery::atmo
