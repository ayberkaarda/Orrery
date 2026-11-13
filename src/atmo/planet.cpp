#include "orrery/atmo/planet.h"

#include <memory>

#include "orrery/atmo/error.h"

namespace orrery::atmo {

std::string_view to_string(AtmoErrorCode code) noexcept {
    switch (code) {
        case AtmoErrorCode::invalid_parameter:
            return "invalid_parameter";
        case AtmoErrorCode::invalid_table:
            return "invalid_table";
        case AtmoErrorCode::solver_unavailable:
            return "solver_unavailable";
        case AtmoErrorCode::solver_failed:
            return "solver_failed";
        case AtmoErrorCode::integration_failed:
            return "integration_failed";
    }
    return "unknown";
}

Planet Planet::earth() {
    Planet p;
    p.name = "earth";
    p.gm = core::si::GravitationalParameter(3.986004418e14);
    p.radius = core::si::Length(6371000.0);
    p.rotation_rate = core::si::Frequency(7.292115e-5);
    p.sutton_graves_k = 1.7415e-4;
    p.entry_interface_altitude = core::si::Length(121920.0);
    p.atmosphere = std::make_shared<const UsStandardAtmosphere1976>();
    return p;
}

Planet Planet::mars() {
    Planet p;
    p.name = "mars";
    p.gm = core::si::GravitationalParameter(4.282837e13);
    p.radius = core::si::Length(3389500.0);
    p.rotation_rate = core::si::Frequency(7.088218e-5);
    p.sutton_graves_k = 1.9027e-4;
    p.entry_interface_altitude = core::si::Length(125000.0);
    p.atmosphere = std::make_shared<const MarsAtmosphere>();
    return p;
}

} // namespace orrery::atmo
