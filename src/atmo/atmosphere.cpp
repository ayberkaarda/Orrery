#include "orrery/atmo/atmosphere.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "layers.h"

namespace orrery::atmo {

namespace {

// Mars layers (geopotential km, K, K per km): the NASA Glenn fit below 7 km,
// its upper lapse rate from the continuous join at 7 km up to 50 km, and an
// isothermal region above.
constexpr double kMarsSurfaceTemperature = 242.15;
constexpr double kMarsLowerLapse = -0.998e-3;
constexpr double kMarsUpperLapse = -2.22e-3;
constexpr double kMarsJoin = 7000.0;
constexpr double kMarsIsothermalBase = 50000.0;
constexpr double kMarsLowerLimit = -8000.0;

bool positive_finite(double v) noexcept {
    return v > 0.0 && std::isfinite(v);
}

AtmosphereSample layered_sample(std::span<const LinearLayer> layers, double h, double g_over_r,
                                const GasProperties& gas) {
    const LinearLayer& layer = detail::find_layer(layers, h);
    const double t = detail::layer_temperature(layer, h);
    const double p = detail::layer_pressure(layer, h, g_over_r);
    AtmosphereSample s;
    s.pressure = core::si::Pressure(p);
    s.density = core::si::Density(p / (gas.gas_constant_j_kg_k * t));
    s.temperature_k = t;
    s.speed_of_sound = core::si::Velocity(std::sqrt(gas.gamma * gas.gas_constant_j_kg_k * t));
    return s;
}

AtmoError parameter_error(const std::string& what) {
    return AtmoError{AtmoErrorCode::invalid_parameter, what};
}

} // namespace

double mach_number(const AtmosphereSample& sample, core::si::Velocity airspeed) noexcept {
    const double v = std::abs(airspeed.value());
    if (v == 0.0) {
        return 0.0;
    }
    const double a = sample.speed_of_sound.value();
    if (a == 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    return v / a;
}

MarsAtmosphere::MarsAtmosphere() {
    layers_[0] = LinearLayer{0.0, kMarsSurfaceTemperature, kMarsLowerLapse, 0.0};
    const double join_temperature = detail::layer_temperature(layers_[0], kMarsJoin);
    layers_[1] = LinearLayer{kMarsJoin, join_temperature, kMarsUpperLapse, 0.0};
    const double top_temperature = detail::layer_temperature(layers_[1], kMarsIsothermalBase);
    layers_[2] = LinearLayer{kMarsIsothermalBase, top_temperature, 0.0, 0.0};
    detail::fill_base_pressures(layers_, surface_pressure, g0 / gas_constant);
}

AtmosphereSample MarsAtmosphere::sample(core::si::Length altitude) const noexcept {
    const double z = altitude.value();
    if (std::isnan(z)) {
        return detail::nan_sample();
    }
    const double clamped = std::clamp(z, kMarsLowerLimit, model_top_m);
    AtmosphereSample s = layered_sample(layers_, detail::geopotential(clamped, reference_radius),
                                        g0 / gas_constant, gas());
    if (z > model_top_m) {
        s.density = core::si::Density(0.0);
        s.pressure = core::si::Pressure(0.0);
        s.inside = false;
    }
    return s;
}

GasProperties MarsAtmosphere::gas() const noexcept {
    return GasProperties{gamma, gas_constant};
}

core::Result<ExponentialAtmosphere, AtmoError>
ExponentialAtmosphere::create(const ExponentialAtmosphereParameters& parameters) {
    if (!positive_finite(parameters.surface_density.value())) {
        return core::failure(parameter_error("surface density must be positive and finite"));
    }
    if (!positive_finite(parameters.scale_height.value())) {
        return core::failure(parameter_error("scale height must be positive and finite"));
    }
    if (!positive_finite(parameters.temperature_k)) {
        return core::failure(parameter_error("temperature must be positive and finite"));
    }
    if (!positive_finite(parameters.gas.gas_constant_j_kg_k)) {
        return core::failure(parameter_error("gas constant must be positive and finite"));
    }
    if (!(parameters.gas.gamma > 1.0) || !std::isfinite(parameters.gas.gamma)) {
        return core::failure(parameter_error("gamma must be above 1 and finite"));
    }
    if (!positive_finite(parameters.top.value())) {
        return core::failure(parameter_error("top must be positive and finite"));
    }
    return ExponentialAtmosphere(parameters);
}

AtmosphereSample ExponentialAtmosphere::sample(core::si::Length altitude) const noexcept {
    const double z = altitude.value();
    if (std::isnan(z)) {
        return detail::nan_sample();
    }
    const ExponentialAtmosphereParameters& p = parameters_;
    const double r = p.gas.gas_constant_j_kg_k;
    AtmosphereSample s;
    s.temperature_k = p.temperature_k;
    s.speed_of_sound = core::si::Velocity(std::sqrt(p.gas.gamma * r * p.temperature_k));
    if (z > p.top.value()) {
        s.inside = false;
        return s;
    }
    const double h = std::max(z, 0.0);
    const double rho = p.surface_density.value() * std::exp(-h / p.scale_height.value());
    s.density = core::si::Density(rho);
    s.pressure = core::si::Pressure(rho * r * p.temperature_k);
    return s;
}

} // namespace orrery::atmo
