#pragma once

// Private helpers of the layered (hydrostatic, linear temperature) atmosphere
// models. Not part of the public include directory.

#include <cmath>
#include <cstddef>
#include <span>

#include "orrery/atmo/atmosphere.h"

namespace orrery::atmo::detail {

// Temperature of `layer` at geopotential altitude h (m').
[[nodiscard]] inline double layer_temperature(const LinearLayer& layer, double h) noexcept {
    return layer.base_temperature_k + layer.lapse_rate_k_m * (h - layer.base_geopotential_m);
}

// Hydrostatic pressure of `layer` at geopotential altitude h, with
// g_over_r = g0 / R_specific (K/m'):
//   L != 0:  P = Pb (Tb / (Tb + L (h - hb)))^(g0 / (R L))
//   L == 0:  P = Pb exp(-g0 (h - hb) / (R Tb))
[[nodiscard]] inline double layer_pressure(const LinearLayer& layer, double h,
                                           double g_over_r) noexcept {
    const double dh = h - layer.base_geopotential_m;
    if (layer.lapse_rate_k_m == 0.0) {
        return layer.base_pressure_pa * std::exp(-g_over_r * dh / layer.base_temperature_k);
    }
    const double ratio = layer.base_temperature_k / layer_temperature(layer, h);
    return layer.base_pressure_pa * std::pow(ratio, g_over_r / layer.lapse_rate_k_m);
}

// Sets the base pressure of every layer from the surface pressure, each from
// the top of the layer below (fixed order, deterministic).
inline void fill_base_pressures(std::span<LinearLayer> layers, double surface_pressure,
                                double g_over_r) noexcept {
    layers[0].base_pressure_pa = surface_pressure;
    for (std::size_t i = 1; i < layers.size(); ++i) {
        layers[i].base_pressure_pa =
            layer_pressure(layers[i - 1], layers[i].base_geopotential_m, g_over_r);
    }
}

// The layer containing geopotential altitude h: the last one whose base is
// at or below h (the first layer for h below every base).
[[nodiscard]] inline const LinearLayer& find_layer(std::span<const LinearLayer> layers,
                                                   double h) noexcept {
    std::size_t index = 0;
    for (std::size_t i = 1; i < layers.size(); ++i) {
        if (h >= layers[i].base_geopotential_m) {
            index = i;
        }
    }
    return layers[index];
}

// Geopotential altitude of geometric altitude z on a sphere of radius r0.
[[nodiscard]] inline double geopotential(double z, double r0) noexcept {
    return r0 * z / (r0 + z);
}

// A NaN-filled sample.
[[nodiscard]] inline AtmosphereSample nan_sample() noexcept {
    const double nan = std::nan("");
    return AtmosphereSample{core::si::Density(nan), core::si::Pressure(nan), nan,
                            core::si::Velocity(nan), false};
}

} // namespace orrery::atmo::detail
