#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

#include "layers.h"
#include "orrery/atmo/atmosphere.h"

namespace orrery::atmo {

namespace {

using Model = UsStandardAtmosphere1976;

// g0 M0 / R*, K per geopotential metre (hydrostatic constant of the standard).
constexpr double kGOverR = Model::g0 * Model::molar_mass_sea_level / Model::gas_constant_star;
// R* / M0, J/(kg K).
constexpr double kAirGasConstant = Model::gas_constant_star / Model::molar_mass_sea_level;

// Layer bases (geopotential km) and lapse rates (K per geopotential km) of
// the standard's Table 4.
struct LayerDefinition {
    double base_km;
    double lapse_k_per_km;
};

constexpr std::array<LayerDefinition, 8> kLayers{{
    {0.0, -6.5},
    {11.0, 0.0},
    {20.0, 1.0},
    {32.0, 2.8},
    {47.0, 0.0},
    {51.0, -2.8},
    {71.0, -2.0},
    {84.852, 0.0},
}};

// Molecular-weight ratio M/M0 between 80 and 86 km geometric altitude, every
// 0.5 km (the standard's Table 8); 1 below 80 km.
constexpr double kRatioStartM = 80000.0;
constexpr double kRatioStepM = 500.0;
constexpr std::array<double, 13> kMolecularWeightRatio{
    1.000000, 0.999996, 0.999989, 0.999971, 0.999941, 0.999909, 0.999870,
    0.999829, 0.999786, 0.999741, 0.999694, 0.999641, 0.999579};

// Tabulated values above 86 km (the standard's Table I, geometric
// altitude): kinetic temperature, pressure and density. The 86 km node is
// replaced by the layered model's own values at construction.
struct UpperNode {
    double altitude_m;
    double temperature_k;
    double pressure_pa;
    double density_kg_m3;
};

constexpr std::array<UpperNode, 20> kUpperNodes{{
    {86000.0, 186.87, 3.7338e-1, 6.958e-6},   {90000.0, 186.87, 1.8359e-1, 3.416e-6},
    {95000.0, 188.42, 7.5966e-2, 1.393e-6},   {100000.0, 195.08, 3.2011e-2, 5.604e-7},
    {110000.0, 240.00, 7.1042e-3, 9.708e-8},  {120000.0, 360.00, 2.5382e-3, 2.222e-8},
    {130000.0, 469.27, 1.2505e-3, 8.152e-9},  {140000.0, 559.63, 7.2028e-4, 3.831e-9},
    {150000.0, 634.39, 4.5422e-4, 2.076e-9},  {160000.0, 696.29, 3.0395e-4, 1.233e-9},
    {180000.0, 790.07, 1.5271e-4, 5.194e-10}, {200000.0, 854.56, 8.4736e-5, 2.541e-10},
    {250000.0, 941.33, 2.4767e-5, 6.073e-11}, {300000.0, 976.01, 8.7704e-6, 1.916e-11},
    {400000.0, 995.83, 1.4518e-6, 2.803e-12}, {500000.0, 999.24, 3.0236e-7, 5.215e-13},
    {600000.0, 999.85, 8.2130e-8, 1.137e-13}, {700000.0, 999.97, 3.1908e-8, 3.070e-14},
    {800000.0, 999.99, 1.7036e-8, 1.136e-14}, {1000000.0, 1000.0, 7.5138e-9, 3.561e-15},
}};

// Callers pass a finite altitude; a NaN still returns 1 rather than reach
// the table index below.
double molecular_weight_ratio(double geometric_m) noexcept {
    if (!(geometric_m > kRatioStartM)) {
        return 1.0;
    }
    const double position = (geometric_m - kRatioStartM) / kRatioStepM;
    const auto last = static_cast<double>(kMolecularWeightRatio.size() - 1);
    if (position >= last) {
        return kMolecularWeightRatio.back();
    }
    const double cell = std::floor(position);
    const auto i = static_cast<std::size_t>(cell);
    const double f = position - cell;
    return kMolecularWeightRatio[i] + f * (kMolecularWeightRatio[i + 1] - kMolecularWeightRatio[i]);
}

AtmosphereSample from_layers(const Model& model, double geopotential_m, double geometric_m) {
    const LinearLayer& layer = detail::find_layer(model.layers(), geopotential_m);
    const double tm = detail::layer_temperature(layer, geopotential_m);
    const double pressure = detail::layer_pressure(layer, geopotential_m, kGOverR);
    AtmosphereSample s;
    s.pressure = core::si::Pressure(pressure);
    s.density = core::si::Density(pressure / (kAirGasConstant * tm));
    s.temperature_k = tm * molecular_weight_ratio(geometric_m);
    s.speed_of_sound = core::si::Velocity(std::sqrt(Model::gamma * kAirGasConstant * tm));
    return s;
}

double lerp(double a, double b, double f) noexcept {
    return a + f * (b - a);
}

double log_lerp(double a, double b, double f) noexcept {
    return a * std::exp(f * std::log(b / a));
}

AtmosphereSample from_upper_nodes(const UpperNode& base, double altitude_m) {
    std::size_t upper = 1;
    while (upper + 1 < kUpperNodes.size() && kUpperNodes[upper].altitude_m < altitude_m) {
        ++upper;
    }
    const UpperNode& lo = upper == 1 ? base : kUpperNodes[upper - 1];
    const UpperNode& hi = kUpperNodes[upper];
    const double f = (altitude_m - lo.altitude_m) / (hi.altitude_m - lo.altitude_m);
    AtmosphereSample s;
    s.temperature_k = lerp(lo.temperature_k, hi.temperature_k, f);
    const double pressure = log_lerp(lo.pressure_pa, hi.pressure_pa, f);
    const double density = log_lerp(lo.density_kg_m3, hi.density_kg_m3, f);
    s.pressure = core::si::Pressure(pressure);
    s.density = core::si::Density(density);
    s.speed_of_sound = core::si::Velocity(std::sqrt(Model::gamma * pressure / density));
    return s;
}

UpperNode layered_top_node(const Model& model) {
    const double h = Model::geopotential_altitude(core::si::Length(Model::layered_top_m)).value();
    const AtmosphereSample s = from_layers(model, h, Model::layered_top_m);
    return UpperNode{Model::layered_top_m, s.temperature_k, s.pressure.value(), s.density.value()};
}

} // namespace

UsStandardAtmosphere1976::UsStandardAtmosphere1976() {
    for (std::size_t i = 0; i < kLayers.size(); ++i) {
        layers_[i].base_geopotential_m = kLayers[i].base_km * 1000.0;
        layers_[i].lapse_rate_k_m = kLayers[i].lapse_k_per_km / 1000.0;
        layers_[i].base_temperature_k =
            i == 0 ? sea_level_temperature
                   : detail::layer_temperature(layers_[i - 1], layers_[i].base_geopotential_m);
    }
    detail::fill_base_pressures(layers_, sea_level_pressure, kGOverR);
}

core::si::Length
UsStandardAtmosphere1976::geopotential_altitude(core::si::Length geometric) noexcept {
    return core::si::Length(detail::geopotential(geometric.value(), earth_radius_r0));
}

core::si::Length
UsStandardAtmosphere1976::geometric_altitude(core::si::Length geopotential) noexcept {
    const double h = geopotential.value();
    return core::si::Length(earth_radius_r0 * h / (earth_radius_r0 - h));
}

double UsStandardAtmosphere1976::molecular_scale_temperature(
    core::si::Length geopotential) const noexcept {
    const double h = geopotential.value();
    return detail::layer_temperature(detail::find_layer(layers_, h), h);
}

AtmosphereSample
UsStandardAtmosphere1976::sample_geopotential(core::si::Length geopotential) const noexcept {
    const double h = geopotential.value();
    if (!std::isfinite(h)) {
        return detail::nan_sample();
    }
    return from_layers(*this, h, geometric_altitude(geopotential).value());
}

AtmosphereSample UsStandardAtmosphere1976::sample(core::si::Length altitude) const noexcept {
    const double z = altitude.value();
    if (std::isnan(z)) {
        return detail::nan_sample();
    }
    if (z <= layered_top_m) {
        const double clamped = std::max(z, lower_limit_m);
        return from_layers(*this, detail::geopotential(clamped, earth_radius_r0), clamped);
    }
    const UpperNode base = layered_top_node(*this);
    if (z > model_top_m) {
        AtmosphereSample s = from_upper_nodes(base, model_top_m);
        s.density = core::si::Density(0.0);
        s.pressure = core::si::Pressure(0.0);
        s.inside = false;
        return s;
    }
    return from_upper_nodes(base, z);
}

GasProperties UsStandardAtmosphere1976::gas() const noexcept {
    return GasProperties{gamma, kAirGasConstant};
}

} // namespace orrery::atmo
