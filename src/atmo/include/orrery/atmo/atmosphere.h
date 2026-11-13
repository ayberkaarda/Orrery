#pragma once

#include <array>
#include <string_view>

#include "orrery/atmo/error.h"
#include "orrery/core/quantity.h"
#include "orrery/core/result.h"

// Atmosphere models (docs/atmosphere.md). Everything is SI (ADR-001); the
// altitude handed to a model is the geometric altitude above the reference
// sphere of the body that carries the atmosphere.
namespace orrery::atmo {

// Thermodynamic state of an atmosphere at one altitude.
struct AtmosphereSample {
    core::si::Density density{};
    core::si::Pressure pressure{};
    // Kinetic temperature, K.
    double temperature_k = 0.0;
    core::si::Velocity speed_of_sound{};
    // False above the model's top: density and pressure are zero there.
    bool inside = true;
};

// Ideal-gas properties of the medium: they set the speed of sound and
// therefore the Mach number.
struct GasProperties {
    // Ratio of specific heats.
    double gamma = 1.4;
    // Specific gas constant, J/(kg K).
    double gas_constant_j_kg_k = 287.0531;
};

// One atmosphere model behind the entry bridge.
//
// sample() is a pure, deterministic function of the altitude:
//   - altitudes above top() give density 0 and pressure 0 (inside = false)
//     with the temperature and speed of sound of the top;
//   - altitudes below the model's lowest altitude give the values there;
//   - a NaN altitude gives a NaN sample.
class IAtmosphere {
public:
    virtual ~IAtmosphere() = default;

    // Short identifier, e.g. "us76".
    [[nodiscard]] virtual std::string_view name() const noexcept = 0;
    [[nodiscard]] virtual AtmosphereSample sample(core::si::Length altitude) const noexcept = 0;
    // Altitude above which the model is a vacuum.
    [[nodiscard]] virtual core::si::Length top() const noexcept = 0;
    [[nodiscard]] virtual GasProperties gas() const noexcept = 0;

protected:
    IAtmosphere() = default;
    IAtmosphere(const IAtmosphere&) = default;
    IAtmosphere(IAtmosphere&&) = default;
    IAtmosphere& operator=(const IAtmosphere&) = default;
    IAtmosphere& operator=(IAtmosphere&&) = default;
};

// Mach number |airspeed| / a. Zero speed gives 0; a zero speed of sound gives
// +infinity for a non-zero speed.
[[nodiscard]] double mach_number(const AtmosphereSample& sample,
                                 core::si::Velocity airspeed) noexcept;

// One layer of a hydrostatic atmosphere with a linear temperature profile in
// geopotential altitude: T(H) = base_temperature + lapse_rate (H - base).
struct LinearLayer {
    // Geopotential altitude of the layer base, m'.
    double base_geopotential_m = 0.0;
    // Molecular-scale temperature at the base, K.
    double base_temperature_k = 0.0;
    // dT/dH, K per geopotential metre.
    double lapse_rate_k_m = 0.0;
    // Pressure at the base, Pa (computed by the model from the layers below).
    double base_pressure_pa = 0.0;
};

// U.S. Standard Atmosphere 1976 (NOAA/NASA/USAF, NOAA-S/T 76-1562).
//
// 0 to 86 km geometric altitude: the standard's seven-layer formulation in
// geopotential altitude H = r0 Z / (r0 + Z) with the exact constants of the
// standard (g0' = 9.80665, R* = 8314.32 J/(kmol K), M0 = 28.9644 kg/kmol,
// r0 = 6356766 m, P0 = 101325 Pa, T0 = 288.15 K, gamma = 1.4). The layer base
// pressures are computed from these constants, not tabulated. Between 80 and
// 86 km the kinetic temperature is the molecular-scale temperature times the
// standard's M/M0 ratio (Table 8); density and speed of sound depend on the
// molecular-scale temperature only.
//
// Above 86 km (up to the 1000 km top): piecewise exponential interpolation of
// the standard's tabulated density and pressure (log-linear between nodes)
// and linear interpolation of its kinetic temperature, starting from the
// layered model's own values at 86 km so the profile is continuous.
// APPROXIMATION (docs/physics.md, atmosphere): not the standard's
// diffusive-equilibrium formulation above 86 km.
// Below 0 km the lowest layer continues down to -5 km (the standard's lower
// limit) and is clamped below that.
class UsStandardAtmosphere1976 final : public IAtmosphere {
public:
    static constexpr double g0 = 9.80665;                   // m'/s^2
    static constexpr double gas_constant_star = 8314.32;    // J/(kmol K)
    static constexpr double molar_mass_sea_level = 28.9644; // kg/kmol
    static constexpr double earth_radius_r0 = 6356766.0;    // m
    static constexpr double sea_level_pressure = 101325.0;  // Pa
    static constexpr double sea_level_temperature = 288.15; // K
    static constexpr double gamma = 1.4;
    // Geometric altitude where the layered formulation ends (86 km), and its
    // geopotential altitude (84.852 km to the standard's precision).
    static constexpr double layered_top_m = 86000.0;
    static constexpr double model_top_m = 1000000.0;
    static constexpr double lower_limit_m = -5000.0;

    UsStandardAtmosphere1976();

    [[nodiscard]] std::string_view name() const noexcept override { return "us76"; }
    [[nodiscard]] AtmosphereSample sample(core::si::Length altitude) const noexcept override;
    [[nodiscard]] core::si::Length top() const noexcept override {
        return core::si::Length(model_top_m);
    }
    [[nodiscard]] GasProperties gas() const noexcept override;

    // Sample at a geopotential altitude in the layered region (H up to
    // 84.852 km), as tabulated by the standard.
    [[nodiscard]] AtmosphereSample
    sample_geopotential(core::si::Length geopotential) const noexcept;
    // Molecular-scale temperature TM, K, at a geopotential altitude.
    [[nodiscard]] double molecular_scale_temperature(core::si::Length geopotential) const noexcept;

    [[nodiscard]] static core::si::Length
    geopotential_altitude(core::si::Length geometric) noexcept;
    [[nodiscard]] static core::si::Length
    geometric_altitude(core::si::Length geopotential) noexcept;

    // The seven layers of the 0-86 km region with their computed base pressures.
    [[nodiscard]] const std::array<LinearLayer, 8>& layers() const noexcept { return layers_; }

private:
    std::array<LinearLayer, 8> layers_{};
};

// Simple Mars atmosphere (docs/atmosphere.md, "Mars").
//
// Temperature: the NASA Glenn Research Center Mars atmosphere curve fit to
// Mars Global Surveyor measurements (April 1996),
//   T = 242.15 K - 0.998 K/km h          (h < 7 km)
//   T = T(7 km) - 2.22 K/km (h - 7 km)   (7 km <= h < 50 km)
// held isothermal above 50 km (the fit falls below 0 K near 112 km); the two
// fit segments are joined continuously at 7 km (the published segments differ
// by 0.95 K there). The altitudes are taken as geopotential altitudes on a
// sphere of radius 3389.5 km with g0 = 3.711 m/s^2.
// Pressure: the hydrostatic equation integrated through the temperature
// layers from the fit's surface pressure 699 Pa (not the fit's own isothermal
// pressure curve), so pressure, density and temperature satisfy the ideal gas
// law with R = 192.1 J/(kg K) (the fit's CO2-rich value) and gamma = 1.29.
// Top: 250 km. APPROXIMATION: a single mean profile, no season, latitude,
// local time or dust dependence (Mars-GRAM covers those).
class MarsAtmosphere final : public IAtmosphere {
public:
    static constexpr double g0 = 3.711;
    static constexpr double reference_radius = 3389500.0;
    static constexpr double surface_pressure = 699.0;
    static constexpr double gas_constant = 192.1;
    static constexpr double gamma = 1.29;
    static constexpr double model_top_m = 250000.0;

    MarsAtmosphere();

    [[nodiscard]] std::string_view name() const noexcept override { return "mars"; }
    [[nodiscard]] AtmosphereSample sample(core::si::Length altitude) const noexcept override;
    [[nodiscard]] core::si::Length top() const noexcept override {
        return core::si::Length(model_top_m);
    }
    [[nodiscard]] GasProperties gas() const noexcept override;

    [[nodiscard]] const std::array<LinearLayer, 3>& layers() const noexcept { return layers_; }

private:
    std::array<LinearLayer, 3> layers_{};
};

struct ExponentialAtmosphereParameters {
    // Density at altitude 0.
    core::si::Density surface_density{1.225};
    core::si::Length scale_height{7200.0};
    // Uniform temperature, K: sets the pressure p = rho R T and the speed of
    // sound. Hydrostatic consistency needs scale_height = R T / g, which is
    // not enforced (the model is a density law first).
    double temperature_k = 288.15;
    GasProperties gas;
    core::si::Length top{150000.0};
};

// Generic isothermal exponential atmosphere rho = rho0 exp(-h / H), the model
// behind the Allen-Eggers ballistic entry solution. Altitudes below 0 are
// clamped to 0.
class ExponentialAtmosphere final : public IAtmosphere {
public:
    // Earth-like defaults (rho0 = 1.225 kg/m^3, H = 7.2 km).
    ExponentialAtmosphere() = default;

    // Rejects non-finite or non-positive density, scale height, temperature,
    // gas constant or top, and gamma <= 1.
    [[nodiscard]] static core::Result<ExponentialAtmosphere, AtmoError>
    create(const ExponentialAtmosphereParameters& parameters);

    [[nodiscard]] std::string_view name() const noexcept override { return "exponential"; }
    [[nodiscard]] AtmosphereSample sample(core::si::Length altitude) const noexcept override;
    [[nodiscard]] core::si::Length top() const noexcept override { return parameters_.top; }
    [[nodiscard]] GasProperties gas() const noexcept override { return parameters_.gas; }
    [[nodiscard]] const ExponentialAtmosphereParameters& parameters() const noexcept {
        return parameters_;
    }

private:
    explicit ExponentialAtmosphere(const ExponentialAtmosphereParameters& parameters) noexcept
        : parameters_(parameters) {}

    ExponentialAtmosphereParameters parameters_;
};

} // namespace orrery::atmo
