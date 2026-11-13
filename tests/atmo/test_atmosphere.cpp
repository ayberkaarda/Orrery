#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <limits>

#include "orrery/atmo/atmosphere.h"
#include "orrery/core/quantity.h"

using orrery::atmo::AtmosphereSample;
using orrery::atmo::ExponentialAtmosphere;
using orrery::atmo::ExponentialAtmosphereParameters;
using orrery::atmo::MarsAtmosphere;
using orrery::atmo::UsStandardAtmosphere1976;
using orrery::core::si::Length;

namespace {

// A value of the standard's tables and half a unit of its last printed digit.
struct Published {
    double value;
    double half_unit;
};

bool matches(double computed, Published p) {
    return std::abs(computed - p.value) <= p.half_unit;
}

struct TableRow {
    double geopotential_km;
    Published temperature_k; // molecular-scale temperature TM
    Published pressure_pa;
    Published density_kg_m3;
    Published speed_of_sound_m_s;
};

// U.S. Standard Atmosphere 1976 (NOAA-S/T 76-1562), Table I, at the
// geopotential layer boundaries: TM to 0.001 K, pressure and density to five
// significant figures, speed of sound to 0.001 m/s.
constexpr std::array<TableRow, 9> kBoundaries{{
    {0.0, {288.150, 5e-4}, {1.01325e5, 0.5}, {1.2250, 5e-5}, {340.294, 5e-4}},
    {5.0, {255.650, 5e-4}, {5.4020e4, 0.5}, {7.3612e-1, 5e-6}, {320.530, 5e-4}},
    {11.0, {216.650, 5e-4}, {2.2632e4, 0.5}, {3.6392e-1, 5e-6}, {295.070, 5e-4}},
    {20.0, {216.650, 5e-4}, {5.4749e3, 5e-2}, {8.8035e-2, 5e-7}, {295.070, 5e-4}},
    {32.0, {228.650, 5e-4}, {8.6802e2, 5e-3}, {1.3225e-2, 5e-7}, {303.131, 5e-4}},
    {47.0, {270.650, 5e-4}, {1.1091e2, 5e-3}, {1.4275e-3, 5e-8}, {329.799, 5e-4}},
    {51.0, {270.650, 5e-4}, {6.6939e1, 5e-4}, {8.6160e-4, 5e-9}, {329.799, 5e-4}},
    {71.0, {214.650, 5e-4}, {3.9564, 5e-5}, {6.4211e-5, 5e-10}, {293.704, 5e-4}},
    {84.852, {186.946, 5e-4}, {3.7338e-1, 5e-6}, {6.9579e-6, 5e-11}, {274.096, 5e-4}},
}};

} // namespace

TEST_CASE("US76 reproduces the published values at the layer boundaries", "[atmo][us76]") {
    const UsStandardAtmosphere1976 us76;
    for (const TableRow& row : kBoundaries) {
        INFO("geopotential altitude " << row.geopotential_km << " km");
        const Length h(row.geopotential_km * 1000.0);
        const AtmosphereSample s = us76.sample_geopotential(h);
        CHECK(matches(us76.molecular_scale_temperature(h), row.temperature_k));
        CHECK(matches(s.pressure.value(), row.pressure_pa));
        CHECK(matches(s.density.value(), row.density_kg_m3));
        CHECK(matches(s.speed_of_sound.value(), row.speed_of_sound_m_s));
    }
}

TEST_CASE("US76 layer base pressures match the standard's Table 4", "[atmo][us76]") {
    // Base pressures to the standard's printed precision (seven figures).
    constexpr std::array<Published, 8> kBase{{{101325.0, 0.05},
                                              {22632.06, 0.005},
                                              {5474.889, 5e-4},
                                              {868.0187, 5e-5},
                                              {110.9063, 5e-5},
                                              {66.93887, 5e-6},
                                              {3.956420, 5e-7},
                                              {0.3733836, 5e-8}}};
    const UsStandardAtmosphere1976 us76;
    for (std::size_t i = 0; i < kBase.size(); ++i) {
        INFO("layer " << i);
        CHECK(matches(us76.layers()[i].base_pressure_pa, kBase[i]));
    }
}

TEST_CASE("US76 by geometric altitude", "[atmo][us76]") {
    const UsStandardAtmosphere1976 us76;
    // Table I at Z = 5 km: T 255.676 K, P 5.4048e4 Pa, rho 7.3643e-1 kg/m^3.
    const AtmosphereSample z5 = us76.sample(Length(5000.0));
    CHECK(matches(z5.temperature_k, {255.676, 5e-4}));
    CHECK(matches(z5.pressure.value(), {5.4048e4, 0.5}));
    CHECK(matches(z5.density.value(), {7.3643e-1, 5e-6}));
    // Z = 86 km is H = 84.852 km: kinetic temperature 186.87 K (TM times
    // M/M0 = 0.999579), density 6.958e-6 kg/m^3.
    const AtmosphereSample z86 = us76.sample(Length(86000.0));
    CHECK(matches(z86.temperature_k, {186.87, 5e-3}));
    CHECK(matches(z86.density.value(), {6.958e-6, 5e-10}));
    CHECK(std::abs(UsStandardAtmosphere1976::geopotential_altitude(Length(86000.0)).value() -
                   84852.0) < 0.5);
    CHECK(std::abs(UsStandardAtmosphere1976::geometric_altitude(
                       UsStandardAtmosphere1976::geopotential_altitude(Length(86000.0)))
                       .value() -
                   86000.0) < 1e-6);
}

TEST_CASE("US76 above 86 km follows the tabulated values continuously", "[atmo][us76]") {
    const UsStandardAtmosphere1976 us76;
    // Nodes are reproduced exactly (Table I): 120 km and 300 km.
    const AtmosphereSample z120 = us76.sample(Length(120000.0));
    CHECK(std::abs(z120.density.value() / 2.222e-8 - 1.0) < 1e-12);
    CHECK(std::abs(z120.temperature_k - 360.0) < 1e-9);
    const AtmosphereSample z300 = us76.sample(Length(300000.0));
    CHECK(std::abs(z300.density.value() / 1.916e-11 - 1.0) < 1e-12);
    // Continuity at 86 km and monotonic density up to the top.
    const double below = us76.sample(Length(85999.999)).density.value();
    const double above = us76.sample(Length(86000.001)).density.value();
    CHECK(std::abs(above / below - 1.0) < 1e-6);
    double previous = std::numeric_limits<double>::infinity();
    for (int step = 0; step <= 402; ++step) {
        const double z = -5000.0 + 2500.0 * step;
        const double rho = us76.sample(Length(z)).density.value();
        REQUIRE(rho < previous);
        previous = rho;
    }
    // Above the top: vacuum, temperature of the top.
    const AtmosphereSample space = us76.sample(Length(2.0e6));
    CHECK_FALSE(space.inside);
    CHECK(space.density.value() == 0.0);
    CHECK(space.pressure.value() == 0.0);
    CHECK(std::abs(space.temperature_k - 1000.0) < 1e-9);
}

TEST_CASE("US76 below sea level and invalid altitudes", "[atmo][us76]") {
    const UsStandardAtmosphere1976 us76;
    // Lowest layer continues to -5 km and is clamped below.
    const AtmosphereSample minus5 = us76.sample(Length(-5000.0));
    CHECK(minus5.temperature_k > 288.15);
    const AtmosphereSample minus9 = us76.sample(Length(-9000.0));
    CHECK(minus9.density == minus5.density);
    const AtmosphereSample nan = us76.sample(Length(std::nan("")));
    CHECK(std::isnan(nan.density.value()));
    CHECK_FALSE(nan.inside);
}

TEST_CASE("Mars model follows the NASA Glenn fit near the surface", "[atmo][mars]") {
    const MarsAtmosphere mars;
    const AtmosphereSample surface = mars.sample(Length(0.0));
    CHECK(std::abs(surface.pressure.value() - 699.0) < 1e-9);
    CHECK(std::abs(surface.temperature_k - 242.15) < 1e-9);
    // rho = p / (R T): Glenn's 0.699 / (0.1921 (242.1)) = 0.01503 kg/m^3.
    CHECK(std::abs(surface.density.value() - 0.01503) < 5e-5);
    // Glenn's own pressure fit p = 699 exp(-9e-5 h) Pa is not hydrostatic
    // with its temperature fit; the hydrostatic integration stays within 10
    // percent of it up to 10 km.
    for (const double h : {2000.0, 5000.0, 10000.0}) {
        const double glenn = 699.0 * std::exp(-9.0e-5 * h);
        CHECK(std::abs(mars.sample(Length(h)).pressure.value() / glenn - 1.0) < 0.10);
    }
    // Isothermal above 50 km, continuous temperature at 7 km.
    CHECK(mars.sample(Length(80000.0)).temperature_k ==
          mars.sample(Length(120000.0)).temperature_k);
    CHECK(std::abs(mars.sample(Length(6999.999)).temperature_k -
                   mars.sample(Length(7000.001)).temperature_k) < 1e-5);
    // Gas: CO2-rich, gamma 1.29, a = sqrt(gamma R T) about 245 m/s at the surface.
    CHECK(std::abs(surface.speed_of_sound.value() - std::sqrt(1.29 * 192.1 * 242.15)) < 1e-9);
    CHECK_FALSE(mars.sample(Length(300000.0)).inside);
}

TEST_CASE("Exponential atmosphere", "[atmo][exponential]") {
    ExponentialAtmosphereParameters p;
    p.surface_density = orrery::core::si::Density(0.02);
    p.scale_height = Length(11100.0);
    p.temperature_k = 210.0;
    auto model = ExponentialAtmosphere::create(p);
    REQUIRE(model);
    const ExponentialAtmosphere& m = model.value();
    CHECK(m.sample(Length(0.0)).density.value() == 0.02);
    CHECK(std::abs(m.sample(Length(11100.0)).density.value() / (0.02 / std::exp(1.0)) - 1.0) <
          1e-14);
    CHECK(m.sample(Length(-100.0)).density.value() == 0.02);
    const AtmosphereSample s = m.sample(Length(5000.0));
    CHECK(std::abs(s.pressure.value() - s.density.value() * 287.0531 * 210.0) < 1e-12);
    CHECK_FALSE(m.sample(Length(200000.0)).inside);

    ExponentialAtmosphereParameters bad = p;
    bad.scale_height = Length(0.0);
    CHECK_FALSE(ExponentialAtmosphere::create(bad));
    bad = p;
    bad.gas.gamma = 1.0;
    CHECK_FALSE(ExponentialAtmosphere::create(bad));
}

TEST_CASE("Mach number of a state", "[atmo]") {
    const UsStandardAtmosphere1976 us76;
    const AtmosphereSample s = us76.sample(Length(0.0));
    using orrery::core::si::Velocity;
    CHECK(std::abs(orrery::atmo::mach_number(s, Velocity(340.294)) - 1.0) < 1e-6);
    CHECK(orrery::atmo::mach_number(s, Velocity(0.0)) == 0.0);
    CHECK(orrery::atmo::mach_number(AtmosphereSample{}, Velocity(1.0)) ==
          std::numeric_limits<double>::infinity());
}
