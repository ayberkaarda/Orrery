#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <utility>

#include "convert.h"
#include "entry_internal.h"
#include "orrery/aero/newtonian.h"

namespace orrery::atmo {

namespace {

constexpr double kDegree = 180.0 / std::numbers::pi;
// Standard gravity for the g-load, m/s^2.
constexpr double kStandardGravity = 9.80665;

using Column = std::vector<double> EntrySeries::*;
using Field = double EntryRow::*;

constexpr std::array<std::pair<Column, Field>, 15> kColumns{{
    {&EntrySeries::time_s, &EntryRow::time_s},
    {&EntrySeries::altitude_m, &EntryRow::altitude_m},
    {&EntrySeries::speed_m_s, &EntryRow::speed_m_s},
    {&EntrySeries::flight_path_angle_deg, &EntryRow::flight_path_angle_deg},
    {&EntrySeries::airspeed_m_s, &EntryRow::airspeed_m_s},
    {&EntrySeries::mach, &EntryRow::mach},
    {&EntrySeries::cd, &EntryRow::cd},
    {&EntrySeries::density_kg_m3, &EntryRow::density_kg_m3},
    {&EntrySeries::dynamic_pressure_pa, &EntryRow::dynamic_pressure_pa},
    {&EntrySeries::g_load, &EntryRow::g_load},
    {&EntrySeries::heat_flux_w_m2, &EntryRow::heat_flux_w_m2},
    {&EntrySeries::latitude_deg, &EntryRow::latitude_deg},
    {&EntrySeries::longitude_deg, &EntryRow::longitude_deg},
    {&EntrySeries::drag_work_j, &EntryRow::drag_work_j},
    {&EntrySeries::energy_j, &EntryRow::energy_j},
}};

double wrap_degrees(double degrees) noexcept {
    return core::wrap_pi(core::Angle::from_degrees(degrees)).degrees();
}

// Linear interpolation of every field at fraction f from a to b (longitude
// along the shorter arc).
EntryRow interpolate(const EntryRow& a, const EntryRow& b, double f) {
    EntryRow row;
    for (const auto& [column, field] : kColumns) {
        row.*field = a.*field + f * (b.*field - a.*field);
    }
    row.drag_power_w = a.drag_power_w + f * (b.drag_power_w - a.drag_power_w);
    const double dlon = wrap_degrees(b.longitude_deg - a.longitude_deg);
    row.longitude_deg = wrap_degrees(a.longitude_deg + f * dlon);
    return row;
}

double flight_path_angle_deg(const core::Vec3<core::si::Length>& r,
                             const core::Vec3<core::si::Velocity>& v) {
    const double rv = core::norm(r).value() * core::norm(v).value();
    if (!(rv > 0.0)) {
        return 0.0;
    }
    return std::asin(std::clamp(core::dot(r, v).value() / rv, -1.0, 1.0)) * kDegree;
}

double heat_flux(const Planet& planet, const EntryVehicle& vehicle, const DragForce& drag) {
    const double rho = drag.atmosphere.density.value();
    if (!(rho > 0.0) || planet.sutton_graves_k == 0.0) {
        return 0.0;
    }
    return aero::sutton_graves_heat_flux(rho, drag.airspeed.value(), vehicle.nose_radius.value(),
                                         planet.sutton_graves_k);
}

// Apoapsis altitude of the osculating two-body orbit; 0 when unbound.
double apoapsis_altitude(const Planet& planet, const core::Vec3<core::si::Length>& r,
                         const core::Vec3<core::si::Velocity>& v, double specific_energy) {
    if (!(specific_energy < 0.0)) {
        return 0.0;
    }
    const double mu = planet.gm.value();
    const double a = -mu / (2.0 * specific_energy);
    const double h = core::norm(core::cross(r, v)).value();
    const double e = std::sqrt(std::max(0.0, 1.0 + 2.0 * specific_energy * h * h / (mu * mu)));
    return a * (1.0 + e) - planet.radius.value();
}

} // namespace

void EntryExperiment::Impl::start() {
    append(measure());
    summary.min_altitude_m = series.altitude_m.back();
    summary.final_speed_m_s = series.speed_m_s.back();
    const EntryRow row = last_row();
    if (row.altitude_m <= config.planet.entry_interface_altitude.value()) {
        events.emplace_back(
            AtmosphericEntryEvent{0.0, row.altitude_m, row.speed_m_s, row.flight_path_angle_deg});
        inside_interface = true;
    }
}

EntryRow EntryExperiment::Impl::measure() const {
    const core::BodyStore& bodies = state.bodies();
    const auto r = detail::to_si_vec(bodies.position(kProbeIndex) - bodies.position(kPlanetIndex));
    const auto v = detail::to_si_vec(bodies.velocity(kProbeIndex) - bodies.velocity(kPlanetIndex));
    const DragForce drag_now = drag.evaluate(core::AeroRef{0}, r, v);
    EntryRow row;
    row.time_s = time_s;
    row.altitude_m = drag_now.altitude.value();
    const double speed = core::norm(v).value();
    row.speed_m_s = speed;
    row.flight_path_angle_deg = flight_path_angle_deg(r, v);
    row.airspeed_m_s = drag_now.airspeed.value();
    row.mach = drag_now.mach;
    row.cd = drag_now.cd;
    row.density_kg_m3 = drag_now.atmosphere.density.value();
    row.dynamic_pressure_pa = drag_now.dynamic_pressure.value();
    row.g_load = core::norm(drag_now.force).value() / mass_kg / kStandardGravity;
    row.heat_flux_w_m2 = heat_flux(config.planet, config.vehicle, drag_now);
    const GroundPoint ground = ground_point(config.planet, r, core::si::Time(time_s));
    row.latitude_deg = ground.latitude_deg;
    row.longitude_deg = ground.longitude_deg;
    row.drag_work_j = drag_work_j;
    row.energy_j =
        mass_kg * (0.5 * speed * speed - config.planet.gm.value() / core::norm(r).value());
    row.drag_power_w = core::dot(drag_now.force, v).value();
    return row;
}

void EntryExperiment::Impl::append(const EntryRow& row) {
    for (const auto& [column, field] : kColumns) {
        (series.*column).push_back(row.*field);
    }
    drag_power_w.push_back(row.drag_power_w);
}

void EntryExperiment::Impl::replace_last(const EntryRow& row) {
    for (const auto& [column, field] : kColumns) {
        (series.*column).back() = row.*field;
    }
    drag_power_w.back() = row.drag_power_w;
}

EntryRow EntryExperiment::Impl::last_row() const {
    EntryRow row;
    for (const auto& [column, field] : kColumns) {
        row.*field = (series.*column).back();
    }
    row.drag_power_w = drag_power_w.back();
    return row;
}

void EntryExperiment::Impl::detect_events(const EntryRow& before, const EntryRow& after) {
    const double interface = config.planet.entry_interface_altitude.value();
    const auto crossing = [&](double level) {
        return interpolate(before, after,
                           (before.altitude_m - level) / (before.altitude_m - after.altitude_m));
    };
    if (!inside_interface && before.altitude_m > interface && after.altitude_m <= interface) {
        const EntryRow at = crossing(interface);
        events.emplace_back(
            AtmosphericEntryEvent{at.time_s, interface, at.speed_m_s, at.flight_path_angle_deg});
        inside_interface = true;
    }
    if (after.altitude_m <= 0.0) {
        EntryRow at = crossing(0.0);
        at.altitude_m = 0.0;
        replace_last(at);
        events.emplace_back(ImpactEvent{at.time_s, at.speed_m_s, at.airspeed_m_s,
                                        at.flight_path_angle_deg, at.latitude_deg,
                                        at.longitude_deg});
        summary.outcome = EntryOutcome::impact;
        return;
    }
    if (inside_interface && before.altitude_m < interface && after.altitude_m >= interface) {
        const EntryRow at = crossing(interface);
        const core::BodyStore& bodies = state.bodies();
        const double energy = at.energy_j / mass_kg;
        const double apoapsis =
            apoapsis_altitude(config.planet, detail::to_si_vec(bodies.position(kProbeIndex)),
                              detail::to_si_vec(bodies.velocity(kProbeIndex)), energy);
        events.emplace_back(SkipOutEvent{at.time_s, interface, at.speed_m_s,
                                         at.flight_path_angle_deg, energy, energy < 0.0, apoapsis});
        inside_interface = false;
        if (config.stop_on_skip_out) {
            summary.outcome = EntryOutcome::skip_out;
        }
    }
}

void EntryExperiment::Impl::update_summary(const EntryRow& before, const EntryRow& after) {
    EntrySummary& s = summary;
    if (after.g_load > s.peak_g_load) {
        s.peak_g_load = after.g_load;
        s.peak_g_time_s = after.time_s;
    }
    if (after.heat_flux_w_m2 > s.peak_heat_flux_w_m2) {
        s.peak_heat_flux_w_m2 = after.heat_flux_w_m2;
        s.peak_heat_flux_time_s = after.time_s;
    }
    if (after.dynamic_pressure_pa > s.peak_dynamic_pressure_pa) {
        s.peak_dynamic_pressure_pa = after.dynamic_pressure_pa;
        s.peak_dynamic_pressure_time_s = after.time_s;
    }
    s.heat_load_j_m2 +=
        0.5 * (before.heat_flux_w_m2 + after.heat_flux_w_m2) * (after.time_s - before.time_s);
    s.min_altitude_m = std::min(s.min_altitude_m, after.altitude_m);
    s.duration_s = after.time_s;
    s.final_speed_m_s = after.speed_m_s;
}

EntryEnergyBalance EntryExperiment::Impl::energy_balance() const noexcept {
    EntryEnergyBalance b;
    if (series.size() == 0) {
        return b;
    }
    b.initial_energy_j = series.energy_j.front();
    b.energy_j = series.energy_j.back();
    b.energy_change_j = b.energy_j - b.initial_energy_j;
    b.drag_work_j = series.drag_work_j.back();
    for (std::size_t i = 1; i < series.size(); ++i) {
        b.drag_work_quadrature_j += 0.5 * (drag_power_w[i - 1] + drag_power_w[i]) *
                                    (series.time_s[i] - series.time_s[i - 1]);
    }
    b.residual_j = b.energy_change_j - b.drag_work_j;
    if (b.drag_work_quadrature_j != 0.0) {
        b.relative_error = std::abs(b.energy_change_j - b.drag_work_quadrature_j) /
                           std::abs(b.drag_work_quadrature_j);
    }
    return b;
}

EntryConfig apollo_like_entry() {
    EntryConfig c;
    c.vehicle.name = "apollo_cm";
    c.vehicle.mass = core::si::Mass(5560.0);
    const double radius = 0.5 * 3.912;
    auto table =
        AeroCoefficientTable::constant(1.3, core::si::Area(std::numbers::pi * radius * radius));
    c.vehicle.aero = std::move(table).value();
    c.vehicle.nose_radius = core::si::Length(4.694);
    c.initial.altitude = core::si::Length(121920.0);
    c.initial.speed = core::si::Velocity(11000.0);
    c.initial.flight_path_angle = core::Angle::from_degrees(-5.7);
    return c;
}

EntryConfig skip_out_entry() {
    EntryConfig c = apollo_like_entry();
    c.initial.flight_path_angle = core::Angle::from_degrees(-5.0);
    return c;
}

} // namespace orrery::atmo
