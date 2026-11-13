#include "orrery/sim/entry_model.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace orrery::sim {

namespace {

constexpr double kDegree = std::numbers::pi / 180.0;

bool positive(double value) noexcept {
    return std::isfinite(value) && value > 0.0;
}

void add_if(std::vector<std::string>& errors, bool failed, const char* message) {
    if (failed) {
        errors.emplace_back(message);
    }
}

void cd_table_errors(const EntrySetup& setup, std::vector<std::string>& errors) {
    if (setup.cd_table.empty()) {
        errors.emplace_back("The Cd(Mach) table needs at least one point");
        return;
    }
    double previous = -1.0;
    for (const MachCd& point : setup.cd_table) {
        if (!std::isfinite(point.mach) || point.mach < 0.0 || !positive(point.cd)) {
            errors.emplace_back("Every Cd(Mach) point needs a Mach number >= 0 and a positive Cd");
            return;
        }
        if (point.mach <= previous) {
            errors.emplace_back(
                "Cd(Mach) points must be sorted by strictly increasing Mach number");
            return;
        }
        previous = point.mach;
    }
}

} // namespace

std::string_view to_string(EntryPlanet planet) noexcept {
    return planet == EntryPlanet::mars ? "mars" : "earth";
}

std::string_view to_string(EntryAtmosphere atmosphere) noexcept {
    return atmosphere == EntryAtmosphere::exponential ? "exponential" : "standard";
}

std::string_view to_string(EntryOutcome outcome) noexcept {
    switch (outcome) {
        case EntryOutcome::none:
            return "none";
        case EntryOutcome::landed:
            return "landed";
        case EntryOutcome::skip_out:
            return "skip_out";
        case EntryOutcome::orbiting:
            return "orbiting";
    }
    return "none";
}

std::string_view outcome_label(EntryOutcome outcome) noexcept {
    switch (outcome) {
        case EntryOutcome::none:
            return "";
        case EntryOutcome::landed:
            return "Landed";
        case EntryOutcome::skip_out:
            return "Skip-out";
        case EntryOutcome::orbiting:
            return "Orbiting";
    }
    return "";
}

bool EntrySeries::consistent() const noexcept {
    const std::size_t n = time_s.size();
    return altitude_m.size() == n && speed_m_s.size() == n && flight_path_angle_deg.size() == n &&
           g_load.size() == n && dynamic_pressure_pa.size() == n && heat_flux_w_m2.size() == n &&
           mach.size() == n && latitude_deg.size() == n && longitude_deg.size() == n &&
           downrange_m.size() == n;
}

void EntrySeries::push(const EntrySample& s) {
    time_s.push_back(s.time_s);
    altitude_m.push_back(s.altitude_m);
    speed_m_s.push_back(s.speed_m_s);
    flight_path_angle_deg.push_back(s.flight_path_angle_deg);
    g_load.push_back(s.g_load);
    dynamic_pressure_pa.push_back(s.dynamic_pressure_pa);
    heat_flux_w_m2.push_back(s.heat_flux_w_m2);
    mach.push_back(s.mach);
    latitude_deg.push_back(s.latitude_deg);
    longitude_deg.push_back(s.longitude_deg);
    downrange_m.push_back(s.downrange_m);
}

EntrySample EntrySeries::at(std::size_t i) const {
    return {time_s.at(i),         altitude_m.at(i),
            speed_m_s.at(i),      flight_path_angle_deg.at(i),
            g_load.at(i),         dynamic_pressure_pa.at(i),
            heat_flux_w_m2.at(i), mach.at(i),
            latitude_deg.at(i),   longitude_deg.at(i),
            downrange_m.at(i)};
}

void EntrySeries::clear() noexcept {
    for (std::vector<double>* v :
         {&time_s, &altitude_m, &speed_m_s, &flight_path_angle_deg, &g_load, &dynamic_pressure_pa,
          &heat_flux_w_m2, &mach, &latitude_deg, &longitude_deg, &downrange_m}) {
        v->clear();
    }
}

void EntrySeries::reserve(std::size_t count) {
    for (std::vector<double>* v :
         {&time_s, &altitude_m, &speed_m_s, &flight_path_angle_deg, &g_load, &dynamic_pressure_pa,
          &heat_flux_w_m2, &mach, &latitude_deg, &longitude_deg, &downrange_m}) {
        v->reserve(count);
    }
}

EntrySummary summarize(const EntrySeries& series, EntryOutcome outcome) {
    EntrySummary s;
    s.outcome = outcome;
    if (series.empty() || !series.consistent()) {
        return s;
    }
    const std::size_t n = series.size();
    for (std::size_t i = 0; i < n; ++i) {
        const double t = series.time_s[i];
        if (std::isfinite(series.g_load[i]) && series.g_load[i] > s.peak_g) {
            s.peak_g = series.g_load[i];
            s.peak_g_time_s = t;
        }
        if (std::isfinite(series.heat_flux_w_m2[i]) &&
            series.heat_flux_w_m2[i] > s.peak_heat_flux_w_m2) {
            s.peak_heat_flux_w_m2 = series.heat_flux_w_m2[i];
            s.peak_heat_flux_time_s = t;
        }
        if (std::isfinite(series.dynamic_pressure_pa[i])) {
            s.peak_dynamic_pressure_pa =
                std::max(s.peak_dynamic_pressure_pa, series.dynamic_pressure_pa[i]);
        }
        if (i > 0) {
            const double dt = t - series.time_s[i - 1];
            const double q = 0.5 * (series.heat_flux_w_m2[i] + series.heat_flux_w_m2[i - 1]);
            if (std::isfinite(dt) && std::isfinite(q) && dt > 0.0) {
                s.heat_load_j_m2 += q * dt;
            }
        }
    }
    s.duration_s = series.time_s.back() - series.time_s.front();
    s.downrange_m = series.downrange_m.back();
    s.final_speed_m_s = series.speed_m_s.back();
    s.final_altitude_m = series.altitude_m.back();
    return s;
}

std::optional<double> value_at(const std::vector<double>& time_s, const std::vector<double>& values,
                               double t) {
    if (time_s.empty() || time_s.size() != values.size() || !std::isfinite(t)) {
        return std::nullopt;
    }
    if (t <= time_s.front()) {
        return values.front();
    }
    if (t >= time_s.back()) {
        return values.back();
    }
    const auto upper = std::upper_bound(time_s.begin(), time_s.end(), t);
    const auto i = static_cast<std::size_t>(upper - time_s.begin());
    const double t0 = time_s[i - 1];
    const double t1 = time_s[i];
    const double f = t1 > t0 ? (t - t0) / (t1 - t0) : 0.0;
    return values[i - 1] + f * (values[i] - values[i - 1]);
}

std::optional<std::size_t> nearest_index(const std::vector<double>& time_s, double t) {
    if (time_s.empty() || !std::isfinite(t)) {
        return std::nullopt;
    }
    const auto upper = std::upper_bound(time_s.begin(), time_s.end(), t);
    if (upper == time_s.begin()) {
        return 0;
    }
    if (upper == time_s.end()) {
        return time_s.size() - 1;
    }
    const auto i = static_cast<std::size_t>(upper - time_s.begin());
    return (t - time_s[i - 1] <= time_s[i] - t) ? i - 1 : i;
}

PlanetConstants planet_constants(EntryPlanet planet) noexcept {
    if (planet == EntryPlanet::mars) {
        return {"Mars", 3'389'500.0, 4.282837e13, 125'000.0};
    }
    return {"Earth", 6'371'000.0, 3.986004418e14, 121'920.0};
}

std::optional<double> flight_path_angle_for_periapsis(EntryPlanet planet, double altitude_m,
                                                      double speed_m_s,
                                                      double periapsis_altitude_m) noexcept {
    const PlanetConstants c = planet_constants(planet);
    const double r = c.radius_m + altitude_m;
    const double rp = c.radius_m + periapsis_altitude_m;
    if (!positive(r) || !positive(rp) || !positive(speed_m_s) || rp > r) {
        return std::nullopt;
    }
    // Energy and angular momentum: vp^2 = v^2 + 2 mu (1/rp - 1/r),
    // r v cos(gamma) = rp vp.
    const double vp2 = speed_m_s * speed_m_s + 2.0 * c.mu_m3_s2 * (1.0 / rp - 1.0 / r);
    if (!(vp2 > 0.0)) {
        return std::nullopt;
    }
    const double cosine = rp * std::sqrt(vp2) / (r * speed_m_s);
    if (!std::isfinite(cosine) || cosine > 1.0 + 1.0e-12) {
        return std::nullopt;
    }
    return -std::acos(std::min(cosine, 1.0)) / kDegree;
}

std::optional<double> periapsis_altitude(EntryPlanet planet, double altitude_m, double speed_m_s,
                                         double flight_path_angle_deg) noexcept {
    const PlanetConstants c = planet_constants(planet);
    const double r = c.radius_m + altitude_m;
    const double gamma = flight_path_angle_deg * kDegree;
    if (!positive(r) || !positive(speed_m_s) || !std::isfinite(gamma)) {
        return std::nullopt;
    }
    const double h = r * speed_m_s * std::cos(gamma); // specific angular momentum
    const double energy = 0.5 * speed_m_s * speed_m_s - c.mu_m3_s2 / r;
    const double p = h * h / c.mu_m3_s2;
    const double e2 = 1.0 + 2.0 * energy * h * h / (c.mu_m3_s2 * c.mu_m3_s2);
    if (!(h > 0.0) || !std::isfinite(e2)) {
        return std::nullopt;
    }
    const double rp = p / (1.0 + std::sqrt(std::max(e2, 0.0)));
    return rp - c.radius_m;
}

std::optional<double> effective_flight_path_angle(const EntrySetup& setup) noexcept {
    if (setup.start == EntryStart::periapsis) {
        return flight_path_angle_for_periapsis(setup.planet, setup.altitude_m, setup.speed_m_s,
                                               setup.periapsis_altitude_m);
    }
    if (!std::isfinite(setup.flight_path_angle_deg)) {
        return std::nullopt;
    }
    return setup.flight_path_angle_deg;
}

std::vector<std::string> setup_errors(const EntrySetup& s) {
    std::vector<std::string> errors;
    add_if(errors, !positive(s.mass_kg), "Mass must be positive");
    add_if(errors, !positive(s.reference_area_m2), "Reference area must be positive");
    add_if(errors, !positive(s.nose_radius_m), "Nose radius must be positive");
    cd_table_errors(s, errors);
    add_if(errors, !std::isfinite(s.altitude_m) || s.altitude_m <= 0.0,
           "Start altitude must be above the ground");
    add_if(errors, !positive(s.speed_m_s), "Speed must be positive");
    if (s.atmosphere == EntryAtmosphere::exponential) {
        add_if(errors, !positive(s.exponential_density_kg_m3),
               "Surface density of the exponential atmosphere must be positive");
        add_if(errors, !positive(s.exponential_scale_height_m),
               "Scale height of the exponential atmosphere must be positive");
    }
    add_if(errors, !positive(s.max_time_s), "Time limit must be positive");
    add_if(errors,
           !std::isfinite(s.latitude_deg) || std::abs(s.latitude_deg) > 90.0 ||
               !std::isfinite(s.longitude_deg) || !std::isfinite(s.heading_deg),
           "Latitude must be within +-90 deg; longitude and heading finite");
    if (s.start == EntryStart::state) {
        add_if(errors,
               !std::isfinite(s.flight_path_angle_deg) || std::abs(s.flight_path_angle_deg) >= 90.0,
               "Flight-path angle must be within +-90 deg");
    } else if (positive(s.speed_m_s) && std::isfinite(s.altitude_m) &&
               !effective_flight_path_angle(s)) {
        errors.emplace_back("No orbit through the start state has this periapsis: lower it "
                            "below the start altitude or raise the speed");
    }
    return errors;
}

double ballistic_coefficient(const EntrySetup& setup) noexcept {
    double cd = 0.0;
    for (const MachCd& point : setup.cd_table) {
        if (std::isfinite(point.cd)) {
            cd = std::max(cd, point.cd);
        }
    }
    const double denominator = cd * setup.reference_area_m2;
    if (!positive(denominator) || !positive(setup.mass_kg)) {
        return 0.0;
    }
    return setup.mass_kg / denominator;
}

std::vector<EntryPreset> entry_presets() {
    // Apollo command module: 3.91 m heat shield (12.02 m^2), 4.69 m spherical
    // radius, about 5.5 t at entry; flown here without lift (ADR-012), so the
    // angle sits near the shallow edge of the lunar-return corridor, where the
    // peak load stays in the 6-8 g band.
    EntrySetup apollo;
    apollo.name = "Apollo-like entry";
    apollo.speed_m_s = 11'000.0;
    apollo.flight_path_angle_deg = -5.75;
    apollo.latitude_deg = -12.0;
    apollo.longitude_deg = 150.0;
    apollo.heading_deg = 60.0;

    EntrySetup leo = apollo;
    leo.name = "Low Earth orbit return";
    leo.speed_m_s = 7'800.0;
    leo.flight_path_angle_deg = -1.5;

    EntrySetup skip = apollo;
    skip.name = "Grazing lunar return (skip-out)";
    skip.flight_path_angle_deg = -5.0;

    // Mars Science Laboratory: 4.5 m 70-degree sphere-cone, 1.125 m nose
    // radius, about 3.3 t, 5.8 km/s at the entry interface.
    EntrySetup mars;
    mars.name = "Mars entry (MSL-like)";
    mars.planet = EntryPlanet::mars;
    mars.mass_kg = 3'300.0;
    mars.reference_area_m2 = 15.9;
    mars.nose_radius_m = 1.125;
    mars.cd_table = {{0.5, 1.00}, {1.5, 1.40}, {3.0, 1.60}, {10.0, 1.68}, {30.0, 1.68}};
    mars.altitude_m = 125'000.0;
    mars.speed_m_s = 5'800.0;
    mars.flight_path_angle_deg = -15.5;
    mars.latitude_deg = -4.6;
    mars.longitude_deg = 137.4;
    mars.heading_deg = 100.0;
    return {{"Apollo-like lunar return", apollo},
            {"Low Earth orbit return", leo},
            {"Grazing lunar return (skip-out)", skip},
            {"Mars entry (MSL-like)", mars}};
}

} // namespace orrery::sim
