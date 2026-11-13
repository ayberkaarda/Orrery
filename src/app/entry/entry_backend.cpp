#include "orrery/sim/entry_backend.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace orrery::sim {

namespace {

// Interim stepper used until the orrery_atmo entry API is wired in: a
// three-degree-of-freedom point mass over a spherical, non-rotating planet
// (RK4, fixed 0.05 s step), drag 0.5 rho v^2 Cd(M) A, a layered exponential
// fit of the standard atmospheres, and the Sutton-Graves stagnation heat flux.

constexpr double kDegree = std::numbers::pi / 180.0;
constexpr double kG0 = 9.80665;
constexpr double kStep = 0.05;

struct Vec {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};
Vec operator+(Vec a, Vec b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
Vec operator*(double s, Vec a) {
    return {s * a.x, s * a.y, s * a.z};
}
double dot(Vec a, Vec b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
double norm(Vec a) {
    return std::sqrt(dot(a, a));
}

struct Air {
    double density = 0.0;
    double sound_speed = 1.0;
};

// Base altitude [m], density [kg/m^3], scale height [m], temperature [K].
struct Layer {
    double base;
    double density;
    double scale_height;
    double temperature;
};
constexpr std::array<Layer, 8> kEarth{{
    {0.0, 1.225, 8'440.0, 288.15},
    {11'000.0, 0.3639, 6'340.0, 216.65},
    {20'000.0, 0.0880, 6'680.0, 216.65},
    {32'000.0, 0.01322, 7'190.0, 228.65},
    {47'000.0, 0.001427, 7'800.0, 270.65},
    {51'000.0, 8.6e-4, 7'260.0, 270.65},
    {71'000.0, 6.42e-5, 5'800.0, 214.65},
    {86'000.0, 6.96e-6, 6'550.0, 186.87},
}};

Air air(const EntrySetup& s, double h) {
    h = std::max(h, 0.0);
    if (s.atmosphere == EntryAtmosphere::exponential) {
        const double t = s.planet == EntryPlanet::mars ? 210.0 : 250.0;
        const double gas = s.planet == EntryPlanet::mars ? 1.29 * 191.8 : 1.4 * 287.05;
        return {s.exponential_density_kg_m3 * std::exp(-h / s.exponential_scale_height_m),
                std::sqrt(gas * t)};
    }
    if (s.planet == EntryPlanet::mars) {
        return {0.020 * std::exp(-h / 11'100.0), std::sqrt(1.29 * 191.8 * 210.0)};
    }
    const Layer* layer = kEarth.data();
    for (const Layer& l : kEarth) {
        if (h >= l.base) {
            layer = &l;
        }
    }
    return {layer->density * std::exp(-(h - layer->base) / layer->scale_height),
            std::sqrt(1.4 * 287.05 * layer->temperature)};
}

double cd_at(const std::vector<MachCd>& table, double mach) {
    if (mach <= table.front().mach) {
        return table.front().cd;
    }
    for (std::size_t i = 1; i < table.size(); ++i) {
        if (mach <= table[i].mach) {
            const double f = (mach - table[i - 1].mach) / (table[i].mach - table[i - 1].mach);
            return table[i - 1].cd + f * (table[i].cd - table[i - 1].cd);
        }
    }
    return table.back().cd;
}

class InterimStepper final : public EntryStepper {
public:
    explicit InterimStepper(EntrySetup setup, double gamma)
        : setup_(std::move(setup)), planet_(planet_constants(setup_.planet)) {
        const double lat = setup_.latitude_deg * kDegree;
        const double lon = setup_.longitude_deg * kDegree;
        const double heading = setup_.heading_deg * kDegree;
        const Vec up{std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
        const Vec east{-std::sin(lon), std::cos(lon), 0.0};
        const Vec north{-std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon),
                        std::cos(lat)};
        start_dir_ = up;
        r_ = (planet_.radius_m + setup_.altitude_m) * up;
        const double v = setup_.speed_m_s;
        const double horizontal = v * std::cos(gamma * kDegree);
        v_ = horizontal * std::cos(heading) * north + horizontal * std::sin(heading) * east +
             v * std::sin(gamma * kDegree) * up;
        entered_ = setup_.altitude_m <= planet_.atmosphere_top_m;
    }

    bool advance(double seconds, EntrySeries& out) override {
        if (!recorded_first_) {
            record(out, sample());
            recorded_first_ = true;
        }
        const double stop = std::min(t_ + std::max(seconds, 0.0), setup_.max_time_s);
        while (outcome_ == EntryOutcome::none && t_ < stop - 1e-12) {
            step(std::min(kStep, stop - t_));
            if (t_ >= next_sample_ - 1e-9) {
                record(out, sample());
                next_sample_ += kEntrySampleInterval;
            }
        }
        if (outcome_ == EntryOutcome::none && t_ >= setup_.max_time_s - 1e-9) {
            outcome_ = EntryOutcome::orbiting;
        }
        if (outcome_ != EntryOutcome::none && (out.empty() || out.time_s.back() < t_)) {
            record(out, sample());
        }
        return outcome_ != EntryOutcome::none;
    }

    [[nodiscard]] EntryOutcome outcome() const noexcept override { return outcome_; }
    [[nodiscard]] double time_s() const noexcept override { return t_; }

private:
    [[nodiscard]] Vec acceleration(Vec r, Vec v, double* drag_out = nullptr) const {
        const double radius = norm(r);
        const Vec gravity = (-planet_.mu_m3_s2 / (radius * radius * radius)) * r;
        const Air a = air(setup_, radius - planet_.radius_m);
        const double speed = norm(v);
        const double cd = cd_at(setup_.cd_table, speed / a.sound_speed);
        const double drag =
            0.5 * a.density * speed * speed * cd * setup_.reference_area_m2 / setup_.mass_kg;
        if (drag_out != nullptr) {
            *drag_out = drag;
        }
        return speed > 0.0 ? gravity + (-drag / speed) * v : gravity;
    }

    void step(double h) {
        const Vec k1v = acceleration(r_, v_);
        const Vec k1r = v_;
        const Vec k2v = acceleration(r_ + (0.5 * h) * k1r, v_ + (0.5 * h) * k1v);
        const Vec k2r = v_ + (0.5 * h) * k1v;
        const Vec k3v = acceleration(r_ + (0.5 * h) * k2r, v_ + (0.5 * h) * k2v);
        const Vec k3r = v_ + (0.5 * h) * k2v;
        const Vec k4v = acceleration(r_ + h * k3r, v_ + h * k3v);
        const Vec k4r = v_ + h * k3v;
        const Vec r_next = r_ + (h / 6.0) * (k1r + 2.0 * k2r + 2.0 * k3r + k4r);
        const Vec v_next = v_ + (h / 6.0) * (k1v + 2.0 * k2v + 2.0 * k3v + k4v);
        if (!std::isfinite(norm(r_next)) || !std::isfinite(norm(v_next))) {
            throw std::runtime_error("the integration produced a non-finite state");
        }
        const double altitude = norm(r_next) - planet_.radius_m;
        if (altitude <= 0.0) {
            // Land on the surface: shorten the last step linearly.
            const double before = norm(r_) - planet_.radius_m;
            const double f = before > altitude ? before / (before - altitude) : 1.0;
            r_ = r_ + f * (r_next + (-1.0) * r_);
            r_ = (planet_.radius_m / norm(r_)) * r_;
            v_ = v_ + f * (v_next + (-1.0) * v_);
            t_ += f * h;
            outcome_ = EntryOutcome::landed;
            return;
        }
        r_ = r_next;
        v_ = v_next;
        t_ += h;
        entered_ = entered_ || altitude < planet_.atmosphere_top_m;
        if (entered_ && altitude > planet_.atmosphere_top_m && dot(r_, v_) > 0.0) {
            outcome_ = EntryOutcome::skip_out;
        }
    }

    [[nodiscard]] EntrySample sample() const {
        EntrySample s;
        s.time_s = t_;
        const double radius = norm(r_);
        s.altitude_m = std::max(radius - planet_.radius_m, 0.0);
        s.speed_m_s = norm(v_);
        s.flight_path_angle_deg =
            s.speed_m_s > 0.0
                ? std::asin(std::clamp(dot(r_, v_) / (radius * s.speed_m_s), -1.0, 1.0)) / kDegree
                : 0.0;
        double drag = 0.0;
        (void)acceleration(r_, v_, &drag);
        s.g_load = drag / kG0;
        const Air a = air(setup_, s.altitude_m);
        s.dynamic_pressure_pa = 0.5 * a.density * s.speed_m_s * s.speed_m_s;
        const double k = setup_.planet == EntryPlanet::mars ? 1.9027e-4 : 1.7415e-4;
        s.heat_flux_w_m2 = k * std::sqrt(a.density / setup_.nose_radius_m) * s.speed_m_s *
                           s.speed_m_s * s.speed_m_s;
        s.mach = s.speed_m_s / a.sound_speed;
        s.latitude_deg = std::asin(std::clamp(r_.z / radius, -1.0, 1.0)) / kDegree;
        s.longitude_deg = std::atan2(r_.y, r_.x) / kDegree;
        const double c = std::clamp(dot(start_dir_, (1.0 / radius) * r_), -1.0, 1.0);
        s.downrange_m = std::acos(c) * planet_.radius_m;
        return s;
    }

    static void record(EntrySeries& out, const EntrySample& s) { out.push(s); }

    EntrySetup setup_;
    PlanetConstants planet_;
    Vec r_;
    Vec v_;
    Vec start_dir_;
    double t_ = 0.0;
    double next_sample_ = kEntrySampleInterval;
    EntryOutcome outcome_ = EntryOutcome::none;
    bool entered_ = false;
    bool recorded_first_ = false;
};

} // namespace

core::Result<std::unique_ptr<EntryStepper>, std::string>
make_entry_stepper(const EntrySetup& setup) {
    const std::vector<std::string> errors = setup_errors(setup);
    if (!errors.empty()) {
        return core::failure(errors.front());
    }
    const std::optional<double> gamma = effective_flight_path_angle(setup);
    if (!gamma) {
        return core::failure(std::string("no flight-path angle for this start"));
    }
    std::unique_ptr<EntryStepper> stepper = std::make_unique<InterimStepper>(setup, *gamma);
    return stepper;
}

core::Result<EntryRunResult, std::string> run_entry(const EntrySetup& setup) {
    auto made = make_entry_stepper(setup);
    if (!made) {
        return core::failure(std::string(made.error()));
    }
    EntryStepper& stepper = *made.value();
    EntryRunResult result;
    try {
        while (!stepper.advance(60.0, result.series)) {
        }
    } catch (const std::exception& error) {
        return core::failure(std::string(error.what()));
    }
    result.summary = summarize(result.series, stepper.outcome());
    return result;
}

} // namespace orrery::sim
