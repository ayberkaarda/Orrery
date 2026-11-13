#include "orrery/atmo/entry.h"

#include <cmath>
#include <memory>
#include <numbers>
#include <utility>

#include "entry_internal.h"

namespace orrery::atmo {

namespace {
constexpr double kDegree = 180.0 / std::numbers::pi;
} // namespace

std::string_view to_string(EntryIntegrator integrator) noexcept {
    switch (integrator) {
        case EntryIntegrator::ias15:
            return "ias15";
        case EntryIntegrator::yoshida4:
            return "yoshida4";
        case EntryIntegrator::leapfrog_kdk:
            return "leapfrog_kdk";
    }
    return "unknown";
}

std::string_view to_string(EntryOutcome outcome) noexcept {
    switch (outcome) {
        case EntryOutcome::running:
            return "running";
        case EntryOutcome::impact:
            return "impact";
        case EntryOutcome::skip_out:
            return "skip_out";
        case EntryOutcome::time_limit:
            return "time_limit";
        case EntryOutcome::failed:
            return "failed";
    }
    return "unknown";
}

std::string_view event_name(const EntryEvent& event) noexcept {
    if (std::holds_alternative<AtmosphericEntryEvent>(event)) {
        return "atmospheric_entry";
    }
    if (std::holds_alternative<SkipOutEvent>(event)) {
        return "skip_out";
    }
    return "impact";
}

GroundPoint ground_point(const Planet& planet, const core::Vec3<core::si::Length>& position,
                         core::si::Time time) noexcept {
    const double x = position.x.value();
    const double y = position.y.value();
    const double z = position.z.value();
    GroundPoint g;
    g.latitude_deg = std::atan2(z, std::hypot(x, y)) * kDegree;
    const double longitude = std::atan2(y, x) - planet.rotation_rate.value() * time.value();
    g.longitude_deg = core::wrap_pi(core::Angle::from_radians(longitude)).degrees();
    return g;
}

EntryExperiment::EntryExperiment(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}
EntryExperiment::EntryExperiment(EntryExperiment&&) noexcept = default;
EntryExperiment& EntryExperiment::operator=(EntryExperiment&&) noexcept = default;
EntryExperiment::~EntryExperiment() = default;

core::Result<EntryExperiment, AtmoError> EntryExperiment::create(EntryConfig config) {
    auto impl = Impl::create(std::move(config));
    if (!impl) {
        return core::failure(impl.error());
    }
    return EntryExperiment(std::move(impl).value());
}

core::Result<EntryOutcome, AtmoError> EntryExperiment::advance(core::si::Time duration) {
    return impl_->advance(duration.value());
}

core::Result<EntryOutcome, AtmoError> EntryExperiment::run() {
    return impl_->advance(impl_->config.max_duration.value());
}

bool EntryExperiment::finished() const noexcept {
    return impl_->summary.outcome != EntryOutcome::running;
}

EntryOutcome EntryExperiment::outcome() const noexcept {
    return impl_->summary.outcome;
}

core::si::Time EntryExperiment::time() const noexcept {
    return core::si::Time(impl_->time_s);
}

const EntryConfig& EntryExperiment::config() const noexcept {
    return impl_->config;
}

const core::SimState& EntryExperiment::state() const noexcept {
    return impl_->state;
}

const AtmosphericDrag& EntryExperiment::drag() const noexcept {
    return impl_->drag;
}

const EntrySeries& EntryExperiment::series() const noexcept {
    return impl_->series;
}

std::span<const EntryEvent> EntryExperiment::events() const noexcept {
    return impl_->events;
}

const EntrySummary& EntryExperiment::summary() const noexcept {
    return impl_->summary;
}

EntryEnergyBalance EntryExperiment::energy_balance() const noexcept {
    return impl_->energy_balance();
}

} // namespace orrery::atmo
