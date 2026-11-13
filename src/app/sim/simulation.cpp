#include "orrery/sim/simulation.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <system_error>
#include <utility>
#include <variant>

#include "orrery/core/barnes_hut.h"
#include "orrery/core/body.h"
#include "orrery/core/conservation.h"
#include "orrery/core/event.h"
#include "orrery/core/event_step.h"
#include "orrery/core/force_model.h"
#include "orrery/core/ias15.h"
#include "orrery/core/integrator.h"
#include "orrery/core/paczynski_wiita_gravity.h"
#include "orrery/scene/error.h"
#include "orrery/sim/space_experiment.h"

namespace orrery::sim {

namespace {

constexpr double kNsPerDay = 86400.0e9;

struct IntegratorName {
    SpaceIntegrator kind = SpaceIntegrator::leapfrog_kdk;
    std::string_view name;
};

constexpr std::array<IntegratorName, 4> kIntegratorNames{{
    {SpaceIntegrator::leapfrog_kdk, "leapfrog_kdk"},
    {SpaceIntegrator::yoshida4, "yoshida4"},
    {SpaceIntegrator::rk4, "rk4"},
    {SpaceIntegrator::ias15, "ias15"},
}};

struct ForceName {
    ForceChoice kind = ForceChoice::automatic;
    std::string_view name;
};

constexpr std::array<ForceName, 4> kForceNames{{
    {ForceChoice::automatic, "auto"},
    {ForceChoice::newtonian, "newtonian"},
    {ForceChoice::barnes_hut, "barnes_hut"},
    {ForceChoice::paczynski_wiita, "paczynski_wiita"},
}};

std::unique_ptr<core::IIntegrator> make_space_integrator(SpaceIntegrator kind) {
    switch (kind) {
        case SpaceIntegrator::leapfrog_kdk:
            return core::make_integrator(core::IntegratorKind::leapfrog_kdk);
        case SpaceIntegrator::yoshida4:
            return core::make_integrator(core::IntegratorKind::yoshida4);
        case SpaceIntegrator::rk4:
            return core::make_integrator(core::IntegratorKind::rk4);
        case SpaceIntegrator::ias15:
            return std::make_unique<core::IAS15>();
    }
    return core::make_integrator(core::default_integrator);
}

bool has_black_hole(const core::BodyStore& bodies) {
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        if (core::tag_of(bodies.kind(i)) == core::BodyKindTag::black_hole) {
            return true;
        }
    }
    return false;
}

core::Result<std::unique_ptr<core::IForceModel>, std::string>
make_force_model(ForceChoice choice, double softening_au, const core::BodyStore& bodies) {
    if (choice == ForceChoice::automatic) {
        if (has_black_hole(bodies)) {
            choice = ForceChoice::paczynski_wiita;
        } else if (bodies.size() > kDirectSummationLimit) {
            choice = ForceChoice::barnes_hut;
        } else {
            choice = ForceChoice::newtonian;
        }
    }
    const core::space::Length softening(softening_au);
    switch (choice) {
        case ForceChoice::automatic:
        case ForceChoice::newtonian: {
            auto model = core::NewtonianGravity::create(softening);
            if (!model) {
                return core::failure("Newtonian gravity: " +
                                     std::string(core::to_string(model.error())));
            }
            return std::unique_ptr<core::IForceModel>(
                std::make_unique<core::NewtonianGravity>(std::move(model).value()));
        }
        case ForceChoice::barnes_hut: {
            core::BarnesHutOptions options;
            options.softening = softening;
            auto model = core::BarnesHutGravity::create(options);
            if (!model) {
                return core::failure("Barnes-Hut gravity: " +
                                     std::string(core::to_string(model.error())));
            }
            return std::unique_ptr<core::IForceModel>(
                std::make_unique<core::BarnesHutGravity>(std::move(model).value()));
        }
        case ForceChoice::paczynski_wiita: {
            auto model = core::PaczynskiWiitaGravity::create(softening);
            if (!model) {
                return core::failure("Paczynski-Wiita gravity: " +
                                     std::string(core::to_string(model.error())));
            }
            return std::unique_ptr<core::IForceModel>(
                std::make_unique<core::PaczynskiWiitaGravity>(std::move(model).value()));
        }
    }
    return core::failure(std::string("unknown force model"));
}

std::string days_text(double days) {
    std::array<char, 64> buffer{};
    const auto written = std::to_chars(buffer.data(), buffer.data() + buffer.size(), days,
                                       std::chars_format::fixed, 3);
    if (written.ec != std::errc{}) {
        return "? d";
    }
    return std::string(buffer.data(), written.ptr) + " d";
}

} // namespace

std::string_view to_string(SpaceIntegrator integrator) noexcept {
    for (const IntegratorName& entry : kIntegratorNames) {
        if (entry.kind == integrator) {
            return entry.name;
        }
    }
    return "unknown";
}

std::optional<SpaceIntegrator> integrator_from_string(std::string_view name) noexcept {
    for (const IntegratorName& entry : kIntegratorNames) {
        if (entry.name == name) {
            return entry.kind;
        }
    }
    return std::nullopt;
}

std::string_view to_string(ForceChoice force) noexcept {
    for (const ForceName& entry : kForceNames) {
        if (entry.kind == force) {
            return entry.name;
        }
    }
    return "unknown";
}

std::optional<ForceChoice> force_from_string(std::string_view name) noexcept {
    for (const ForceName& entry : kForceNames) {
        if (entry.name == name) {
            return entry.kind;
        }
    }
    return std::nullopt;
}

std::optional<std::string> validate(const SimulationConfig& config) {
    if (!std::isfinite(config.dt_days) || config.dt_days < kMinStepDays ||
        config.dt_days > kMaxStepDays) {
        return "time step " + std::to_string(config.dt_days) + " d is outside [" +
               std::to_string(kMinStepDays) + ", " + std::to_string(kMaxStepDays) + "] d";
    }
    if (!std::isfinite(config.softening_au) || config.softening_au < 0.0) {
        return std::string("softening must be finite and non-negative");
    }
    if (config.time_warp < kMinTimeWarp || config.time_warp > kMaxTimeWarp) {
        return "time warp " + std::to_string(config.time_warp) + " is outside [" +
               std::to_string(kMinTimeWarp) + ", " + std::to_string(kMaxTimeWarp) + "]";
    }
    if (config.max_steps_per_advance < 1) {
        return std::string("the step budget must allow at least one step per advance");
    }
    if (!std::isfinite(config.step_time_budget_s) || config.step_time_budget_s < 0.0) {
        return std::string("the step time budget must be finite and non-negative");
    }
    if (config.warp_budget) {
        return validate(*config.warp_budget);
    }
    return std::nullopt;
}

struct Simulation::Impl {
    SimulationConfig config;
    scene::Scene initial;
    core::SimState state;
    std::unique_ptr<core::IIntegrator> integrator;
    std::unique_ptr<core::IForceModel> forces;
    std::optional<core::ConservationDiagnostics> diagnostics;
    core::EventStepper stepper;
    core::EventLog log;
    // Simulated nanoseconds of one physics step, and owed by the accumulator.
    std::int64_t step_ns = 1;
    std::int64_t owed_ns = 0;
    SimStatus status;
    // Step count at which status.energy_drift was last evaluated.
    std::optional<std::uint64_t> drift_steps;
    std::optional<WarpGovernor> governor;

    [[nodiscard]] std::int64_t now_ns() const {
        if (config.clock) {
            return config.clock();
        }
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    // The warp credited to the accumulator.
    [[nodiscard]] std::int64_t credited_warp() const noexcept {
        return governor ? governor->effective() : config.time_warp;
    }

    void publish_governor() {
        status.governed_warp = credited_warp();
        status.warp_reduced = governor && governor->reduced();
        status.physics_load = governor ? governor->load() : std::nullopt;
        status.load_limit = governor ? governor->budget().load_limit : 0.0;
    }

    // Applies the step length and warp of `config` (after create or a loaded
    // experiment).
    void apply_timing() {
        step_ns = std::max<std::int64_t>(
            1, static_cast<std::int64_t>(std::llround(config.dt_days * kNsPerDay)));
        status.dt_days = config.dt_days;
        status.paused = config.paused;
        status.time_warp = config.time_warp;
        status.settings = RunSettings{config.integrator, config.force, config.softening_au,
                                      config.max_steps_per_advance};
        if (config.warp_budget) {
            governor.emplace(*config.warp_budget, config.time_warp, step_ns);
        }
        publish_governor();
    }

    void load_experiment(const std::filesystem::path& path);

    // Makes `scene` the simulated system (and the target of Reset). Builds a
    // fresh integrator so that adaptive history never leaks across loads.
    std::optional<std::string> start(scene::Scene scene) {
        auto forces_built =
            make_force_model(config.force, config.softening_au, scene.state.bodies());
        if (!forces_built) {
            return forces_built.error();
        }
        std::unique_ptr<core::IForceModel> new_forces = std::move(forces_built).value();
        std::unique_ptr<core::IIntegrator> new_integrator =
            make_space_integrator(config.integrator);
        std::optional<core::ConservationDiagnostics> new_diagnostics;
        if (scene.state.bodies().size() <= kDriftBodyLimit) {
            new_diagnostics.emplace(scene.state, *new_forces);
        }
        core::SimState new_state = scene.state;
        // Commit (no throwing operations below except string copies, which
        // leave the simulation consistent if they fail).
        forces = std::move(new_forces);
        integrator = std::move(new_integrator);
        diagnostics = new_diagnostics;
        state = std::move(new_state);
        initial = std::move(scene);
        log.clear();
        owed_ns = 0;
        drift_steps.reset();
        status.scenario = initial.name;
        status.integrator = std::string(integrator->name());
        status.force_model = std::string(forces->name());
        status.steps = 0;
        status.events = 0;
        status.step_ms.reset();
        status.energy_drift.reset();
        status.effective_warp = 0.0;
        status.warp_limited = false;
        status.message.clear();
        ++status.generation;
        if (governor) {
            // Another system steps at another cost.
            governor->restart(step_ns);
        }
        publish_governor();
        return std::nullopt;
    }

    // One physics step; on failure the state is unchanged (integrator
    // contract) and the simulation pauses with a message.
    bool step_once() {
        const auto stepped =
            stepper.step(*integrator, state, core::space::Time(config.dt_days), *forces, log);
        if (!stepped) {
            status.paused = true;
            owed_ns = 0;
            status.message = "Integration failed at t = " + days_text(state.time().value()) + ": " +
                             std::string(core::to_string(stepped.error())) + "; simulation paused";
            return false;
        }
        status.events += log.size();
        log.clear();
        ++status.steps;
        return true;
    }
};

void Simulation::Impl::load_experiment(const std::filesystem::path& path) {
    auto loaded = load_space_experiment(path);
    if (!loaded) {
        status.message =
            "Cannot open experiment " + path.filename().string() + ": " + std::move(loaded).error();
        return;
    }
    SpaceExperiment experiment = std::move(loaded).value();
    // The experiment brings its own settings; the interactive budgets and
    // the clock belong to the caller and stay.
    SimulationConfig next = experiment.config;
    next.step_time_budget_s = config.step_time_budget_s;
    next.warp_budget = config.warp_budget;
    next.clock = config.clock;
    SimulationConfig previous = std::move(config);
    config = std::move(next);
    if (auto problem = start(std::move(experiment.scene))) {
        config = std::move(previous);
        status.message = "Cannot start experiment " + path.filename().string() + ": " + *problem;
        return;
    }
    owed_ns = 0;
    apply_timing();
    status.scenario_file = experiment.scenario_path;
    status.experiment_file = path;
}

Simulation::Simulation(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}

Simulation::Simulation(Simulation&&) noexcept = default;

Simulation& Simulation::operator=(Simulation&&) noexcept = default;

Simulation::~Simulation() = default;

core::Result<Simulation, std::string> Simulation::create(scene::Scene scene,
                                                         const SimulationConfig& config) {
    if (auto problem = validate(config)) {
        return core::failure(std::move(*problem));
    }
    auto impl = std::make_unique<Impl>();
    impl->config = config;
    impl->apply_timing();
    if (auto problem = impl->start(std::move(scene))) {
        return core::failure(std::move(*problem));
    }
    return Simulation(std::move(impl));
}

void Simulation::apply(const SimCommand& command) {
    Impl& s = *impl_;
    if (const auto* paused = std::get_if<SetPaused>(&command)) {
        s.status.paused = paused->paused;
        s.owed_ns = 0;
        if (!paused->paused) {
            s.status.message.clear();
        }
    } else if (const auto* warp = std::get_if<SetTimeWarp>(&command)) {
        s.config.time_warp = std::clamp(warp->warp, kMinTimeWarp, kMaxTimeWarp);
        s.status.time_warp = s.config.time_warp;
        if (s.governor) {
            s.governor->request(s.config.time_warp);
        }
        s.publish_governor();
    } else if (const auto* load = std::get_if<LoadScenario>(&command)) {
        auto loaded = scene::load_scene_file(load->path);
        if (!loaded) {
            s.status.message = "Cannot load scenario " + load->path.filename().string() + ": " +
                               scene::to_string(loaded.error());
            return;
        }
        if (auto problem = s.start(std::move(loaded).value())) {
            s.status.message =
                "Cannot start scenario " + load->path.filename().string() + ": " + *problem;
            return;
        }
        s.status.scenario_file = load->path;
        s.status.experiment_file.clear();
    } else if (const auto* experiment = std::get_if<LoadExperiment>(&command)) {
        s.load_experiment(experiment->path);
    } else if (std::holds_alternative<Reset>(command)) {
        scene::Scene initial = s.initial;
        if (auto problem = s.start(std::move(initial))) {
            s.status.message = "Cannot reset: " + *problem;
        }
    } else if (std::holds_alternative<SingleStep>(command)) {
        s.status.paused = true;
        s.owed_ns = 0;
        const std::int64_t begin = s.now_ns();
        if (s.step_once()) {
            s.status.step_ms = static_cast<double>(s.now_ns() - begin) * 1.0e-6;
        }
    }
}

AdvanceResult Simulation::advance(std::int64_t wall_ns) {
    Impl& s = *impl_;
    AdvanceResult result;
    if (s.status.paused) {
        s.owed_ns = 0;
        s.status.effective_warp = 0.0;
        s.status.warp_limited = false;
        return result;
    }
    const std::int64_t wall = std::clamp<std::int64_t>(wall_ns, 0, kMaxAdvanceWallNs);
    // At most kMaxAdvanceWallNs * kMaxTimeWarp = 2.5e15 ns per call, plus a
    // remainder below one step (at most kMaxStepDays = 8.64e17 ns): no
    // overflow of the 64-bit accumulator.
    // The warp budget's warp, never the requested one directly: the credit
    // is what the governor allows (the same integral accumulator either way).
    const std::int64_t warp = s.credited_warp();
    s.owed_ns += wall * warp;

    // The clock is read once before the first step and once after each one,
    // so a fake clock that advances per reading charges every step equally.
    const std::int64_t begin = s.now_ns();
    std::int64_t last = begin;
    // Validated finite and non-negative; capped so the conversion is defined.
    const auto budget_ns =
        static_cast<std::int64_t>(std::min(s.config.step_time_budget_s, 1.0e6) * 1.0e9);
    while (s.owed_ns >= s.step_ns) {
        if (result.steps >= s.config.max_steps_per_advance ||
            (budget_ns > 0 && result.steps > 0 && last - begin > budget_ns)) {
            result.warp_limited = true;
            break;
        }
        const bool stepped = s.step_once();
        last = s.now_ns();
        if (!stepped) {
            result.failed = true;
            break;
        }
        s.owed_ns -= s.step_ns;
        ++result.steps;
    }
    if (result.warp_limited) {
        // Drop the simulated time the budget could not cover, keep the
        // fraction of a step (no spiral of ever-growing debt).
        s.owed_ns %= s.step_ns;
    }
    if (result.steps > 0) {
        s.status.step_ms =
            static_cast<double>(last - begin) * 1.0e-6 / static_cast<double>(result.steps);
    }
    if (s.governor && !result.failed) {
        (void)s.governor->update(WarpSample{wall, last - begin, result.steps});
    }
    s.publish_governor();
    if (result.failed) {
        s.status.effective_warp = 0.0;
        s.status.warp_limited = false;
        return result;
    }
    if (wall > 0) {
        // Limited: the rate the budget sustains (steps actually taken per
        // wall-clock time); otherwise every owed step is taken and the
        // credited warp holds on average.
        s.status.effective_warp = result.warp_limited ? static_cast<double>(result.steps) *
                                                            static_cast<double>(s.step_ns) /
                                                            static_cast<double>(wall)
                                                      : static_cast<double>(warp);
        s.status.warp_limited = result.warp_limited || s.status.warp_reduced;
    }
    return result;
}

std::shared_ptr<const core::SimSnapshot> Simulation::snapshot() const {
    return core::SimSnapshot::capture(impl_->state, impl_->status.steps);
}

const SimStatus& Simulation::status() {
    Impl& s = *impl_;
    if (s.diagnostics && s.drift_steps != s.status.steps) {
        s.status.energy_drift = s.diagnostics->drift(s.state, *s.forces).relative_energy;
        s.drift_steps = s.status.steps;
    }
    return s.status;
}

const core::SimState& Simulation::state() const noexcept {
    return impl_->state;
}

const SimulationConfig& Simulation::config() const noexcept {
    return impl_->config;
}

} // namespace orrery::sim
