#include "orrery/sim/aero_runner.h"

#include <algorithm>
#include <chrono>
#include <exception>
#include <utility>
#include <variant>

#include <tbb/concurrent_queue.h>

namespace orrery::sim {

namespace {

// Longest wait for a command or a frame while nothing runs back to back.
constexpr std::chrono::milliseconds kIdleWait{50};

using Clock = std::chrono::steady_clock;

} // namespace

std::string_view to_string(AeroRunState state) noexcept {
    switch (state) {
        case AeroRunState::idle:
            return "idle";
        case AeroRunState::setting_up:
            return "setting up";
        case AeroRunState::running:
            return "running";
        case AeroRunState::paused:
            return "paused";
        case AeroRunState::finished:
            return "finished";
        case AeroRunState::failed:
            return "failed";
    }
    return "unknown";
}

struct AeroRunner::Queue {
    tbb::concurrent_queue<AeroCommand> commands;
};

struct AeroRunner::Run {
    std::shared_ptr<const AeroRunSpec> spec;
    std::unique_ptr<aero::IAeroSolver> solver;
    SolverPlan plan;
    FlowSummary flow;
    aero::ReferenceValues reference;
};

AeroRunner::AeroRunner()
    : queue_(std::make_unique<Queue>()), progress_(AeroProgress{}),
      thread_([this](const std::stop_token& stop) { loop(stop); }) {}

AeroRunner::~AeroRunner() {
    thread_.request_stop();
    wake_.release();
    thread_.join();
}

void AeroRunner::send(AeroCommand command) {
    const bool control = std::holds_alternative<StartAeroRun>(command) ||
                         std::holds_alternative<SetAeroPaused>(command) ||
                         std::holds_alternative<StopAeroRun>(command);
    queue_->commands.push(std::move(command));
    if (control) {
        controls_pending_.fetch_add(1, std::memory_order_acq_rel);
    }
    wake_.release();
}

void AeroRunner::wake() {
    frames_pending_.fetch_add(1, std::memory_order_acq_rel);
    wake_.release();
}

bool AeroRunner::fetch() noexcept {
    return progress_.fetch();
}

void AeroRunner::loop(const std::stop_token& stop) {
    while (!stop.stop_requested()) {
        const bool back_to_back =
            run_ && status_.state == AeroRunState::running && status_.to_convergence;
        if (!back_to_back) {
            (void)wake_.try_acquire_for(kIdleWait);
        }
        while (wake_.try_acquire()) {
        }
        if (stop.stop_requested()) {
            break;
        }
        try {
            bool changed = false;
            AeroCommand command;
            controls_pending_.store(0, std::memory_order_release);
            while (!stop.stop_requested() && queue_->commands.try_pop(command)) {
                changed = apply(command, stop) || changed;
            }
            const std::uint32_t frames = frames_pending_.exchange(0, std::memory_order_acq_rel);
            if (run_ && status_.state == AeroRunState::running) {
                const bool direct = run_->plan.kind != aero::SolverKind::lattice_boltzmann;
                if (status_.to_convergence || direct) {
                    (void)step_once();
                    publish(true);
                    changed = false;
                } else if (frames > 0) {
                    run_batch(calls_per_frame(), stop);
                    publish(true);
                    changed = false;
                }
            }
            if (changed) {
                publish(false);
            }
        } catch (const std::exception& error) {
            fail(std::string("solver thread error: ") + error.what());
        } catch (...) {
            // Nothing may leave the thread (std::terminate handlers are per
            // thread with the MSVC runtime: the report would be lost).
            fail("solver thread error: non-standard exception");
        }
    }
}

bool AeroRunner::apply(AeroCommand& command, const std::stop_token& stop) {
    if (auto* start_run = std::get_if<StartAeroRun>(&command)) {
        start(*start_run, stop);
        return false; // start() publishes itself
    }
    if (const auto* paused = std::get_if<SetAeroPaused>(&command)) {
        if (run_ &&
            (status_.state == AeroRunState::running || status_.state == AeroRunState::paused)) {
            status_.state = paused->paused ? AeroRunState::paused : AeroRunState::running;
        }
        return true;
    }
    if (const auto* steps = std::get_if<SetAeroStepsPerFrame>(&command)) {
        status_.steps_per_frame = std::max<std::int64_t>(1, steps->steps);
        return true;
    }
    if (const auto* mode = std::get_if<SetAeroToConvergence>(&command)) {
        status_.to_convergence = mode->enabled;
        return true;
    }
    if (std::holds_alternative<StepAeroFrame>(command)) {
        if (run_ &&
            (status_.state == AeroRunState::running || status_.state == AeroRunState::paused)) {
            run_batch(calls_per_frame(), stop);
            if (status_.state == AeroRunState::running) {
                status_.state = AeroRunState::paused;
            }
            publish(true);
        }
        return false;
    }
    if (std::holds_alternative<StopAeroRun>(command)) {
        run_.reset();
        status_.state = AeroRunState::idle;
        status_.message.clear();
        return true;
    }
    return false;
}

void AeroRunner::start(StartAeroRun& start_run, const std::stop_token& stop) {
    run_.reset();
    ++status_.run;
    status_.to_convergence = start_run.to_convergence;
    status_.steps_per_frame =
        std::max<std::int64_t>(1, start_run.spec.experiment.solver.steps_per_frame);
    status_.message.clear();
    status_.step_ms = 0.0;
    status_.steps_per_second = 0.0;

    auto run = std::make_unique<Run>();
    run->spec = std::make_shared<const AeroRunSpec>(std::move(start_run.spec));
    const AeroRunSpec& spec = *run->spec;
    if (!spec.geometry) {
        fail("no geometry to solve");
        return;
    }
    run->flow = spec.flow();
    run->plan = spec.plan();
    run->reference = reference_values(*spec.geometry, spec.experiment.solver);
    status_.solver = run->plan.kind;
    status_.two_dimensional = run->plan.two_dimensional;
    published_.result.reset();
    published_.diagnostics.reset();
    published_.surface_cp.reset();
    published_.flow_field.reset();
    ++published_.view_revision;
    published_.spec = run->spec;
    published_.flow = run->flow;
    published_.reference = run->reference;
    if (!run->flow.runnable()) {
        fail("flow conditions: " + run->flow.input_errors.front());
        return;
    }
    status_.state = AeroRunState::setting_up;
    publish(false);
    if (stop.stop_requested()) {
        return;
    }
    try {
        run->solver = make_solver(run->plan, spec.experiment.solver);
        run->solver->setup(to_aero_geometry(*spec.geometry, spec.experiment.solver),
                           run->flow.flow);
    } catch (const std::exception& error) {
        fail(std::string(solver_display_name(run->plan.kind)) + " setup failed: " + error.what());
        return;
    }
    run_ = std::move(run);
    status_.state = AeroRunState::running;
    publish(true);
}

std::int64_t AeroRunner::calls_per_frame() const {
    if (!run_ || run_->plan.kind != aero::SolverKind::lattice_boltzmann) {
        return 1;
    }
    const auto granularity =
        static_cast<std::int64_t>(lbm_steps_per_call(run_->plan.two_dimensional));
    const std::int64_t steps = status_.steps_per_frame;
    return std::max<std::int64_t>(1, steps / granularity + (steps % granularity != 0 ? 1 : 0));
}

void AeroRunner::run_batch(std::int64_t calls, const std::stop_token& stop) {
    for (std::int64_t call = 0; call < calls && !stop.stop_requested(); ++call) {
        if (!step_once() || controls_pending_.load(std::memory_order_acquire) != 0) {
            break;
        }
    }
}

bool AeroRunner::step_once() {
    if (!run_ || !run_->solver) {
        return false;
    }
    const std::uint64_t before = lattice_steps(*run_->solver);
    const Clock::time_point start = Clock::now();
    bool finished = false;
    try {
        finished = run_->solver->step();
    } catch (const std::exception& error) {
        fail(std::string(solver_display_name(run_->plan.kind)) + " step failed: " + error.what());
        return false;
    }
    const double seconds = std::chrono::duration<double>(Clock::now() - start).count();
    step_calls_.fetch_add(1, std::memory_order_acq_rel);
    status_.step_ms = seconds * 1000.0;
    const std::uint64_t advanced = lattice_steps(*run_->solver) - before;
    status_.steps_per_second = seconds > 0.0 ? static_cast<double>(advanced) / seconds : 0.0;
    if (finished) {
        status_.state = AeroRunState::finished;
        return false;
    }
    return true;
}

void AeroRunner::publish(bool with_result) {
    if (with_result && run_ && run_->solver) {
        published_.result = std::make_shared<const aero::AeroResult>(run_->solver->result());
        published_.diagnostics = std::make_shared<const RunDiagnostics>(
            diagnostics(*run_->solver, run_->spec->experiment.solver));
        refresh_view(status_.state != AeroRunState::running ||
                     status_.run != published_.status.run);
    }
    published_.status = status_;
    progress_.publish(published_);
}

void AeroRunner::refresh_view(bool force) {
    const Clock::time_point now = Clock::now();
    if (!force && now - last_view_ < std::chrono::duration<double>(kViewRefreshSeconds)) {
        return;
    }
    last_view_ = now;
    const PreparedGeometry& geometry = *run_->spec->geometry;
    const aero::AeroResult& result = run_->solver->result();
    published_.surface_cp = std::make_shared<const std::vector<double>>(
        geometry.two_dimensional ? panel_cp(geometry.section, result.surface)
                                 : triangle_cp(geometry.mesh, result.surface));
    if (run_->plan.kind == aero::SolverKind::lattice_boltzmann) {
        auto flow = capture_flow(*run_->solver, geometry, run_->flow.flow, run_->reference);
        if (flow) {
            published_.flow_field = std::make_shared<const FlowSnapshot>(std::move(flow).value());
        }
    } else {
        published_.flow_field.reset();
    }
    ++published_.view_revision;
}

void AeroRunner::fail(const std::string& what) {
    status_.state = AeroRunState::failed;
    status_.message = what;
    if (run_) {
        run_->solver.reset();
    }
    run_.reset();
    publish(false);
}

} // namespace orrery::sim
