#include "orrery/sim/physics_thread.h"

#include <algorithm>
#include <exception>
#include <string>
#include <utility>
#include <variant>

namespace orrery::sim {

namespace {

PhysicsThreadOptions usable(PhysicsThreadOptions options) {
    options.max_commands_per_tick = std::max<std::size_t>(options.max_commands_per_tick, 1);
    options.max_single_steps_per_tick = std::max<std::size_t>(options.max_single_steps_per_tick, 1);
    return options;
}

PublishedFrame capture(Simulation& simulation) {
    return PublishedFrame{simulation.snapshot(), simulation.status()};
}

} // namespace

PhysicsThread::PhysicsThread(Simulation simulation, PhysicsThreadOptions options)
    : simulation_(std::move(simulation)), options_(usable(options)), frames_(capture(simulation_)),
      published_snapshot_(frames_.read().snapshot), published_status_(frames_.read().status),
      thread_([this](const std::stop_token& stop) { run(stop); }) {}

PhysicsThread::~PhysicsThread() {
    thread_.request_stop();
    wake_.release();
    thread_.join();
}

void PhysicsThread::send(SimCommand command) {
    commands_.push(std::move(command));
    wake_.release();
}

void PhysicsThread::wake() {
    wake_.release();
}

bool PhysicsThread::fetch() noexcept {
    return frames_.fetch();
}

void PhysicsThread::run(const std::stop_token& stop) {
    using Clock = std::chrono::steady_clock;
    Clock::time_point last = Clock::now();
    while (!stop.stop_requested()) {
        // Wait for a wake-up (frame or command) or the idle tick, then
        // consume every pending wake-up so that the count stays bounded.
        (void)wake_.try_acquire_for(options_.idle_tick);
        while (wake_.try_acquire()) {
        }
        if (stop.stop_requested()) {
            break;
        }
        const Clock::time_point now = Clock::now();
        const auto wall = std::chrono::duration_cast<std::chrono::nanoseconds>(now - last);
        last = now;
        try {
            tick(wall.count(), stop);
        } catch (const std::exception& error) {
            // Nothing may escape the thread; pause and report instead.
            report(std::string("Physics thread error: ") + error.what());
        } catch (...) {
            // Also nothing else: std::terminate handlers are per thread with
            // the MSVC runtime, so a report from here would be lost.
            report("Physics thread error: non-standard exception");
        }
        ticks_.fetch_add(1, std::memory_order_acq_rel);
    }
}

void PhysicsThread::report(std::string message) {
    simulation_.apply(SetPaused{true});
    published_status_ = simulation_.status();
    published_status_.message = std::move(message);
    frames_.publish(PublishedFrame{published_snapshot_, published_status_});
}

void PhysicsThread::tick(std::int64_t wall_ns, const std::stop_token& stop) {
    // The wall-clock time since the previous tick elapsed under the state
    // that held then (paused or not, the old warp, the old scenario): credit
    // it before applying the commands that arrived in the meantime, so no
    // command acts retroactively.
    const AdvanceResult advanced = simulation_.advance(wall_ns);

    // Bounded command work per tick: at most max_commands_per_tick commands,
    // of which at most max_single_steps_per_tick single steps (each one is a
    // physics step outside advance()'s budget). The rest waits for the next
    // tick, which is requested at once; cancellation is checked between
    // commands.
    bool commanded = false;
    std::size_t applied = 0;
    std::size_t single_steps = 0;
    while (applied < options_.max_commands_per_tick && !stop.stop_requested()) {
        std::optional<SimCommand> command;
        if (held_back_) {
            command = std::move(held_back_);
            held_back_.reset();
        } else {
            command = commands_.try_pop();
        }
        if (!command) {
            break;
        }
        if (std::holds_alternative<SingleStep>(*command)) {
            if (single_steps >= options_.max_single_steps_per_tick) {
                held_back_ = std::move(command);
                break;
            }
            ++single_steps;
        }
        simulation_.apply(*command);
        commanded = true;
        ++applied;
    }
    if (held_back_ || !commands_.empty()) {
        wake_.release();
    }
    const SimStatus& status = simulation_.status();
    const bool state_changed = commanded || advanced.steps > 0;
    if (!state_changed && status == published_status_) {
        return;
    }
    if (state_changed) {
        published_snapshot_ = simulation_.snapshot();
    }
    published_status_ = status;
    PublishedFrame& slot = frames_.write_slot();
    slot.snapshot = published_snapshot_;
    slot.status = published_status_;
    frames_.publish();
}

} // namespace orrery::sim
