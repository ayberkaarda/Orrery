#include "orrery/sim/entry_runner.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <utility>

#include <tbb/concurrent_queue.h>

namespace orrery::sim {

namespace {

// Longest wait for a command while nothing runs.
constexpr std::chrono::milliseconds kIdleWait{50};
// Longest wait between paced advances while running (a frame normally
// wakes the thread sooner).
constexpr std::chrono::milliseconds kPacedWait{8};
// Wall-clock length of one batch of slices before progress is published.
constexpr std::chrono::milliseconds kBatchWall{12};
// A paced run never catches up more than this much wall time at once (after
// a stall, a pause or a debugger break).
constexpr double kMaxCatchUpSeconds = 0.25;

} // namespace

std::string_view to_string(EntryRunState state) noexcept {
    switch (state) {
        case EntryRunState::idle:
            return "idle";
        case EntryRunState::running:
            return "running";
        case EntryRunState::paused:
            return "paused";
        case EntryRunState::finished:
            return "finished";
        case EntryRunState::stopped:
            return "stopped";
        case EntryRunState::failed:
            return "failed";
    }
    return "unknown";
}

struct EntryRunner::Queue {
    tbb::concurrent_queue<EntryCommand> commands;
};

struct EntryRunner::Run {
    std::unique_ptr<EntryStepper> stepper;
    EntrySeries series;
};

EntryRunner::EntryRunner(EntryStepperFactory factory)
    : factory_(std::move(factory)), queue_(std::make_unique<Queue>()), progress_(EntryProgress{}),
      thread_([this](const std::stop_token& stop) { loop(stop); }) {}

EntryRunner::~EntryRunner() {
    thread_.request_stop();
    wake_.release();
    thread_.join();
}

void EntryRunner::send(EntryCommand command) {
    const bool control = !std::holds_alternative<SetEntryPace>(command);
    queue_->commands.push(std::move(command));
    if (control) {
        controls_pending_.fetch_add(1, std::memory_order_acq_rel);
    }
    wake_.release();
}

void EntryRunner::wake() {
    wake_.release();
}

bool EntryRunner::fetch() noexcept {
    return progress_.fetch();
}

void EntryRunner::loop(const std::stop_token& stop) {
    while (!stop.stop_requested()) {
        const bool running = run_ && status_.state == EntryRunState::running;
        if (!running) {
            (void)wake_.try_acquire_for(kIdleWait);
        } else if (status_.pace > 0.0) {
            (void)wake_.try_acquire_for(kPacedWait);
        }
        while (wake_.try_acquire()) {
        }
        if (stop.stop_requested()) {
            break;
        }
        try {
            bool changed = false;
            EntryCommand command;
            controls_pending_.store(0, std::memory_order_release);
            while (!stop.stop_requested() && queue_->commands.try_pop(command)) {
                changed = apply(command) || changed;
            }
            if (run_ && status_.state == EntryRunState::running) {
                advance(stop);
            } else if (changed) {
                publish(run_ != nullptr);
            }
        } catch (const std::exception& error) {
            fail(std::string("entry solver thread error: ") + error.what());
        }
    }
}

bool EntryRunner::apply(EntryCommand& command) {
    if (auto* start_run = std::get_if<StartEntryRun>(&command)) {
        start(*start_run);
        return false; // start() publishes itself
    }
    if (const auto* paused = std::get_if<SetEntryPaused>(&command)) {
        if (run_ &&
            (status_.state == EntryRunState::running || status_.state == EntryRunState::paused)) {
            status_.state = paused->paused ? EntryRunState::paused : EntryRunState::running;
            last_advance_ = Clock::now();
        }
        return true;
    }
    if (const auto* pace = std::get_if<SetEntryPace>(&command)) {
        status_.pace = std::isfinite(pace->pace) ? std::max(pace->pace, 0.0) : 0.0;
        last_advance_ = Clock::now();
        return true;
    }
    if (std::holds_alternative<StopEntryRun>(command)) {
        if (run_ &&
            (status_.state == EntryRunState::running || status_.state == EntryRunState::paused)) {
            status_.state = EntryRunState::stopped;
            publish(true); // everything integrated so far stays shown
        }
        run_.reset();
        return false;
    }
    return false;
}

void EntryRunner::start(StartEntryRun& start_run) {
    run_.reset();
    ++status_.run;
    status_.message.clear();
    status_.sim_time_s = 0.0;
    published_.setup = std::make_shared<const EntrySetup>(std::move(start_run.setup));
    published_.trace.reset();
    ++published_.revision;
    auto made = factory_(*published_.setup);
    if (!made) {
        fail("entry setup: " + made.error());
        return;
    }
    run_ = std::make_unique<Run>();
    run_->stepper = std::move(made).value();
    status_.state = EntryRunState::running;
    last_advance_ = Clock::now();
    publish(false);
}

void EntryRunner::advance(const std::stop_token& stop) {
    const Clock::time_point now = Clock::now();
    const bool paced = status_.pace > 0.0;
    double remaining = std::numeric_limits<double>::infinity();
    if (paced) {
        const double wall = std::clamp(std::chrono::duration<double>(now - last_advance_).count(),
                                       0.0, kMaxCatchUpSeconds);
        remaining = status_.pace * wall;
    }
    last_advance_ = now;
    const Clock::time_point deadline = now + kBatchWall;
    const EntryRunState before = status_.state;
    while (!stop.stop_requested() && controls_pending_.load(std::memory_order_acquire) == 0 &&
           remaining > 0.0) {
        const double seconds = std::min(kEntrySliceSeconds, remaining);
        if (!slice(seconds)) {
            break;
        }
        remaining -= seconds;
        if (Clock::now() >= deadline) {
            break;
        }
    }
    const bool due =
        std::chrono::duration<double>(Clock::now() - last_publish_).count() >= kEntryPublishSeconds;
    if (status_.state != before || due) {
        publish(true);
    }
}

bool EntryRunner::slice(double seconds) {
    if (!run_ || !run_->stepper) {
        return false;
    }
    bool finished = false;
    try {
        finished = run_->stepper->advance(seconds, run_->series);
    } catch (const std::exception& error) {
        fail(std::string("entry integration failed: ") + error.what());
        return false;
    }
    slices_.fetch_add(1, std::memory_order_acq_rel);
    status_.sim_time_s = run_->stepper->time_s();
    if (!run_->series.consistent()) {
        fail("entry integration failed: the stepper returned series of different lengths");
        return false;
    }
    if (finished) {
        status_.state = EntryRunState::finished;
        return false;
    }
    return true;
}

void EntryRunner::publish(bool with_trace) {
    if (with_trace && run_ && run_->stepper) {
        auto trace = std::make_shared<EntryTrace>();
        trace->series = run_->series;
        trace->summary = summarize(trace->series, run_->stepper->outcome());
        published_.trace = std::move(trace);
        ++published_.revision;
    }
    if (status_.state == EntryRunState::finished) {
        run_.reset(); // the stepper is done; its trace is published
    }
    published_.status = status_;
    progress_.publish(published_);
    last_publish_ = Clock::now();
}

void EntryRunner::fail(const std::string& what) {
    status_.state = EntryRunState::failed;
    status_.message = what;
    if (run_ && !run_->series.empty()) {
        // Keep what was integrated before the failure visible.
        auto trace = std::make_shared<EntryTrace>();
        trace->series = run_->series;
        trace->summary = summarize(trace->series, EntryOutcome::none);
        published_.trace = std::move(trace);
        ++published_.revision;
    }
    run_.reset();
    published_.status = status_;
    progress_.publish(published_);
    last_publish_ = Clock::now();
}

} // namespace orrery::sim
