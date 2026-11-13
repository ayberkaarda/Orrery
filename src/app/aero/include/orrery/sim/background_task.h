#pragma once

#include <atomic>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>

// One piece of work on its own thread whose result the user interface polls
// once per frame (docs/aero-workspace.md, "Background work"): model
// downloads, imports and repairs. No frame ever waits for it: done() is one
// atomic load, and the result is handed over by a release store after the
// worker has written it. The work reports its current stage (a string
// literal) for the progress line.
namespace orrery::sim {

// Progress channel of a running task.
class TaskProgress {
public:
    // `stage` must be a string literal (it is kept by pointer).
    void set_stage(const char* stage) noexcept { stage_.store(stage, std::memory_order_release); }
    [[nodiscard]] const char* stage() const noexcept {
        return stage_.load(std::memory_order_acquire);
    }

private:
    std::atomic<const char*> stage_{"starting"};
};

template <typename T> class BackgroundTask {
public:
    BackgroundTask() = default;

    // Starts `work(progress)` on a new thread. An exception escaping the
    // work is turned into `on_exception(message)`, so the result is always
    // set.
    template <typename Work, typename OnException>
    BackgroundTask(Work work, OnException on_exception) : state_(std::make_shared<State>()) {
        std::shared_ptr<State> state = state_;
        thread_ = std::jthread([state, work = std::move(work),
                                on_exception = std::move(on_exception)]() mutable {
            try {
                state->result.emplace(work(state->progress));
            } catch (const std::exception& error) {
                state->result.emplace(on_exception(std::string(error.what())));
            } catch (...) {
                state->result.emplace(on_exception(std::string("unexpected error")));
            }
            state->done.store(true, std::memory_order_release);
        });
    }

    BackgroundTask(BackgroundTask&&) noexcept = default;
    BackgroundTask& operator=(BackgroundTask&&) noexcept = default;
    BackgroundTask(const BackgroundTask&) = delete;
    BackgroundTask& operator=(const BackgroundTask&) = delete;
    // Joins the worker (a finished task joins at once).
    ~BackgroundTask() = default;

    [[nodiscard]] bool started() const noexcept { return state_ != nullptr; }
    [[nodiscard]] bool done() const noexcept {
        return state_ && state_->done.load(std::memory_order_acquire);
    }
    [[nodiscard]] const char* stage() const noexcept {
        return state_ ? state_->progress.stage() : "";
    }

    // The result, once done(); the task is empty afterwards.
    [[nodiscard]] T take() {
        if (thread_.joinable()) {
            thread_.join();
        }
        T value = std::move(*state_->result);
        state_.reset();
        return value;
    }

private:
    struct State {
        TaskProgress progress;
        std::optional<T> result;
        std::atomic<bool> done{false};
    };

    std::shared_ptr<State> state_;
    std::jthread thread_; // last: joined before state_ is released
};

} // namespace orrery::sim
