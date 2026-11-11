#include "orrery/render/log.h"

#include <iostream>
#include <memory>
#include <mutex>
#include <utility>

namespace orrery::render {

namespace {

// The sink is held through a shared pointer: log_message copies the pointer
// under the mutex and calls the sink after releasing it, so a sink may
// replace itself or log again, and a sink replaced during a call stays alive
// until that call returns. stderr_mutex only keeps default-sink lines whole.
struct LogState {
    std::mutex mutex;
    std::shared_ptr<const LogSink> sink;
    std::mutex stderr_mutex;
};

LogState& log_state() {
    static LogState state;
    return state;
}

} // namespace

std::string_view to_string(LogLevel level) noexcept {
    switch (level) {
        case LogLevel::debug:
            return "debug";
        case LogLevel::info:
            return "info";
        case LogLevel::warning:
            return "warning";
        case LogLevel::error:
            return "error";
    }
    return "unknown";
}

LogSink set_log_sink(LogSink sink) {
    LogState& state = log_state();
    std::shared_ptr<const LogSink> replacement =
        sink ? std::make_shared<const LogSink>(std::move(sink)) : nullptr;
    std::shared_ptr<const LogSink> previous;
    {
        const std::scoped_lock lock(state.mutex);
        previous = std::exchange(state.sink, std::move(replacement));
    }
    // Copied outside the lock: the previous sink may still be running on
    // another thread, which holds its own reference.
    return previous ? *previous : LogSink{};
}

void log_message(LogLevel level, std::string_view message) {
    LogState& state = log_state();
    std::shared_ptr<const LogSink> sink;
    {
        const std::scoped_lock lock(state.mutex);
        sink = state.sink;
    }
    if (sink) {
        (*sink)(level, message);
        return;
    }
    const std::scoped_lock lock(state.stderr_mutex);
    std::cerr << "[orrery.render] " << to_string(level) << ": " << message << '\n';
}

} // namespace orrery::render
