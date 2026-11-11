#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <exception>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "orrery/render/log.h"

// The log sink is user code. It may replace the sink or log again (for
// example a forwarding sink, or one called from a synchronous GL debug
// callback), so the logger must not hold its lock while calling it.

using namespace orrery::render;

namespace {

constexpr std::chrono::seconds kTimeout{20};

// Runs `body` on its own thread and reports whether it returned within the
// timeout. A deadlocked body cannot be joined, so the thread is detached; the
// test then fails instead of hanging.
bool returns_in_time(std::function<void()> body) {
    auto done = std::make_shared<std::promise<void>>();
    std::future<void> finished = done->get_future();
    std::thread([done, body = std::move(body)] {
        try {
            body();
            done->set_value();
        } catch (...) {
            done->set_exception(std::current_exception());
        }
    }).detach();
    if (finished.wait_for(kTimeout) != std::future_status::ready) {
        return false;
    }
    finished.get(); // rethrows a failure inside the body
    return true;
}

// Restores the default sink when a test ends.
struct DefaultSinkOnExit {
    DefaultSinkOnExit() = default;
    DefaultSinkOnExit(const DefaultSinkOnExit&) = delete;
    DefaultSinkOnExit& operator=(const DefaultSinkOnExit&) = delete;
    DefaultSinkOnExit(DefaultSinkOnExit&&) = delete;
    DefaultSinkOnExit& operator=(DefaultSinkOnExit&&) = delete;
    ~DefaultSinkOnExit() { (void)set_log_sink({}); }
};

} // namespace

TEST_CASE("A log sink may remove itself while it runs", "[render][log]") {
    const DefaultSinkOnExit restore;
    auto calls = std::make_shared<std::atomic<int>>(0);
    (void)set_log_sink([calls](LogLevel, std::string_view) {
        ++*calls;
        (void)set_log_sink({}); // destroys this sink's stored copy
    });
    REQUIRE(returns_in_time([] { log_message(LogLevel::info, "once"); }));
    CHECK(*calls == 1);
    // The sink is gone: the default sink is installed again.
    CHECK_FALSE(static_cast<bool>(set_log_sink({})));
}

TEST_CASE("A log sink may log again from inside the sink", "[render][log]") {
    const DefaultSinkOnExit restore;
    auto seen = std::make_shared<std::vector<std::string>>();
    (void)set_log_sink([seen](LogLevel, std::string_view message) {
        seen->emplace_back(message);
        if (message == "outer") {
            log_message(LogLevel::warning, "inner");
        }
    });
    REQUIRE(returns_in_time([] { log_message(LogLevel::info, "outer"); }));
    CHECK(*seen == std::vector<std::string>{"outer", "inner"});
}

TEST_CASE("Replacing the sink is safe while other threads log", "[render][log]") {
    const DefaultSinkOnExit restore;
    constexpr int kThreads = 4;
    constexpr int kMessages = 2000;
    auto first = std::make_shared<std::atomic<int>>(0);
    auto second = std::make_shared<std::atomic<int>>(0);
    const LogSink count_first = [first](LogLevel, std::string_view) { ++*first; };
    const LogSink count_second = [second](LogLevel, std::string_view) { ++*second; };
    (void)set_log_sink(count_first);
    const auto hammer = [&] {
        std::vector<std::thread> loggers;
        loggers.reserve(kThreads);
        for (int t = 0; t < kThreads; ++t) {
            loggers.emplace_back([] {
                for (int i = 0; i < kMessages; ++i) {
                    log_message(LogLevel::debug, "concurrent");
                }
            });
        }
        for (int i = 0; i < kMessages; ++i) {
            (void)set_log_sink((i % 2) == 0 ? count_second : count_first);
        }
        for (std::thread& logger : loggers) {
            logger.join();
        }
    };
    REQUIRE(returns_in_time(hammer));
    // Every message reached exactly one of the two sinks.
    CHECK(*first + *second == kThreads * kMessages);
}
