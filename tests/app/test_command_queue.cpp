#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <thread>
#include <variant>
#include <vector>

#include "orrery/sim/command_queue.h"

using namespace orrery::sim;

TEST_CASE("Command queue is FIFO and keeps every command kind", "[app][sim][commands]") {
    CommandQueue queue;
    CHECK(queue.empty());
    CHECK_FALSE(queue.try_pop().has_value());

    queue.push(SetPaused{true});
    queue.push(SetTimeWarp{1000});
    queue.push(LoadScenario{"scenarios/sun_earth.json"});
    queue.push(Reset{});
    queue.push(SingleStep{});
    CHECK_FALSE(queue.empty());

    auto first = queue.try_pop();
    REQUIRE(first.has_value());
    REQUIRE(std::holds_alternative<SetPaused>(*first));
    CHECK(std::get<SetPaused>(*first).paused);
    auto second = queue.try_pop();
    REQUIRE(second.has_value());
    REQUIRE(std::holds_alternative<SetTimeWarp>(*second));
    CHECK(std::get<SetTimeWarp>(*second).warp == 1000);
    auto third = queue.try_pop();
    REQUIRE(third.has_value());
    REQUIRE(std::holds_alternative<LoadScenario>(*third));
    CHECK(std::get<LoadScenario>(*third).path.filename() == "sun_earth.json");
    auto fourth = queue.try_pop();
    REQUIRE(fourth.has_value());
    CHECK(std::holds_alternative<Reset>(*fourth));
    auto fifth = queue.try_pop();
    REQUIRE(fifth.has_value());
    CHECK(std::holds_alternative<SingleStep>(*fifth));
    CHECK_FALSE(queue.try_pop().has_value());
    CHECK(queue.empty());

    // Moving the queue keeps its contents.
    queue.push(Reset{});
    CommandQueue moved = std::move(queue);
    auto kept = moved.try_pop();
    REQUIRE(kept.has_value());
    CHECK(std::holds_alternative<Reset>(*kept));
}

TEST_CASE("Command queue: concurrent producers, one consumer, per-producer order kept",
          "[app][sim][commands]") {
    constexpr std::int64_t kProducers = 4;
    constexpr std::int64_t kPerProducer = 20'000;
    CommandQueue queue;
    std::vector<std::thread> producers;
    for (std::int64_t p = 0; p < kProducers; ++p) {
        producers.emplace_back([&queue, p] {
            for (std::int64_t i = 0; i < kPerProducer; ++i) {
                queue.push(SetTimeWarp{p * kPerProducer + i});
            }
        });
    }
    std::array<std::int64_t, kProducers> next{};
    std::int64_t received = 0;
    bool ordered = true;
    bool other_kind = false;
    while (received < kProducers * kPerProducer) {
        const auto command = queue.try_pop();
        if (!command) {
            std::this_thread::yield();
            continue;
        }
        const auto* warp = std::get_if<SetTimeWarp>(&*command);
        if (warp == nullptr) {
            other_kind = true;
            break;
        }
        const auto producer = static_cast<std::size_t>(warp->warp / kPerProducer);
        const std::int64_t index = warp->warp % kPerProducer;
        ordered = ordered && producer < next.size() && index == next.at(producer);
        if (producer < next.size()) {
            next.at(producer) = index + 1;
        }
        ++received;
    }
    for (std::thread& producer : producers) {
        producer.join();
    }
    CHECK_FALSE(other_kind);
    CHECK(ordered);
    CHECK(received == kProducers * kPerProducer);
    CHECK(queue.empty());
}
