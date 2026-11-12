#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include "orrery/core/sim_snapshot.h"
#include "orrery/sim/triple_buffer.h"

using orrery::sim::TripleBuffer;

TEST_CASE("Triple buffer hands over the newest published value", "[app][sim][triple_buffer]") {
    TripleBuffer<int> buffer(7);
    CHECK(buffer.read() == 7);
    CHECK_FALSE(buffer.fetch());
    CHECK(buffer.read() == 7);

    buffer.publish(1);
    CHECK(buffer.read() == 7); // nothing changes before fetch
    CHECK(buffer.fetch());
    CHECK(buffer.read() == 1);
    CHECK_FALSE(buffer.fetch());
    CHECK(buffer.read() == 1);

    // Values published between two fetches: only the newest is seen.
    buffer.publish(2);
    buffer.publish(3);
    buffer.publish(4);
    CHECK(buffer.fetch());
    CHECK(buffer.read() == 4);
    CHECK_FALSE(buffer.fetch());

    // The write slot never aliases the slot being read.
    buffer.write_slot() = 99;
    CHECK(buffer.read() == 4);
    buffer.publish();
    CHECK(buffer.fetch());
    CHECK(buffer.read() == 99);
}

TEST_CASE("Triple buffer releases shared snapshots it no longer holds",
          "[app][sim][triple_buffer]") {
    TripleBuffer<std::shared_ptr<const int>> buffer;
    auto first = std::make_shared<const int>(1);
    const std::weak_ptr<const int> watch = first;
    buffer.publish(std::move(first));
    CHECK(buffer.fetch());
    std::shared_ptr<const int> held = buffer.read(); // a reader keeps its own copy
    // Three more publications cycle every slot, so the buffer drops its
    // references to the first value; the reader's copy keeps it alive.
    for (int i = 2; i <= 4; ++i) {
        buffer.publish(std::make_shared<const int>(i));
        CHECK(buffer.fetch());
    }
    CHECK_FALSE(watch.expired());
    held.reset();
    CHECK(watch.expired());
}

namespace {

// Payload whose consistency a torn read would break: every word is derived
// from the sequence number.
struct Payload {
    std::uint64_t sequence = 0;
    std::array<std::uint64_t, 31> words{};
};

Payload make_payload(std::uint64_t sequence) {
    Payload payload;
    payload.sequence = sequence;
    for (std::size_t i = 0; i < payload.words.size(); ++i) {
        payload.words[i] = sequence * (i + 1) ^ 0x9E3779B97F4A7C15ULL;
    }
    return payload;
}

bool consistent(const Payload& payload) {
    for (std::size_t i = 0; i < payload.words.size(); ++i) {
        if (payload.words[i] != (payload.sequence * (i + 1) ^ 0x9E3779B97F4A7C15ULL)) {
            return false;
        }
    }
    return true;
}

} // namespace

// Data-race stress test: one producer writes slots in place while one
// consumer reads them. Every value read must be a complete published value
// and sequence numbers must never go backwards. Under ThreadSanitizer this
// also checks that the exchange orderings make every slot hand-over a
// happens-before edge (no plain access races).
TEST_CASE("Triple buffer stress: no torn or out-of-order reads", "[app][sim][triple_buffer]") {
    constexpr std::uint64_t kValues = 200'000;
    TripleBuffer<Payload> buffer(make_payload(0));

    std::thread producer([&] {
        for (std::uint64_t sequence = 1; sequence <= kValues; ++sequence) {
            buffer.write_slot() = make_payload(sequence);
            buffer.publish();
        }
    });

    // The last value stays fresh until it is fetched, so the consumer ends
    // exactly on it.
    std::uint64_t last = 0;
    std::uint64_t fetched = 0;
    std::uint64_t torn = 0;
    std::uint64_t backwards = 0;
    while (last < kValues) {
        if (buffer.fetch()) {
            ++fetched;
            const Payload& value = buffer.read();
            torn += consistent(value) ? 0U : 1U;
            backwards += value.sequence <= last ? 1U : 0U;
            last = value.sequence;
            if (backwards > 0) {
                break;
            }
        }
    }
    producer.join();

    CHECK(torn == 0);
    CHECK(backwards == 0);
    CHECK(fetched >= 1);
    // The consumer ends on the final value.
    CHECK(last == kValues);
    CHECK(buffer.read().sequence == kValues);
}

TEST_CASE("Triple buffer carries immutable simulation snapshots", "[app][sim][triple_buffer]") {
    orrery::core::SimState state;
    TripleBuffer<std::shared_ptr<const orrery::core::SimSnapshot>> buffer(
        orrery::core::SimSnapshot::capture(state, 0));
    std::thread producer([&] {
        for (std::uint64_t sequence = 1; sequence <= 2000; ++sequence) {
            buffer.publish(orrery::core::SimSnapshot::capture(state, sequence));
        }
    });
    std::uint64_t last = 0;
    bool ordered = true;
    while (last < 2000) {
        if (buffer.fetch()) {
            const std::uint64_t sequence = buffer.read()->sequence();
            ordered = ordered && sequence > last;
            last = sequence;
        }
    }
    producer.join();
    CHECK(ordered);
    CHECK(last == 2000);
}
