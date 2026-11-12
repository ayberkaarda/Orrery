#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <utility>

namespace orrery::sim {

// Lock-free single-producer, single-consumer triple buffer (docs/app.md,
// "Physics thread"). Three slots rotate between the roles
//
//   back   - owned by the producer, written in place
//   middle - the most recently published slot, owned by nobody
//   front  - owned by the consumer, read in place
//
// publish() swaps back and middle and marks middle fresh; fetch() swaps
// front and a fresh middle. Both swaps are one atomic exchange on a byte that
// holds the middle index and the fresh flag, so neither side ever waits for
// the other: the producer always has a slot to write and the consumer always
// has a complete value to read (the latest one published before its last
// fetch). Values published while the consumer did not fetch are overwritten;
// the consumer only ever sees the newest.
//
// Memory ordering: the exchange in publish() is a release of the producer's
// writes to its slot and the exchange in fetch() acquires them; in the other
// direction fetch() releases the consumer's reads of the slot it hands back
// before publish() can acquire that slot for writing. No other shared state
// exists, so the type is free of data races by construction (checked by the
// stress test under ThreadSanitizer builds).
//
// Exactly one thread may call write_slot()/publish() and exactly one thread
// may call fetch()/read(); the two may be the same thread. T must be
// default constructible and assignable.
template <typename T> class TripleBuffer {
public:
    TripleBuffer() = default;
    // Every slot starts as a copy of `initial`; nothing is fresh.
    explicit TripleBuffer(const T& initial) : slots_{initial, initial, initial} {}

    TripleBuffer(const TripleBuffer&) = delete;
    TripleBuffer& operator=(const TripleBuffer&) = delete;
    TripleBuffer(TripleBuffer&&) = delete;
    TripleBuffer& operator=(TripleBuffer&&) = delete;
    ~TripleBuffer() = default;

    // Producer: the slot the next value is written into. It holds an older
    // value (whatever was last published into it, or the initial value).
    [[nodiscard]] T& write_slot() noexcept { return slots_[back_]; }

    // Producer: publishes the write slot and takes over the previous middle
    // slot as the new write slot. Never blocks.
    void publish() noexcept {
        const auto fresh_back = static_cast<std::uint8_t>(back_ | kFresh);
        const std::uint8_t previous = middle_.exchange(fresh_back, std::memory_order_acq_rel);
        back_ = static_cast<std::uint8_t>(previous & kIndexMask);
    }

    // Producer: assigns `value` to the write slot and publishes it.
    void publish(T value) {
        write_slot() = std::move(value);
        publish();
    }

    // Consumer: makes the newest published value readable through read().
    // Returns false (and keeps the current value) when nothing was published
    // since the last successful fetch. Never blocks.
    bool fetch() noexcept {
        if ((middle_.load(std::memory_order_acquire) & kFresh) == 0U) {
            return false;
        }
        const std::uint8_t previous = middle_.exchange(front_, std::memory_order_acq_rel);
        front_ = static_cast<std::uint8_t>(previous & kIndexMask);
        return true;
    }

    // Consumer: the value obtained by the last successful fetch (the initial
    // value before any).
    [[nodiscard]] const T& read() const noexcept { return slots_[front_]; }

private:
    static constexpr std::uint8_t kIndexMask = 0x3U;
    static constexpr std::uint8_t kFresh = 0x4U;

    std::array<T, 3> slots_{};
    // Index of the middle slot, plus kFresh when it holds a value the
    // consumer has not fetched yet.
    std::atomic<std::uint8_t> middle_{1};
    std::uint8_t back_ = 0;  // producer only
    std::uint8_t front_ = 2; // consumer only
};

} // namespace orrery::sim
