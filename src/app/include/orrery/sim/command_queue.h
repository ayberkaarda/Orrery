#pragma once

#include <memory>
#include <optional>

#include "orrery/sim/sim_command.h"

namespace orrery::sim {

// Multi-producer, multi-consumer FIFO of SimCommands backed by
// tbb::concurrent_queue (oneTBB stays an implementation detail: no public
// header includes it). push() and try_pop() never wait for each other; the
// queue grows as needed (push may throw std::bad_alloc). Commands pushed by
// one thread are popped in the order that thread pushed them.
class CommandQueue {
public:
    CommandQueue();
    CommandQueue(const CommandQueue&) = delete;
    CommandQueue& operator=(const CommandQueue&) = delete;
    CommandQueue(CommandQueue&&) noexcept;
    CommandQueue& operator=(CommandQueue&&) noexcept;
    ~CommandQueue();

    void push(SimCommand command);
    // The oldest command, or nothing when the queue is empty.
    [[nodiscard]] std::optional<SimCommand> try_pop();
    // Snapshot of emptiness; may be stale as soon as it returns when other
    // threads push or pop concurrently.
    [[nodiscard]] bool empty() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace orrery::sim
