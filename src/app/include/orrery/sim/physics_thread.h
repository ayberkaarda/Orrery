#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <semaphore>
#include <stop_token>
#include <string>
#include <thread>

#include "orrery/core/sim_snapshot.h"
#include "orrery/sim/command_queue.h"
#include "orrery/sim/sim_command.h"
#include "orrery/sim/simulation.h"
#include "orrery/sim/triple_buffer.h"

// The physics thread of the application (ADR-011, docs/app.md "Physics
// thread"). It owns a Simulation; the render thread talks to it only through
//
//   send(SimCommand)  render -> physics, lock-free queue (any thread)
//   wake()            render -> physics, "a frame started" (never blocks)
//   fetch()/latest()  physics -> render, triple buffer of immutable
//                     shared_ptr<const SimSnapshot> plus the SimStatus
//
// No mutex is shared between the threads: a frame never waits for physics.
namespace orrery::sim {

// What the physics thread publishes after a tick that changed anything.
struct PublishedFrame {
    std::shared_ptr<const core::SimSnapshot> snapshot;
    SimStatus status;
};

struct PhysicsThreadOptions {
    // Longest wait between two ticks when nothing wakes the thread (no frames
    // are being rendered, for example while the window is minimised).
    std::chrono::microseconds idle_tick{8333};
    // Command work of one tick (at least 1 each; the rest waits for the
    // next tick, which follows at once).
    std::size_t max_commands_per_tick = 256;
    std::size_t max_single_steps_per_tick = 32;
};

class PhysicsThread {
public:
    // Publishes the initial state of `simulation` (readable through latest()
    // at once) and starts the thread.
    explicit PhysicsThread(Simulation simulation, PhysicsThreadOptions options = {});
    PhysicsThread(const PhysicsThread&) = delete;
    PhysicsThread& operator=(const PhysicsThread&) = delete;
    PhysicsThread(PhysicsThread&&) = delete;
    PhysicsThread& operator=(PhysicsThread&&) = delete;
    // Stops and joins the thread.
    ~PhysicsThread();

    // Queues a command for the next tick and wakes the thread. Any thread.
    void send(SimCommand command);
    // Wakes the thread for a tick now instead of after idle_tick. Never
    // blocks; called by the render loop once per frame.
    void wake();

    // Consumer side (one thread, the render thread): makes the newest
    // published frame readable; false when nothing new was published.
    bool fetch() noexcept;
    [[nodiscard]] const PublishedFrame& latest() const noexcept { return frames_.read(); }

    // Ticks completed so far (for tests and diagnostics).
    [[nodiscard]] std::uint64_t ticks() const noexcept {
        return ticks_.load(std::memory_order_acquire);
    }

private:
    void run(const std::stop_token& stop);
    void tick(std::int64_t wall_ns, const std::stop_token& stop);
    // Pauses and publishes `message` as the status message.
    void report(std::string message);

    Simulation simulation_;
    PhysicsThreadOptions options_;
    CommandQueue commands_;
    // A command taken from the queue but deferred to the next tick (physics
    // thread only).
    std::optional<SimCommand> held_back_;
    TripleBuffer<PublishedFrame> frames_;
    // Physics-thread copies of what was published last.
    std::shared_ptr<const core::SimSnapshot> published_snapshot_;
    SimStatus published_status_;
    std::counting_semaphore<> wake_{0};
    std::atomic<std::uint64_t> ticks_{0};
    // Declared last: started after every other member is constructed and
    // joined (by the destructor) before any of them is destroyed.
    std::jthread thread_;
};

} // namespace orrery::sim
