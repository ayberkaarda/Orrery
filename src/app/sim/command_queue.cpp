#include "orrery/sim/command_queue.h"

#include <utility>

#include <tbb/concurrent_queue.h>

namespace orrery::sim {

struct CommandQueue::Impl {
    tbb::concurrent_queue<SimCommand> queue;
};

CommandQueue::CommandQueue() : impl_(std::make_unique<Impl>()) {}

CommandQueue::CommandQueue(CommandQueue&&) noexcept = default;

CommandQueue& CommandQueue::operator=(CommandQueue&&) noexcept = default;

CommandQueue::~CommandQueue() = default;

void CommandQueue::push(SimCommand command) {
    impl_->queue.push(std::move(command));
}

std::optional<SimCommand> CommandQueue::try_pop() {
    SimCommand command;
    if (!impl_->queue.try_pop(command)) {
        return std::nullopt;
    }
    return command;
}

bool CommandQueue::empty() const {
    return impl_->queue.empty();
}

} // namespace orrery::sim
