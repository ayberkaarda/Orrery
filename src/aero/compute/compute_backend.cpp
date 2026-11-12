#include "orrery/aero/compute/compute_backend.h"

namespace orrery::aero::compute {

core::Result<ScopedBuffer, std::string> make_buffer(IComputeBackend& backend, std::size_t bytes) {
    auto id = backend.create_buffer(bytes);
    if (!id) {
        return core::failure(std::string(id.error()));
    }
    return ScopedBuffer(backend, id.value());
}

} // namespace orrery::aero::compute
