// GPU upload boundary: the only translation unit of orrery_aero that uses
// single precision (listed in FLOAT_ALLOWLIST of tools/check_deps.py).
#include "orrery/aero/compute/float32_pack.h"

#include <cmath>
#include <cstring>
#include <limits>

namespace orrery::aero::compute {

static_assert(sizeof(float) == float32_bytes, "binary32 float expected");

namespace {

// Converting a double outside the binary32 range is undefined behaviour in
// C++, so those values are mapped to infinities explicitly.
float to_float32(double value) noexcept {
    if (std::abs(value) > static_cast<double>(std::numeric_limits<float>::max())) {
        return value > 0.0 ? std::numeric_limits<float>::infinity()
                           : -std::numeric_limits<float>::infinity();
    }
    return static_cast<float>(value);
}

} // namespace

std::vector<std::byte> pack_float32(std::span<const double> values) {
    std::vector<std::byte> bytes(values.size() * float32_bytes);
    std::size_t offset = 0;
    for (const double value : values) {
        const float single = to_float32(value);
        std::memcpy(bytes.data() + offset, &single, float32_bytes);
        offset += float32_bytes;
    }
    return bytes;
}

bool unpack_float32(std::span<const std::byte> bytes, std::span<double> values) {
    if (bytes.size() != values.size() * float32_bytes) {
        return false;
    }
    std::size_t offset = 0;
    for (double& value : values) {
        float single = 0.0F;
        std::memcpy(&single, bytes.data() + offset, float32_bytes);
        value = static_cast<double>(single);
        offset += float32_bytes;
    }
    return true;
}

double round_to_float32(double value) noexcept {
    return static_cast<double>(to_float32(value));
}

} // namespace orrery::aero::compute
