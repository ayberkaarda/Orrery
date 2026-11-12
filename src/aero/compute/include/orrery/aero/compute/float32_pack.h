#pragma once

#include <cstddef>
#include <span>
#include <vector>

// Conversion between host doubles and the IEEE binary32 values stored in
// device buffers. This is the single place in orrery_aero where single
// precision appears (float32_pack.cpp is on the FLOAT_ALLOWLIST of
// tools/check_deps.py); everything on the host side stays double.
namespace orrery::aero::compute {

inline constexpr std::size_t float32_bytes = 4;

// Each value rounded to nearest binary32 (values above the largest finite
// binary32 become infinities), packed in native byte order.
[[nodiscard]] std::vector<std::byte> pack_float32(std::span<const double> values);

// Writes values.size() doubles read from `bytes` (which must hold exactly
// values.size() binary32 values); returns false and leaves `values` untouched
// on a size mismatch.
[[nodiscard]] bool unpack_float32(std::span<const std::byte> bytes, std::span<double> values);

// The binary32 value nearest to `value`, returned as a double (the rounding a
// Float32Uniform or a packed buffer applies).
[[nodiscard]] double round_to_float32(double value) noexcept;

} // namespace orrery::aero::compute
