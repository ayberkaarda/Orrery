#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include <glm/ext/vector_double3.hpp>
#include <glm/ext/vector_float4.hpp>

#include "orrery/aero/compute/compute_backend.h"
#include "orrery/core/result.h"
#include "orrery/render/aero_field.h"

// Shared pieces of the aero visualisation sources: binary32 packing for the
// GPU kernels (render side of the precision boundary) and the buffer layouts
// the kernels in shaders/aero/compute expect.
namespace orrery::render::detail {

[[nodiscard]] inline float to_float(double value) noexcept {
    return static_cast<float>(value);
}

// A new backend buffer holding `bytes`.
[[nodiscard]] core::Result<aero::compute::ScopedBuffer, std::string>
upload_new(aero::compute::IComputeBackend& backend, std::span<const std::byte> bytes);

// Marching-cubes table on the GPU: per case a triangle count and up to 12
// triangles of three cell edges.
inline constexpr std::size_t kMarchingCubesStride = 37;

// aero_vis_marching_cubes output, std430 { vec4 position[3]; uvec4 edge; }:
// positions in grid index units relative to the grid origin.
struct TriangleRecord {
    std::array<std::array<float, 4>, 3> position{};
    std::array<std::uint32_t, 4> edge{};
};
static_assert(sizeof(TriangleRecord) == 64);
inline constexpr std::size_t kTriangleRecordBytes = sizeof(TriangleRecord);

// The flow field as the kernels read it: velocity (m/s) per node as vec4,
// solid flags packed four to a uint (byte n % 4 of word n / 4).
struct PackedField {
    std::vector<glm::vec4> velocity;
    std::vector<std::uint32_t> solid;
    float max_speed = 0.0F;
};
[[nodiscard]] PackedField pack_field(const VectorGrid& velocity,
                                     std::span<const std::uint8_t> solid);

// World position -> grid index coordinates (binary32) and back.
[[nodiscard]] glm::vec4 to_index_space(const GridGeometry& grid, const glm::dvec3& world);
[[nodiscard]] glm::dvec3 from_index_space(const GridGeometry& grid, const glm::vec4& index);

// Field buffers on a backend, uploaded once per field.
struct FieldBuffers {
    aero::compute::ScopedBuffer velocity;
    aero::compute::ScopedBuffer solid;
    float max_speed = 0.0F;
};
[[nodiscard]] core::Result<FieldBuffers, std::string>
upload_field(aero::compute::IComputeBackend& backend, const VectorGrid& velocity,
             std::span<const std::uint8_t> solid);

// Largest |u| over fluid nodes.
[[nodiscard]] double max_fluid_speed(const VectorGrid& velocity,
                                     std::span<const std::uint8_t> solid) noexcept;

} // namespace orrery::render::detail
