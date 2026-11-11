#pragma once

#include <cstddef>

namespace orrery::scene::limits {

// Resource limits of the scenario and experiment readers. They are enforced
// while reading and parsing, before the corresponding memory is allocated,
// and a document beyond any of them fails with SceneErrorCode::limit_exceeded.

// Size of a document, as text or as a file.
inline constexpr std::size_t max_document_bytes = std::size_t{16} * 1024 * 1024;

// Nesting of JSON objects and arrays; the root container is at depth 1.
inline constexpr std::size_t max_nesting_depth = 64;

// Members of one JSON object or elements of one JSON array.
inline constexpr std::size_t max_container_elements = 200'000;

// JSON values (scalars and containers) in one document.
inline constexpr std::size_t max_json_values = 2'000'000;

// Bodies of one scenario.
inline constexpr std::size_t max_bodies = 100'000;

// Distinct probe aero_ref names of one scenario. Far below 2^32, so an
// AeroRef never wraps to the reserved value 0.
inline constexpr std::size_t max_aero_tables = 65'536;

} // namespace orrery::scene::limits
