#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "orrery/core/body_store.h"
#include "orrery/core/kepler.h"

namespace orrery::scene {

enum class SceneErrorCode : std::uint8_t {
    // The file could not be opened or read.
    io_error,
    // The text is not well-formed JSON.
    syntax_error,
    // The same key appears twice in one JSON object.
    duplicate_key,
    // A key that the schema does not define (or not for this body kind).
    unknown_key,
    // A required key is absent.
    missing_key,
    // A value has the wrong JSON type.
    wrong_type,
    // The schema version is not supported by this build.
    unsupported_version,
    // A value has the right type but is outside its allowed set or range.
    invalid_value,
    // Two bodies share a name.
    duplicate_name,
    // Keplerian elements name a parent that does not exist.
    unknown_parent,
    // The parent chain of a body loops back onto itself.
    parent_cycle,
    // The Kepler conversion rejected the elements (see SceneError::kepler).
    kepler_error,
    // The body store rejected the body (see SceneError::body).
    body_error,
    // The input exceeds one of the resource limits of orrery/scene/limits.h.
    limit_exceeded,
    // Memory ran out while reading, parsing or building.
    out_of_memory,
    // Any other unexpected exception was caught at the API boundary.
    internal_error,
};

[[nodiscard]] std::string_view to_string(SceneErrorCode code) noexcept;

// Error of the scenario and experiment formats. `path` locates the offending
// value in the document, e.g. "bodies[2].elements.e"; it is empty when the
// error concerns the document as a whole.
struct SceneError {
    SceneErrorCode code = SceneErrorCode::syntax_error;
    std::string path;
    std::string message;
    // Set for kepler_error.
    std::optional<core::KeplerError> kepler;
    // Set for body_error.
    std::optional<core::BodyError> body;
};

// "path: message", or the message alone for document-level errors.
[[nodiscard]] std::string to_string(const SceneError& error);

} // namespace orrery::scene
