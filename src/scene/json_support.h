#pragma once

// Private helpers shared by the scenario and experiment formats. Not installed
// and not part of the public include directory: nlohmann/json is a private
// dependency of orrery_scene.

#include <cstddef>
#include <exception>
#include <initializer_list>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include <nlohmann/json.hpp>

#include "orrery/core/result.h"
#include "orrery/scene/error.h"

namespace orrery::scene::detail {

using Json = nlohmann::json;
using OrderedJson = nlohmann::ordered_json;

// Internal control flow: readers throw this and the public entry points turn
// it into a Result.
struct SceneFailure {
    SceneError error;
};

[[noreturn]] void fail(SceneErrorCode code, std::string path, std::string message);

[[nodiscard]] std::string child_path(std::string_view parent, std::string_view key);
[[nodiscard]] std::string index_path(std::string_view parent, std::size_t index);

// Empties value bottom-up without allocating. nlohmann's destructor of a
// non-empty array or object allocates a work stack inside a noexcept
// destructor, which would terminate the program when memory is exhausted.
// Recursion depth is bounded by limits::max_nesting_depth for parsed
// documents.
void release_tree(Json& value) noexcept;

// A parsed document; its destructor uses release_tree.
class Document {
public:
    explicit Document(Json&& value) noexcept : value_(std::move(value)) {}
    Document(Document&& other) noexcept : value_(std::move(other.value_)) {}
    Document(const Document&) = delete;
    Document& operator=(const Document&) = delete;
    Document& operator=(Document&&) = delete;
    ~Document() { release_tree(value_); }

    [[nodiscard]] const Json& value() const noexcept { return value_; }

private:
    Json value_;
};

// Parses JSON text, rejecting syntax errors, duplicate keys in any object,
// nonzero number literals that underflow to zero, and documents beyond the
// limits of orrery/scene/limits.h (checked before the memory is allocated).
// Throws SceneFailure, or std::bad_alloc when memory runs out.
[[nodiscard]] Document parse_or_throw(std::string_view text);

// out_of_memory for std::bad_alloc, internal_error for anything else.
[[nodiscard]] SceneError exception_error(const std::exception_ptr& exception) noexcept;

// JSON type name used in error messages.
[[nodiscard]] std::string_view type_name(const Json& value) noexcept;

// Read access to one JSON object with schema checks. Every accessor throws a
// SceneFailure carrying the path of the offending value.
class ObjectReader {
public:
    // Fails with wrong_type unless value is an object.
    ObjectReader(const Json& value, std::string path);

    [[nodiscard]] const std::string& path() const noexcept { return path_; }
    [[nodiscard]] bool has(std::string_view key) const;
    [[nodiscard]] std::string path_of(std::string_view key) const { return child_path(path_, key); }

    // Fails with unknown_key for the first key (in document order) not in
    // allowed.
    void allow_only(std::initializer_list<std::string_view> allowed) const;

    // Fails with missing_key if absent, wrong_type if not of the right type.
    [[nodiscard]] const Json& required(std::string_view key) const;
    [[nodiscard]] std::string required_string(std::string_view key) const;
    [[nodiscard]] double required_number(std::string_view key) const;
    [[nodiscard]] long long required_integer(std::string_view key) const;

    [[nodiscard]] std::optional<std::string> optional_string(std::string_view key) const;
    [[nodiscard]] std::optional<double> optional_number(std::string_view key) const;

private:
    const Json& value_;
    std::string path_;
};

// Type-checked conversions of a single value at path.
[[nodiscard]] std::string as_string(const Json& value, const std::string& path);
[[nodiscard]] double as_number(const Json& value, const std::string& path);

// Number to JSON; non-finite values become null (JSON has no NaN or infinity).
[[nodiscard]] OrderedJson number_to_json(double value);

// Runs body and converts a thrown SceneFailure into a failed Result; any other
// exception (std::bad_alloc included) becomes exception_error, so nothing
// escapes the public API.
template <typename T, typename F> [[nodiscard]] core::Result<T, SceneError> guarded(F&& body) {
    try {
        return std::forward<F>(body)();
    } catch (SceneFailure& f) {
        return core::failure(std::move(f.error));
    } catch (...) {
        return core::failure(exception_error(std::current_exception()));
    }
}

} // namespace orrery::scene::detail
