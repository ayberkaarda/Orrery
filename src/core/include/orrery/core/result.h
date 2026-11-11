#pragma once

#include <utility>
#include <variant>

namespace orrery::core {

// Wrapper that marks a value as an error when constructing a Result.
template <typename E> struct Failure {
    E error;
};

template <typename E> [[nodiscard]] constexpr Failure<E> failure(E error) {
    return Failure<E>{std::move(error)};
}

// Minimal value-or-error type (std::expected is C++23).
// Accessing the alternative that is not held throws std::bad_variant_access.
template <typename T, typename E> class [[nodiscard]] Result {
public:
    // NOLINTNEXTLINE(google-explicit-constructor): implicit success, as with std::expected.
    constexpr Result(T value) : storage_(std::in_place_index<0>, std::move(value)) {}
    // NOLINTNEXTLINE(google-explicit-constructor): implicit failure, as with std::unexpected.
    constexpr Result(Failure<E> f) : storage_(std::in_place_index<1>, std::move(f.error)) {}

    [[nodiscard]] constexpr bool has_value() const noexcept { return storage_.index() == 0; }
    constexpr explicit operator bool() const noexcept { return has_value(); }

    [[nodiscard]] constexpr const T& value() const& { return std::get<0>(storage_); }
    [[nodiscard]] constexpr T& value() & { return std::get<0>(storage_); }
    [[nodiscard]] constexpr T&& value() && { return std::get<0>(std::move(storage_)); }

    [[nodiscard]] constexpr const E& error() const& { return std::get<1>(storage_); }

private:
    std::variant<T, E> storage_;
};

} // namespace orrery::core
