#include "json_support.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <new>
#include <set>
#include <vector>

#include "orrery/scene/limits.h"

namespace orrery::scene {

std::string_view to_string(SceneErrorCode code) noexcept {
    switch (code) {
        case SceneErrorCode::io_error:
            return "io error";
        case SceneErrorCode::syntax_error:
            return "syntax error";
        case SceneErrorCode::duplicate_key:
            return "duplicate key";
        case SceneErrorCode::unknown_key:
            return "unknown key";
        case SceneErrorCode::missing_key:
            return "missing key";
        case SceneErrorCode::wrong_type:
            return "wrong type";
        case SceneErrorCode::unsupported_version:
            return "unsupported version";
        case SceneErrorCode::invalid_value:
            return "invalid value";
        case SceneErrorCode::duplicate_name:
            return "duplicate name";
        case SceneErrorCode::unknown_parent:
            return "unknown parent";
        case SceneErrorCode::parent_cycle:
            return "parent cycle";
        case SceneErrorCode::kepler_error:
            return "kepler error";
        case SceneErrorCode::body_error:
            return "body error";
        case SceneErrorCode::limit_exceeded:
            return "limit exceeded";
        case SceneErrorCode::out_of_memory:
            return "out of memory";
        case SceneErrorCode::internal_error:
            return "internal error";
    }
    return "unknown scene error";
}

std::string to_string(const SceneError& error) {
    if (error.path.empty()) {
        return error.message;
    }
    return error.path + ": " + error.message;
}

} // namespace orrery::scene

namespace orrery::scene::detail {

void fail(SceneErrorCode code, std::string path, std::string message) {
    throw SceneFailure{SceneError{code, std::move(path), std::move(message), {}, {}}};
}

std::string child_path(std::string_view parent, std::string_view key) {
    std::string out(parent);
    if (!out.empty()) {
        out += '.';
    }
    out += key;
    return out;
}

std::string index_path(std::string_view parent, std::size_t index) {
    std::string out(parent);
    out += '[';
    out += std::to_string(index);
    out += ']';
    return out;
}

SceneError exception_error(std::exception_ptr exception) noexcept {
    // The messages fit the small-string buffer of the standard libraries, so
    // building the error does not allocate while memory is exhausted.
    try {
        std::rethrow_exception(std::move(exception));
    } catch (const std::bad_alloc&) {
        return SceneError{SceneErrorCode::out_of_memory, {}, "out of memory", {}, {}};
    } catch (const std::exception& e) {
        try {
            return SceneError{SceneErrorCode::internal_error, {}, e.what(), {}, {}};
        } catch (...) {
            return SceneError{SceneErrorCode::internal_error, {}, "exception", {}, {}};
        }
    } catch (...) {
        return SceneError{SceneErrorCode::internal_error, {}, "exception", {}, {}};
    }
}

namespace {

// True if the significand of a JSON number lexeme has a nonzero digit, i.e.
// the number written is not zero whatever its exponent.
bool has_nonzero_significand(std::string_view lexeme) {
    for (const char c : lexeme) {
        if (c == 'e' || c == 'E') {
            return false;
        }
        if (c >= '1' && c <= '9') {
            return true;
        }
    }
    return false;
}

// SAX handler that builds the document while enforcing the limits of
// orrery/scene/limits.h before each value is stored, and that tracks the path
// of the value being parsed for duplicate keys, underflow and limit errors.
class StrictBuilder {
public:
    using number_integer_t = Json::number_integer_t;
    using number_unsigned_t = Json::number_unsigned_t;
    using number_float_t = Json::number_float_t;
    using string_t = Json::string_t;
    using binary_t = Json::binary_t;

    StrictBuilder() = default;
    StrictBuilder(const StrictBuilder&) = delete;
    StrictBuilder& operator=(const StrictBuilder&) = delete;
    StrictBuilder(StrictBuilder&&) = delete;
    StrictBuilder& operator=(StrictBuilder&&) = delete;
    // A partial tree left by a failed parse is torn down without allocating.
    ~StrictBuilder() { release_tree(root_); }

    bool null() { return store(Json(nullptr)); }
    bool boolean(bool value) { return store(Json(value)); }
    bool number_integer(number_integer_t value) { return store(Json(value)); }
    bool number_unsigned(number_unsigned_t value) { return store(Json(value)); }

    bool number_float(number_float_t value, const string_t& lexeme) {
        // strtod turns a nonzero literal below the double range into a
        // (signed) zero, which would slip past sign and range checks. An
        // explicit zero such as -0.0 or 0e-400 is kept as written.
        if (value == 0.0 && has_nonzero_significand(lexeme)) {
            const std::string shown = lexeme.size() > 40 ? lexeme.substr(0, 40) + "..." : lexeme;
            return stop(SceneErrorCode::invalid_value, current_path(),
                        "number " + shown + " is not zero but underflows to zero");
        }
        return store(Json(value));
    }

    bool string(string_t& value) { return store(Json(std::move(value))); }
    bool binary(binary_t& /*value*/) {
        // Only binary formats produce this event; JSON text never does.
        return stop(SceneErrorCode::syntax_error, current_path(), "binary values are not JSON");
    }

    bool start_object(std::size_t /*elements*/) { return open(Json::object(), false); }
    bool start_array(std::size_t /*elements*/) { return open(Json::array(), true); }

    bool key(string_t& name) {
        Frame& frame = frames_.back();
        if (frame.size == limits::max_container_elements) {
            return stop(SceneErrorCode::limit_exceeded, container_path(),
                        "object has more than " + std::to_string(limits::max_container_elements) +
                            " members");
        }
        ++frame.size;
        const bool duplicate = !frame.keys.insert(name).second;
        frame.key = std::move(name);
        if (duplicate && !duplicate_) {
            // Recorded, not fatal: a later syntax error takes precedence.
            duplicate_ = SceneError{SceneErrorCode::duplicate_key,
                                    current_path(),
                                    "key '" + frame.key + "' appears more than once",
                                    {},
                                    {}};
        }
        return true;
    }

    bool end_object() { return close(); }
    bool end_array() { return close(); }

    bool parse_error(std::size_t /*position*/, const std::string& /*token*/,
                     const nlohmann::detail::exception& e) {
        if (e.id == 406) {
            // A number literal beyond the double range (e.g. 1e400) would be
            // infinite; the path still points at the value being parsed.
            return stop(SceneErrorCode::invalid_value, current_path(),
                        "value must be finite (" + std::string(e.what()) + ")");
        }
        return stop(SceneErrorCode::syntax_error, {}, std::string(e.what()));
    }

    [[nodiscard]] const std::optional<SceneError>& error() const noexcept { return error_; }
    [[nodiscard]] const std::optional<SceneError>& duplicate() const noexcept { return duplicate_; }
    [[nodiscard]] Json& root() noexcept { return root_; }

private:
    struct Frame {
        Json* container;
        bool array;
        std::size_t size;
        std::string key;
        std::set<std::string, std::less<>> keys;
    };

    // Path of the value about to be stored: the next element of the
    // innermost container (or its member under the last key).
    [[nodiscard]] std::string current_path() const { return path_of(frames_.size(), true); }

    // Path of the innermost container itself.
    [[nodiscard]] std::string container_path() const {
        return path_of(frames_.empty() ? 0 : frames_.size() - 1, false);
    }

    // Path through the first `depth` frames. Every enclosing array has already
    // counted the open element that leads inwards, so its index is size - 1;
    // with pending_last, the last array's next element (index size) is meant.
    [[nodiscard]] std::string path_of(std::size_t depth, bool pending_last) const {
        std::string path;
        for (std::size_t i = 0; i < depth; ++i) {
            const Frame& frame = frames_[i];
            if (!frame.array) {
                path = child_path(path, frame.key);
                continue;
            }
            const bool pending = pending_last && i + 1 == depth;
            path = index_path(path, pending ? frame.size : frame.size - 1);
        }
        return path;
    }

    bool stop(SceneErrorCode code, std::string path, std::string message) {
        error_ = SceneError{code, std::move(path), std::move(message), {}, {}};
        return false;
    }

    // Counts a new value against the document and array limits.
    bool admit() {
        if (values_ == limits::max_json_values) {
            return stop(SceneErrorCode::limit_exceeded, current_path(),
                        "document has more than " + std::to_string(limits::max_json_values) +
                            " values");
        }
        if (!frames_.empty() && frames_.back().array) {
            Frame& frame = frames_.back();
            if (frame.size == limits::max_container_elements) {
                return stop(SceneErrorCode::limit_exceeded, container_path(),
                            "array has more than " +
                                std::to_string(limits::max_container_elements) + " elements");
            }
            ++frame.size;
        }
        ++values_;
        return true;
    }

    // Places value in the innermost container (or as the root).
    Json* place(Json&& value) {
        if (frames_.empty()) {
            root_ = std::move(value);
            return &root_;
        }
        Frame& frame = frames_.back();
        if (frame.array) {
            frame.container->push_back(std::move(value));
            return &frame.container->back();
        }
        // A duplicate key replaces the earlier value (the duplicate is
        // reported after parsing); empty it first so that it is destroyed
        // without allocating.
        Json& slot = (*frame.container)[frame.key];
        release_tree(slot);
        slot = std::move(value);
        return &slot;
    }

    bool store(Json&& value) {
        if (!admit()) {
            return false;
        }
        place(std::move(value));
        return true;
    }

    bool open(Json&& container, bool array) {
        if (frames_.size() == limits::max_nesting_depth) {
            return stop(SceneErrorCode::limit_exceeded, current_path(),
                        "nesting is deeper than " + std::to_string(limits::max_nesting_depth) +
                            " levels");
        }
        if (!admit()) {
            return false;
        }
        Json* placed = place(std::move(container));
        frames_.push_back(Frame{placed, array, 0, {}, {}});
        return true;
    }

    bool close() {
        frames_.pop_back();
        return true;
    }

    Json root_;
    std::vector<Frame> frames_;
    std::size_t values_ = 0;
    std::optional<SceneError> error_;
    std::optional<SceneError> duplicate_;
};

} // namespace

void release_tree(Json& value) noexcept {
    if (value.is_array()) {
        Json::array_t& array = *value.get_ptr<Json::array_t*>();
        for (Json& child : array) {
            release_tree(child);
        }
        // Destroys only scalars and empty containers: no allocation.
        array.clear();
    } else if (value.is_object()) {
        Json::object_t& object = *value.get_ptr<Json::object_t*>();
        for (auto& member : object) {
            release_tree(member.second);
        }
        object.clear();
    }
}

Document parse_or_throw(std::string_view text) {
    if (text.size() > limits::max_document_bytes) {
        fail(SceneErrorCode::limit_exceeded, {},
             "document is larger than " + std::to_string(limits::max_document_bytes) + " bytes");
    }
    StrictBuilder builder;
    const bool parsed = Json::sax_parse(text.begin(), text.end(), &builder);
    if (builder.error()) {
        throw SceneFailure{*builder.error()};
    }
    if (!parsed) {
        fail(SceneErrorCode::syntax_error, {}, "the document could not be parsed");
    }
    if (builder.duplicate()) {
        throw SceneFailure{*builder.duplicate()};
    }
    return Document(std::move(builder.root()));
}

std::string_view type_name(const Json& value) noexcept {
    switch (value.type()) {
        case Json::value_t::null:
            return "null";
        case Json::value_t::object:
            return "object";
        case Json::value_t::array:
            return "array";
        case Json::value_t::string:
            return "string";
        case Json::value_t::boolean:
            return "boolean";
        case Json::value_t::number_integer:
        case Json::value_t::number_unsigned:
        case Json::value_t::number_float:
            return "number";
        case Json::value_t::binary:
            return "binary";
        case Json::value_t::discarded:
            return "discarded";
    }
    return "unknown";
}

namespace {

[[noreturn]] void wrong_type(const Json& value, const std::string& path,
                             std::string_view expected) {
    fail(SceneErrorCode::wrong_type, path,
         "expected " + std::string(expected) + ", found " + std::string(type_name(value)));
}

} // namespace

std::string as_string(const Json& value, const std::string& path) {
    if (!value.is_string()) {
        wrong_type(value, path, "string");
    }
    return value.get<std::string>();
}

double as_number(const Json& value, const std::string& path) {
    if (!value.is_number()) {
        wrong_type(value, path, "number");
    }
    return value.get<double>();
}

ObjectReader::ObjectReader(const Json& value, std::string path)
    : value_(value), path_(std::move(path)) {
    if (!value_.is_object()) {
        wrong_type(value_, path_, "object");
    }
}

bool ObjectReader::has(std::string_view key) const {
    return value_.find(key) != value_.end();
}

void ObjectReader::allow_only(std::initializer_list<std::string_view> allowed) const {
    for (const auto& item : value_.items()) {
        if (std::ranges::find(allowed, std::string_view(item.key())) == allowed.end()) {
            std::string expected;
            for (const std::string_view key : allowed) {
                expected += expected.empty() ? "" : ", ";
                expected += key;
            }
            fail(SceneErrorCode::unknown_key, path_of(item.key()),
                 "unknown key '" + item.key() + "' (allowed: " + expected + ")");
        }
    }
}

const Json& ObjectReader::required(std::string_view key) const {
    const auto it = value_.find(key);
    if (it == value_.end()) {
        fail(SceneErrorCode::missing_key, path_of(key),
             "required key '" + std::string(key) + "' is missing");
    }
    return *it;
}

std::string ObjectReader::required_string(std::string_view key) const {
    return as_string(required(key), path_of(key));
}

double ObjectReader::required_number(std::string_view key) const {
    return as_number(required(key), path_of(key));
}

long long ObjectReader::required_integer(std::string_view key) const {
    const Json& value = required(key);
    if (!value.is_number_integer()) {
        wrong_type(value, path_of(key), "integer");
    }
    if (value.is_number_unsigned()) {
        const auto u = value.get<unsigned long long>();
        // Saturate; any value this large is an unsupported version anyway.
        return u > 0x7fffffffffffffffULL ? 0x7fffffffffffffffLL : static_cast<long long>(u);
    }
    return value.get<long long>();
}

std::optional<std::string> ObjectReader::optional_string(std::string_view key) const {
    if (!has(key)) {
        return std::nullopt;
    }
    return required_string(key);
}

std::optional<double> ObjectReader::optional_number(std::string_view key) const {
    if (!has(key)) {
        return std::nullopt;
    }
    return required_number(key);
}

OrderedJson number_to_json(double value) {
    if (!std::isfinite(value)) {
        return OrderedJson(nullptr);
    }
    return OrderedJson(value);
}

} // namespace orrery::scene::detail
