#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <ios>
#include <istream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <streambuf>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "../core/allocation_failure_injection.h"
#include "orrery/scene/error.h"
#include "orrery/scene/experiment.h"
#include "orrery/scene/limits.h"
#include "orrery/scene/scenario.h"
#include "scene_test_support.h"

namespace scene = orrery::scene;
namespace limits = orrery::scene::limits;
using scene::SceneErrorCode;
using scene::test::document_with_bodies;
using scene::test::scenario_path;
using scene::test::sun_json;

namespace {

// A file in the temporary directory, removed on destruction.
class TempFile {
public:
    TempFile(const std::string& name, const std::string& content)
        : path_(std::filesystem::temp_directory_path() / ("orrery_scene_test_" + name)) {
        std::ofstream out(path_, std::ios::binary | std::ios::trunc);
        out.write(content.data(), static_cast<std::streamsize>(content.size()));
        REQUIRE(out.good());
    }
    TempFile(const TempFile&) = delete;
    TempFile& operator=(const TempFile&) = delete;
    ~TempFile() {
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
    }

    [[nodiscard]] const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
};

// Stream buffer that yields `prefix` and then fails like a device read error.
class FailingStreambuf : public std::streambuf {
public:
    explicit FailingStreambuf(std::string prefix) : prefix_(std::move(prefix)) {
        setg(prefix_.data(), prefix_.data(), prefix_.data() + prefix_.size());
    }

protected:
    int_type underflow() override { throw std::ios_base::failure("simulated device read error"); }

private:
    std::string prefix_;
};

std::string nested_arrays(std::size_t depth) {
    return std::string(depth, '[') + "0" + std::string(depth, ']');
}

std::string zeros_array(std::size_t count) {
    std::string out = "[";
    for (std::size_t i = 0; i < count; ++i) {
        out += i == 0 ? "0" : ",0";
    }
    out += "]";
    return out;
}

const std::string two_body_document =
    document_with_bodies(sun_json + R"(, {"name": "P", "kind": "planet", "mass": 0.001,
        "elements": {"parent": "Sun", "a": 2.0, "e": 0.1, "i": 0.2, "raan": 0.3,
                     "argp": 0.4, "ma": 0.5}},
       {"name": "Probe", "kind": "probe", "mass": 1e-27, "aero_ref": "capsule",
        "state": {"position": [3, 0, 0], "velocity": [0, 0.5, 0]}})");

// Runs call once for every allocation it makes, failing that allocation.
// Every run must return (not throw) and, when the injected failure fired,
// report out_of_memory. The final run, with no failure injected, must give
// the same outcome as an undisturbed call. Returns the number of injected
// failures.
template <typename F> std::size_t inject_allocation_failures(F&& call) {
    const bool baseline_ok = call().has_value();
    std::size_t injected = 0;
    for (long long failure_at = 0;; ++failure_at) {
        bool escaped = false;
        bool fired = false;
        std::optional<scene::SceneError> error;
        {
            orrery::test::AllocationFailureScope inject(failure_at);
            try {
                auto result = call();
                inject.disarm();
                if (!result.has_value()) {
                    error = result.error();
                }
            } catch (...) {
                inject.disarm();
                escaped = true;
            }
            fired = inject.fired();
        }
        CAPTURE(failure_at);
        CHECK_FALSE(escaped);
        if (!fired) {
            CHECK(error.has_value() != baseline_ok);
            return injected;
        }
        ++injected;
        if (!escaped) {
            REQUIRE(error.has_value());
            CHECK(error->code == SceneErrorCode::out_of_memory);
        }
    }
}

} // namespace

// ---------------------------------------------------------------------------
// SCENE-02: resource limits and the exception boundary
// ---------------------------------------------------------------------------

TEST_CASE("nesting deeper than the limit is rejected", "[scene][limits]") {
    SECTION("at the limit the document parses") {
        const auto parsed = scene::parse_scenario(nested_arrays(limits::max_nesting_depth));
        REQUIRE_FALSE(parsed.has_value());
        CHECK(parsed.error().code == SceneErrorCode::wrong_type);
    }
    SECTION("one level beyond the limit") {
        const auto parsed = scene::parse_scenario(nested_arrays(limits::max_nesting_depth + 1));
        REQUIRE_FALSE(parsed.has_value());
        CHECK(parsed.error().code == SceneErrorCode::limit_exceeded);
    }
    SECTION("far beyond the limit, inside an object") {
        const auto loaded =
            scene::load_scene(R"({"version": 1, "bodies": )" + nested_arrays(100'000) + "}");
        REQUIRE_FALSE(loaded.has_value());
        CHECK(loaded.error().code == SceneErrorCode::limit_exceeded);
        CHECK(loaded.error().path.rfind("bodies[0][0]", 0) == 0);
    }
    SECTION("experiments share the limit") {
        const auto parsed =
            scene::parse_experiment(R"({"version": 1, "inputs": )" + nested_arrays(1000) + "}");
        REQUIRE_FALSE(parsed.has_value());
        CHECK(parsed.error().code == SceneErrorCode::limit_exceeded);
    }
}

TEST_CASE("containers wider than the limit are rejected", "[scene][limits]") {
    SECTION("array") {
        const auto loaded = scene::load_scene(zeros_array(limits::max_container_elements + 1));
        REQUIRE_FALSE(loaded.has_value());
        CHECK(loaded.error().code == SceneErrorCode::limit_exceeded);
        CHECK(loaded.error().path.empty());
    }
    SECTION("array of empty objects under bodies") {
        std::string text = R"({"version": 1, "bodies": [{})";
        for (std::size_t i = 0; i < limits::max_container_elements; ++i) {
            text += ",{}";
        }
        text += "]}";
        const auto loaded = scene::load_scene(text);
        REQUIRE_FALSE(loaded.has_value());
        CHECK(loaded.error().code == SceneErrorCode::limit_exceeded);
        CHECK(loaded.error().path == "bodies");
    }
    SECTION("object") {
        std::string text = R"({"version": 1, "inputs": {)";
        for (std::size_t i = 0; i <= limits::max_container_elements; ++i) {
            text += (i == 0 ? "\"k" : ",\"k") + std::to_string(i) + "\":0";
        }
        text += "}}";
        const auto parsed = scene::parse_experiment(text);
        REQUIRE_FALSE(parsed.has_value());
        CHECK(parsed.error().code == SceneErrorCode::limit_exceeded);
        CHECK(parsed.error().path == "inputs");
    }
}

TEST_CASE("documents with too many values are rejected", "[scene][limits]") {
    // Each array is within the container limit; together they are not.
    const std::size_t per_array = limits::max_container_elements;
    const std::size_t arrays = limits::max_json_values / per_array + 1;
    std::string text = "[";
    for (std::size_t i = 0; i < arrays; ++i) {
        text += i == 0 ? "" : ",";
        text += zeros_array(per_array);
    }
    text += "]";
    REQUIRE(text.size() <= limits::max_document_bytes);
    const auto loaded = scene::load_scene(text);
    REQUIRE_FALSE(loaded.has_value());
    CHECK(loaded.error().code == SceneErrorCode::limit_exceeded);
}

TEST_CASE("scenarios with too many bodies are rejected", "[scene][limits]") {
    SECTION("document") {
        std::string text = R"({"version": 1, "bodies": [{})";
        for (std::size_t i = 0; i < limits::max_bodies; ++i) {
            text += ",{}";
        }
        text += "]}";
        const auto loaded = scene::load_scene(text);
        REQUIRE_FALSE(loaded.has_value());
        CHECK(loaded.error().code == SceneErrorCode::limit_exceeded);
        CHECK(loaded.error().path == "bodies");
    }
    SECTION("in memory") {
        scene::Scenario scenario;
        scenario.metadata = {"test", "t = 0", ""};
        scene::BodySpec body;
        body.kind = orrery::core::BodyKindTag::asteroid;
        scenario.bodies.resize(limits::max_bodies + 1, body);
        const auto error = scene::validate(scenario);
        REQUIRE(error.has_value());
        CHECK(error->code == SceneErrorCode::limit_exceeded);
        CHECK(error->path == "bodies");
        const auto built = scene::build_scene(scenario);
        REQUIRE_FALSE(built.has_value());
        CHECK(built.error().code == SceneErrorCode::limit_exceeded);
    }
}

TEST_CASE("scenarios with too many distinct aero tables are rejected", "[scene][limits]") {
    scene::Scenario scenario;
    scenario.metadata = {"test", "t = 0", ""};
    for (std::size_t i = 0; i <= limits::max_aero_tables; ++i) {
        scene::BodySpec probe;
        probe.name = "probe-" + std::to_string(i);
        probe.kind = orrery::core::BodyKindTag::probe;
        probe.mass = orrery::core::space::Mass(1e-27);
        probe.aero_ref = "table-" + std::to_string(i);
        probe.orbit = scene::CartesianSpec{
            {orrery::core::space::Length(static_cast<double>(i) + 1.0), {}, {}}, {}};
        scenario.bodies.push_back(std::move(probe));
    }
    const auto error = scene::validate(scenario);
    REQUIRE(error.has_value());
    CHECK(error->code == SceneErrorCode::limit_exceeded);
    CHECK(error->path == "bodies[" + std::to_string(limits::max_aero_tables) + "].aero_ref");

    // One table fewer is accepted and numbered without wrapping.
    scenario.bodies.pop_back();
    REQUIRE_FALSE(scene::validate(scenario).has_value());
    const auto built = scene::build_scene(scenario);
    REQUIRE(built.has_value());
    CHECK(built.value().aero_tables.size() == limits::max_aero_tables);
}

TEST_CASE("documents larger than the byte limit are rejected", "[scene][limits]") {
    // Whitespace around a valid document: only the size is wrong.
    const std::string valid = two_body_document;
    std::string text(limits::max_document_bytes + 1 - valid.size(), ' ');
    text += valid;
    REQUIRE(text.size() == limits::max_document_bytes + 1);

    SECTION("text") {
        const auto loaded = scene::load_scene(text);
        REQUIRE_FALSE(loaded.has_value());
        CHECK(loaded.error().code == SceneErrorCode::limit_exceeded);
        const auto experiment = scene::parse_experiment(text);
        REQUIRE_FALSE(experiment.has_value());
        CHECK(experiment.error().code == SceneErrorCode::limit_exceeded);
    }
    SECTION("file") {
        const TempFile file("over_limit.json", text);
        const auto read = scene::read_text_file(file.path());
        REQUIRE_FALSE(read.has_value());
        CHECK(read.error().code == SceneErrorCode::limit_exceeded);
        const auto loaded = scene::load_scene_file(file.path());
        REQUIRE_FALSE(loaded.has_value());
        CHECK(loaded.error().code == SceneErrorCode::limit_exceeded);
    }
    SECTION("stream") {
        std::istringstream in(text);
        const auto read = scene::read_text_stream(in, "stream");
        REQUIRE_FALSE(read.has_value());
        CHECK(read.error().code == SceneErrorCode::limit_exceeded);
    }
    SECTION("exactly at the limit") {
        text.erase(0, 1);
        const TempFile file("at_limit.json", text);
        const auto loaded = scene::load_scene_file(file.path());
        REQUIRE(loaded.has_value());
        CHECK(loaded.value().state.bodies().size() == 3);
    }
}

TEST_CASE("allocation failures become out_of_memory errors", "[scene][limits][alloc]") {
    SECTION("load_scene") {
        CHECK(inject_allocation_failures([] { return scene::load_scene(two_body_document); }) > 0);
    }
    SECTION("parse_scenario of an invalid document") {
        REQUIRE_FALSE(
            scene::parse_scenario(R"({"version": 1, "bodies": [{"a": [1, 2]}]})").has_value());
        CHECK(inject_allocation_failures([] {
                  return scene::parse_scenario(R"({"version": 1, "bodies": [{"a": [1, 2]}]})");
              }) > 0);
    }
    SECTION("parse_experiment") {
        CHECK(inject_allocation_failures([] {
                  return scene::parse_experiment(
                      R"({"version": 1, "domain": "aero", "inputs": {"mach": 0.8, "body": "x"},
                          "solver": {"name": "panel", "settings": {"n": 64}}})");
              }) > 0);
    }
    SECTION("load_scene_file") {
        // The path is built outside the window in which allocations fail.
        const std::filesystem::path path = scenario_path("sun_earth_moon.json");
        CHECK(inject_allocation_failures([&path] { return scene::load_scene_file(path); }) > 0);
    }
    SECTION("validate") {
        const auto parsed = scene::parse_scenario(two_body_document);
        REQUIRE(parsed.has_value());
        const scene::Scenario& scenario = parsed.value();
        // validate returns an optional error rather than a Result.
        std::size_t injected = 0;
        for (long long failure_at = 0;; ++failure_at) {
            bool escaped = false;
            bool fired = false;
            std::optional<scene::SceneError> error;
            {
                orrery::test::AllocationFailureScope inject(failure_at);
                try {
                    error = scene::validate(scenario);
                } catch (...) {
                    escaped = true;
                }
                inject.disarm();
                fired = inject.fired();
            }
            CAPTURE(failure_at);
            CHECK_FALSE(escaped);
            if (!fired) {
                CHECK_FALSE(error.has_value());
                break;
            }
            ++injected;
            if (!escaped) {
                REQUIRE(error.has_value());
                CHECK(error->code == SceneErrorCode::out_of_memory);
            }
        }
        CHECK(injected > 0);
    }
}

// ---------------------------------------------------------------------------
// SCENE-03: read errors after a successful open
// ---------------------------------------------------------------------------

TEST_CASE("a stream read error is an io error, not a short read", "[scene][io]") {
    SECTION("fails before any data") {
        FailingStreambuf buffer("");
        std::istream in(&buffer);
        const auto read = scene::read_text_stream(in, "device");
        REQUIRE_FALSE(read.has_value());
        CHECK(read.error().code == SceneErrorCode::io_error);
        CHECK(read.error().message.find("device") != std::string::npos);
    }
    SECTION("fails after a complete, valid document") {
        // A short read would return a document that loads successfully.
        FailingStreambuf buffer(two_body_document);
        std::istream in(&buffer);
        const auto read = scene::read_text_stream(in, "device");
        REQUIRE_FALSE(read.has_value());
        CHECK(read.error().code == SceneErrorCode::io_error);
    }
    SECTION("fails after a partial document") {
        FailingStreambuf buffer(R"({"version": 1, "bod)");
        std::istream in(&buffer);
        const auto read = scene::read_text_stream(in, "device");
        REQUIRE_FALSE(read.has_value());
        CHECK(read.error().code == SceneErrorCode::io_error);
    }
    SECTION("a stream already in a failed state") {
        std::istringstream in("{}");
        in.setstate(std::ios::failbit);
        const auto read = scene::read_text_stream(in, "stream");
        REQUIRE_FALSE(read.has_value());
        CHECK(read.error().code == SceneErrorCode::io_error);
    }
}

TEST_CASE("a clean end of stream returns everything read", "[scene][io]") {
    SECTION("empty") {
        std::istringstream in("");
        const auto read = scene::read_text_stream(in, "empty");
        REQUIRE(read.has_value());
        CHECK(read.value().empty());
    }
    SECTION("larger than any read chunk") {
        std::string text;
        for (int i = 0; i < 100'000; ++i) {
            text += static_cast<char>('a' + i % 26);
        }
        std::istringstream in(text);
        const auto read = scene::read_text_stream(in, "text");
        REQUIRE(read.has_value());
        CHECK(read.value() == text);
    }
    SECTION("binary content is kept byte for byte") {
        const std::string text("a\r\nb\0c\x1a"
                               "d",
                               8);
        const TempFile file("binary.txt", text);
        const auto read = scene::read_text_file(file.path());
        REQUIRE(read.has_value());
        CHECK(read.value() == text);
    }
}

TEST_CASE("an empty file reads as empty text and fails to parse", "[scene][io]") {
    const TempFile file("empty.json", "");
    const auto read = scene::read_text_file(file.path());
    REQUIRE(read.has_value());
    CHECK(read.value().empty());
    const auto loaded = scene::load_scene_file(file.path());
    REQUIRE_FALSE(loaded.has_value());
    CHECK(loaded.error().code == SceneErrorCode::syntax_error);
}

TEST_CASE("a directory is an io error", "[scene][io]") {
    const auto read = scene::read_text_file(std::filesystem::temp_directory_path());
    REQUIRE_FALSE(read.has_value());
    CHECK(read.error().code == SceneErrorCode::io_error);
}

#if defined(__linux__)
TEST_CASE("a file that opens but cannot be read is an io error", "[scene][io]") {
    // Opening succeeds; reading offset zero (unmapped) fails with EIO.
    const auto read = scene::read_text_file("/proc/self/mem");
    REQUIRE_FALSE(read.has_value());
    CHECK(read.error().code == SceneErrorCode::io_error);
}
#endif
