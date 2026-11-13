#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <filesystem>
#include <stop_token>
#include <string>
#include <thread>
#include <utility>

#include "app_test_support.h"
#include "mock_server.h"
#include "net_test_support.h"
#include "orrery/sim/aero_import.h"
#include "orrery/sim/background_task.h"

// Downloads of the Aero workspace against the loader's local mock server
// (tests/geom/net): cancelling a stalled download.

using namespace orrery;
namespace net = orrery::geom::net;
using namespace std::chrono_literals;

namespace {

using Clock = std::chrono::steady_clock;

// A TLS mock server whose certificate the downloads trust, and a cache.
struct Server {
    net::test::TempDir dir;
    net::test::TestCertificate certificate = net::test::make_test_certificate(dir.path());
    net::test::MockServer server{net::test::MockServer::Mode::tls, &certificate};

    [[nodiscard]] sim::UrlImportOptions options() const {
        sim::UrlImportOptions o;
        o.cache_dir = dir.path() / "cache";
        o.ca_bundle_override = certificate.ca_file;
        return o;
    }
};

net::test::MockResponse body(std::string content) {
    net::test::MockResponse response;
    response.body = std::move(content);
    response.headers = {{"Content-Type", "application/octet-stream"}};
    return response;
}

// One byte every 200 ms: the fetch limits alone would let it run for minutes.
net::test::MockResponse trickle() {
    net::test::MockResponse response = body(net::test::binary_stl(40));
    response.trickle_interval = 200ms;
    return response;
}

} // namespace

TEST_CASE("A stop request ends a stalled URL import within about a second",
          "[aero][import][cancel]") {
    Server s;
    s.server.route("/trickle.stl", trickle());
    std::stop_source stop;
    sim::TaskProgress progress;
    progress.watch(stop.get_token());
    Clock::time_point stopped;
    std::jthread canceller([&stop, &stopped] {
        std::this_thread::sleep_for(500ms);
        stopped = Clock::now();
        stop.request_stop();
    });
    const auto imported =
        sim::fetch_model_url(s.server.url("/trickle.stl"), s.options(), &progress);
    const Clock::time_point ended = Clock::now();
    canceller.join();
    REQUIRE_FALSE(imported.has_value());
    CHECK(imported.error().title == "Cancelled");
    CHECK(s.server.requests("/trickle.stl") == 1);
    // About a second after the stop at most (libcurl's progress interval).
    CHECK(ended - stopped < 2500ms);
    CHECK(net::test::count_files(s.dir.path() / "cache") == 0);
}

TEST_CASE("Closing during a stalled download does not wait for the fetch limits",
          "[aero][import][cancel]") {
    Server s;
    s.server.route("/trickle.stl", trickle());
    const sim::UrlImportOptions options = s.options();
    const std::string url = s.server.url("/trickle.stl");
    Clock::time_point closed;
    {
        sim::BackgroundTask<sim::ImportResult> task(
            [url, options](sim::TaskProgress& progress) {
                return sim::fetch_model_url(url, options, &progress);
            },
            [](const std::string& message) -> sim::ImportResult {
                return core::failure(sim::GeometryError{"Download failed", message});
            });
        REQUIRE(app_test::wait_until([&] { return s.server.requests("/trickle.stl") == 1; }));
        CHECK_FALSE(task.done());
        closed = Clock::now();
    } // the window closes: the task requests the stop and joins
    CHECK(Clock::now() - closed < 2500ms);
    CHECK(net::test::count_files(s.dir.path() / "cache") == 0);
}

TEST_CASE("Cancel stops a running download task", "[aero][import][cancel]") {
    Server s;
    s.server.route("/trickle.stl", trickle());
    const sim::UrlImportOptions options = s.options();
    const std::string url = s.server.url("/trickle.stl");
    sim::BackgroundTask<sim::ImportResult> task(
        [url, options](sim::TaskProgress& progress) {
            return sim::fetch_model_url(url, options, &progress);
        },
        [](const std::string& message) -> sim::ImportResult {
            return core::failure(sim::GeometryError{"Download failed", message});
        });
    REQUIRE(app_test::wait_until([&] { return s.server.requests("/trickle.stl") == 1; }));
    const Clock::time_point pressed = Clock::now();
    task.request_stop();
    REQUIRE(app_test::wait_until([&] { return task.done(); }, 5s));
    CHECK(Clock::now() - pressed < 2500ms);
    const sim::ImportResult result = task.take();
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().title == "Cancelled");
}
