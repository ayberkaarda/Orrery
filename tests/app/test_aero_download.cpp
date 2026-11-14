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
#include "orrery/geom/net/sha256.h"
#include "orrery/sim/aero_import.h"
#include "orrery/sim/background_task.h"

// Downloads of the Aero workspace against the loader's local mock server
// (tests/geom/net): cancelling a stalled download, the recorded digest of an
// experiment checked before the importer, and no download without
// permission (docs/url-import-security.md, "Experiments").

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

    // An experiment reference to `path` on this server whose cached file is gone.
    [[nodiscard]] scene::GeometryRef reference(const std::string& path,
                                               const std::string& sha256) const {
        scene::GeometryRef ref;
        ref.source = scene::GeometrySource::url;
        ref.url = server.url(path);
        ref.path = (dir.path() / "gone" / "source.stl").string();
        ref.sha256 = sha256;
        return ref;
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

std::string sha256_of(const std::string& text) {
    return net::sha256_hex(net::test::bytes_of(text));
}

// A closed ASCII STL tetrahedron (a model the importer and repair accept).
std::string tetrahedron() {
    return "solid t\n"
           " facet normal 0 0 0\n  outer loop\n   vertex 0 0 0\n   vertex 0 1 0\n"
           "   vertex 1 0 0\n  endloop\n endfacet\n"
           " facet normal 0 0 0\n  outer loop\n   vertex 0 0 0\n   vertex 1 0 0\n"
           "   vertex 0 0 1\n  endloop\n endfacet\n"
           " facet normal 0 0 0\n  outer loop\n   vertex 0 0 0\n   vertex 0 0 1\n"
           "   vertex 0 1 0\n  endloop\n endfacet\n"
           " facet normal 0 0 0\n  outer loop\n   vertex 1 0 0\n   vertex 0 1 0\n"
           "   vertex 0 0 1\n  endloop\n endfacet\n"
           "endsolid t\n";
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

TEST_CASE("An experiment never downloads its model without permission",
          "[aero][import][download]") {
    Server s;
    const std::string model = tetrahedron();
    s.server.route("/model.stl", body(model));
    const scene::GeometryRef ref = s.reference("/model.stl", sha256_of(model));
    REQUIRE(sim::needs_download(ref, {}));
    CHECK(sim::url_host(ref.url) == "127.0.0.1");

    // The default (opening an experiment): refused before any request.
    const auto refused = sim::load_experiment_geometry(ref, {});
    REQUIRE_FALSE(refused.has_value());
    CHECK(refused.error().title == sim::kDownloadNotAllowedTitle);
    CHECK(refused.error().detail.find("127.0.0.1") != std::string::npos);
    CHECK(refused.error().detail.find(ref.sha256) != std::string::npos);
    CHECK(refused.error().detail.find("--allow-download") != std::string::npos);
    CHECK(s.server.total_requests() == 0);

    // With permission: fetched from the server and prepared.
    sim::ExperimentDownload download;
    download.allowed = true;
    download.fetch = s.options();
    const auto fetched = sim::load_experiment_geometry(ref, {}, nullptr, download);
    INFO((fetched ? std::string() : fetched.error().title + ": " + fetched.error().detail));
    REQUIRE(fetched.has_value());
    CHECK(s.server.requests("/model.stl") == 1);
    CHECK(fetched.value().ref.sha256 == ref.sha256);
    CHECK(fetched.value().ref.url == ref.url);
    CHECK_FALSE(sim::needs_download(fetched.value().ref, {}));
}

TEST_CASE("A download that is not the recorded model never reaches the importer",
          "[aero][import][download]") {
    Server s;
    // Passes the loader's content sniffing (an ASCII STL) but not the
    // importer: if the importer saw it, its error would be the one reported.
    const std::string served = "solid broken\nfacet garbage\nendsolid broken\n";
    s.server.route("/model.stl", body(served));
    const scene::GeometryRef ref = s.reference("/model.stl", sha256_of(tetrahedron()));
    sim::ExperimentDownload download;
    download.allowed = true;
    download.fetch = s.options();

    const auto result = sim::load_experiment_geometry(ref, {}, nullptr, download);
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().title == sim::kModelChangedTitle);
    CHECK(result.error().detail.find(sha256_of(served)) != std::string::npos);
    CHECK(result.error().detail.find(ref.sha256) != std::string::npos);
    CHECK(s.server.requests("/model.stl") == 1);
    // Discarded: nothing of the download stays in the cache.
    CHECK(net::test::count_files(s.dir.path() / "cache") == 0);
}
