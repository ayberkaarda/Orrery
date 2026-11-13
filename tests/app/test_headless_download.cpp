#include <catch2/catch_test_macros.hpp>

#include <array>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>

#include "../render/gl_test_support.h"
#include "mock_server.h"
#include "net_test_support.h"
#include "orrery/app/aero_headless.h"
#include "orrery/app/cli.h"
#include "orrery/app/headless.h"
#include "orrery/geom/net/sha256.h"
#include "orrery/scene/aero_experiment.h"

// The headless path never downloads an experiment's model on its own: only
// with --allow-download, and then the recorded digest is checked before the
// model is read (docs/url-import-security.md, "Experiments"; docs/app.md,
// "Command line"). The model is served by the loader's local mock server.

using namespace orrery;
namespace net = orrery::geom::net;
using orrery::render::test::environment_flag;

namespace {

// A closed ASCII STL tetrahedron.
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

struct Served {
    net::test::TempDir dir;
    net::test::TestCertificate certificate = net::test::make_test_certificate(dir.path());
    net::test::MockServer server{net::test::MockServer::Mode::tls, &certificate};
    std::string model = tetrahedron();

    Served() {
        net::test::MockResponse response;
        response.body = model;
        response.headers = {{"Content-Type", "model/stl"}};
        server.route("/body.stl", response);
    }

    // An aero experiment whose URL model is not in the cache.
    [[nodiscard]] scene::AeroExperiment experiment() const {
        scene::AeroExperiment e;
        e.name = "Downloaded body";
        e.geometry.source = scene::GeometrySource::url;
        e.geometry.url = server.url("/body.stl");
        e.geometry.path = "gone/source.stl";
        e.geometry.sha256 = net::sha256_hex(net::test::bytes_of(model));
        e.solver.solver = aero::SolverKind::newtonian;
        return e;
    }
};

} // namespace

TEST_CASE("Headless runs refuse to download without --allow-download",
          "[app][headless][cli][download]") {
    Served s;
    const std::filesystem::path file = s.dir.path() / "downloaded_body.json";
    {
        std::ofstream out(file, std::ios::binary);
        out << scene::save_aero_experiment(s.experiment());
    }
    app::CliOptions options;
    options.mode = app::RunMode::headless;
    options.experiment = file.string();
    options.out = s.dir.path() / "never.png";
    const std::array<std::filesystem::path, 0> dirs{};
    std::ostringstream out;
    std::ostringstream err;

    CHECK(app::run_headless_command(options, dirs, out, err) == app::kExitFailure);
    INFO(err.str());
    CHECK(err.str().starts_with("orrery_app: Download not allowed: "));
    CHECK(err.str().find("--allow-download") != std::string::npos);
    CHECK(err.str().find(s.experiment().geometry.sha256) != std::string::npos);
    CHECK(s.server.total_requests() == 0);
    CHECK_FALSE(std::filesystem::exists(options.out));
}

TEST_CASE("Headless runs with download permission fetch and render the model",
          "[app][headless][download][gl]") {
    Served s;
    app::AeroHeadlessRequest request;
    request.width = 160;
    request.height = 90;
    request.msaa_samples = 1;
    request.download.allowed = true; // what --allow-download sets
    request.download.fetch.cache_dir = s.dir.path() / "cache";
    request.download.fetch.ca_bundle_override = s.certificate.ca_file;
    const auto frame = app::render_aero_headless(s.experiment(), s.dir.path(), request);
    if (!frame && frame.error().kind == app::HeadlessErrorKind::no_context) {
        if (environment_flag("ORRERY_REQUIRE_GL")) {
            FAIL("OpenGL 4.6 required (ORRERY_REQUIRE_GL=1) but unavailable: "
                 << frame.error().message);
        }
        SKIP("no OpenGL 4.6 core context: " << frame.error().message);
    }
    INFO((frame ? std::string() : frame.error().message));
    REQUIRE(frame.has_value());
    CHECK(s.server.requests("/body.stl") == 1);
    CHECK(frame.value().image.width == 160);
}
