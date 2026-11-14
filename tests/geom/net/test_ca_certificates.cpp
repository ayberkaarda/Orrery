#include "mock_server.h"
#include "net_test_support.h"

#include "orrery/geom/net/ca_certificates.h"
#include "orrery/geom/net/model_fetch.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

using orrery::geom::net::CaEnvironment;
using orrery::geom::net::fetch_error_kind_name;
using orrery::geom::net::fetch_model;
using orrery::geom::net::FetchErrorKind;
using orrery::geom::net::FetchOptions;
using orrery::geom::net::find_ca_certificates;
using orrery::geom::net::kDefaultCaBundles;
using orrery::geom::net::kDefaultCaDirectories;
using namespace orrery::geom::net::test;
using namespace std::chrono_literals;

namespace {

// Candidate paths under a temporary directory: the probing logic sees only
// what each test puts there.
struct Candidates {
    TempDir dir;
    std::vector<std::string> bundle_paths;
    std::vector<std::string> directory_paths;

    std::string bundle(const std::string& name, std::string_view content) {
        const auto path = dir.path() / name;
        write_file(path, content);
        return path.string();
    }
    std::string directory(const std::string& name, const std::vector<std::string>& files) {
        const auto path = dir.path() / name;
        std::filesystem::create_directories(path);
        for (const auto& file : files) {
            write_file(path / file, "certificate");
        }
        return path.string();
    }
    [[nodiscard]] std::string missing(const std::string& name) const {
        return (dir.path() / name).string();
    }

    [[nodiscard]] std::vector<std::string_view> bundles() const {
        return {bundle_paths.begin(), bundle_paths.end()};
    }
    [[nodiscard]] std::vector<std::string_view> directories() const {
        return {directory_paths.begin(), directory_paths.end()};
    }
};

} // namespace

TEST_CASE("The default CA locations cover the common distributions", "[net][ca]") {
    CHECK(kDefaultCaBundles[0] == "/etc/ssl/certs/ca-certificates.crt"); // Debian, Ubuntu
    const auto has_bundle = [](std::string_view path) {
        for (const std::string_view bundle : kDefaultCaBundles) {
            if (bundle == path) {
                return true;
            }
        }
        return false;
    };
    CHECK(has_bundle("/etc/pki/tls/certs/ca-bundle.crt")); // Fedora, RHEL
    CHECK(has_bundle("/etc/ssl/ca-bundle.pem"));           // openSUSE
    CHECK(has_bundle("/etc/ssl/cert.pem"));                // Alpine
    CHECK(kDefaultCaDirectories[0] == "/etc/ssl/certs");
}

TEST_CASE("The first non-empty CA bundle is used", "[net][ca]") {
    Candidates c;
    c.bundle_paths = {c.missing("absent.crt"), c.bundle("empty.crt", ""),
                      c.bundle("first.crt", "PEM"), c.bundle("second.crt", "PEM")};
    c.directory_paths = {c.directory("certs", {"5ed36f99.0"})};
    const auto found = find_ca_certificates(CaEnvironment{}, c.bundles(), c.directories());
    REQUIRE(found.has_value());
    CHECK(found.value().bundle == std::filesystem::path{c.bundle_paths[2]});
    CHECK(found.value().directories.empty());
    CHECK(found.value().origin == c.bundle_paths[2]);
}

TEST_CASE("Without a bundle, a directory of hashed certificates is used", "[net][ca]") {
    Candidates c;
    c.bundle_paths = {c.missing("absent.crt")};
    c.directory_paths = {c.missing("absent"),
                         c.directory("unhashed", {"README", "ca.pem", "5ed36f9.0", "5ED36F99.0",
                                                  "5ed36f99.r0", "5ed36f99.", "5ed36f99x0"}),
                         c.directory("hashed", {"README", "5ed36f99.1"}),
                         c.directory("later", {"0123abcd.0"})};
    const auto found = find_ca_certificates(CaEnvironment{}, c.bundles(), c.directories());
    REQUIRE(found.has_value());
    CHECK(found.value().bundle.empty());
    CHECK(found.value().directories == c.directory_paths[2]);
}

TEST_CASE("Without any certificates the error names everything tried", "[net][ca]") {
    Candidates c;
    c.bundle_paths = {c.missing("absent.crt"), c.bundle("empty.crt", "")};
    c.directory_paths = {c.directory("unhashed", {"README"}), c.missing("absent")};
    const auto found = find_ca_certificates(CaEnvironment{}, c.bundles(), c.directories());
    REQUIRE_FALSE(found.has_value());
    const auto& message = found.error().message;
    UNSCOPED_INFO(message);
    CHECK(found.error().kind == FetchErrorKind::tls_failure);
    for (const auto& path : c.bundle_paths) {
        CHECK(message.find(path) != std::string::npos);
    }
    for (const auto& path : c.directory_paths) {
        CHECK(message.find(path) != std::string::npos);
    }
    for (const char* variable : {"CURL_CA_BUNDLE", "SSL_CERT_FILE", "SSL_CERT_DIR"}) {
        CHECK(message.find(variable) != std::string::npos);
    }
}

TEST_CASE("The CA environment variables replace the default locations", "[net][ca]") {
    Candidates c;
    c.bundle_paths = {c.bundle("default.crt", "PEM")};
    c.directory_paths = {c.directory("default-certs", {"5ed36f99.0"})};
    const std::string file = c.bundle("ssl_cert_file.pem", "PEM");
    const std::string curl = c.bundle("curl_ca_bundle.pem", "PEM");
    const std::string dir = c.directory("ssl_cert_dir", {});

    SECTION("SSL_CERT_FILE") {
        const auto found =
            find_ca_certificates(CaEnvironment{"", file, ""}, c.bundles(), c.directories());
        REQUIRE(found.has_value());
        CHECK(found.value().bundle == std::filesystem::path{file});
        CHECK(found.value().directories.empty());
        CHECK(found.value().origin == "SSL_CERT_FILE=" + file);
    }
    SECTION("CURL_CA_BUNDLE wins over SSL_CERT_FILE") {
        const auto found =
            find_ca_certificates(CaEnvironment{curl, file, ""}, c.bundles(), c.directories());
        REQUIRE(found.has_value());
        CHECK(found.value().bundle == std::filesystem::path{curl});
    }
    SECTION("SSL_CERT_DIR alone: no bundle, even though a default one exists") {
        const auto found =
            find_ca_certificates(CaEnvironment{"", "", dir}, c.bundles(), c.directories());
        REQUIRE(found.has_value());
        CHECK(found.value().bundle.empty());
        CHECK(found.value().directories == dir);
    }
    SECTION("SSL_CERT_FILE and SSL_CERT_DIR together") {
        const auto found =
            find_ca_certificates(CaEnvironment{"", file, dir}, c.bundles(), c.directories());
        REQUIRE(found.has_value());
        CHECK(found.value().bundle == std::filesystem::path{file});
        CHECK(found.value().directories == dir);
    }
    SECTION("SSL_CERT_DIR as a list with one usable entry") {
#ifdef _WIN32
        const std::string list = c.missing("absent") + ";" + dir;
#else
        const std::string list = c.missing("absent") + ":" + dir;
#endif
        const auto found =
            find_ca_certificates(CaEnvironment{"", "", list}, c.bundles(), c.directories());
        REQUIRE(found.has_value());
        CHECK(found.value().directories == list);
    }
}

TEST_CASE("A CA variable naming nothing usable is an error, not a fallback", "[net][ca]") {
    Candidates c;
    c.bundle_paths = {c.bundle("default.crt", "PEM")}; // would be found without the variables
    const std::string absent = c.missing("absent.pem");
    const std::string empty = c.bundle("empty.pem", "");
    const std::string dir = c.directory("certs", {});
    const auto rejected = [&c](const CaEnvironment& environment, std::string_view variable,
                               const std::string& value) {
        const auto found = find_ca_certificates(environment, c.bundles(), c.directories());
        REQUIRE_FALSE(found.has_value());
        UNSCOPED_INFO(found.error().message);
        CHECK(found.error().kind == FetchErrorKind::tls_failure);
        CHECK(found.error().message.find(variable) != std::string::npos);
        CHECK(found.error().message.find(value) != std::string::npos);
    };
    rejected(CaEnvironment{"", absent, ""}, "SSL_CERT_FILE", absent);
    rejected(CaEnvironment{"", empty, ""}, "SSL_CERT_FILE", empty);
    rejected(CaEnvironment{"", dir, ""}, "SSL_CERT_FILE", dir);
    rejected(CaEnvironment{absent, c.bundle_paths[0], ""}, "CURL_CA_BUNDLE", absent);
    rejected(CaEnvironment{"", "", absent}, "SSL_CERT_DIR", absent);
    rejected(CaEnvironment{"", c.bundle_paths[0], absent}, "SSL_CERT_DIR", absent);
}

#ifndef _WIN32
namespace {

// Sets an environment variable for the lifetime of the object.
class ScopedVariable {
public:
    ScopedVariable(const char* name, const std::string& value) : name_(name) {
        if (const char* old = std::getenv(name)) { // NOLINT(concurrency-mt-unsafe): test setup
            old_ = std::string{old};
        }
        ::setenv(name, value.c_str(), 1); // NOLINT(concurrency-mt-unsafe): test setup
    }
    ScopedVariable(const ScopedVariable&) = delete;
    ScopedVariable& operator=(const ScopedVariable&) = delete;
    ScopedVariable(ScopedVariable&&) = delete;
    ScopedVariable& operator=(ScopedVariable&&) = delete;
    ~ScopedVariable() {
        if (old_) {
            ::setenv(name_, old_->c_str(), 1); // NOLINT(concurrency-mt-unsafe): test teardown
        } else {
            ::unsetenv(name_); // NOLINT(concurrency-mt-unsafe): test teardown
        }
    }

private:
    const char* name_;
    std::optional<std::string> old_;
};

FetchOptions system_store_options(const TempDir& dir) {
    FetchOptions options; // no ca_bundle_override: the certificates are looked up
    options.cache_dir = dir.path() / "cache";
    options.use_environment_proxy = false;
    options.limits.total_timeout = 10s;
    return options;
}

} // namespace

TEST_CASE("The CA variables of the process reach the transport", "[net][ca][tls]") {
    TempDir dir;
    const TestCertificate certificate = make_test_certificate(dir.path());
    MockServer server{MockServer::Mode::tls, &certificate};
    MockResponse response;
    response.body = binary_stl(1);
    response.headers = {{"Content-Type", "application/octet-stream"}};
    server.route("/part.stl", response);
    const ScopedVariable no_curl_bundle{"CURL_CA_BUNDLE", ""};
    const ScopedVariable no_cert_dir{"SSL_CERT_DIR", ""};

    SECTION("SSL_CERT_FILE naming the test CA makes the server trusted") {
        const ScopedVariable cert_file{"SSL_CERT_FILE", certificate.ca_file.string()};
        const auto result = fetch_model(server.url("/part.stl"), system_store_options(dir));
        if (!result) {
            UNSCOPED_INFO(fetch_error_kind_name(result.error().kind)
                          << ": " << result.error().message);
        }
        CHECK(result.has_value());
    }
    SECTION("SSL_CERT_FILE naming a missing file fails before any request") {
        const std::string absent = (dir.path() / "absent.pem").string();
        const ScopedVariable cert_file{"SSL_CERT_FILE", absent};
        const auto result = fetch_model(server.url("/part.stl"), system_store_options(dir));
        REQUIRE_FALSE(result.has_value());
        CHECK(result.error().kind == FetchErrorKind::tls_failure);
        CHECK(result.error().message.find("SSL_CERT_FILE") != std::string::npos);
        CHECK(server.total_requests() == 0);
    }
}
#endif
