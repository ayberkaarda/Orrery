#include "mock_server.h"
#include "net_test_support.h"

#include "orrery/geom/net/model_fetch.h"
#include "orrery/geom/net/sha256.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <filesystem>
#include <functional>
#include <string>

using orrery::geom::net::default_model_cache_dir;
using orrery::geom::net::fetch_error_kind_name;
using orrery::geom::net::fetch_model;
using orrery::geom::net::FetchErrorKind;
using orrery::geom::net::FetchOptions;
using orrery::geom::net::ModelFormat;
using orrery::geom::net::sha256_hex;
using namespace orrery::geom::net::test;
using namespace std::chrono_literals;

namespace {

// One TLS mock server with a fresh certificate and cache per test.
struct Fixture {
    TempDir dir;
    TestCertificate certificate = make_test_certificate(dir.path());
    MockServer server{MockServer::Mode::tls, &certificate};

    [[nodiscard]] FetchOptions options() const {
        FetchOptions options;
        options.cache_dir = dir.path() / "cache";
        options.ca_bundle_override = certificate.ca_file;
        options.use_environment_proxy = false;
        options.limits.total_timeout = 10s;
        return options;
    }
};

MockResponse body(std::string content, std::string type = "application/octet-stream") {
    MockResponse response;
    response.body = std::move(content);
    response.headers = {{"Content-Type", std::move(type)}};
    return response;
}

MockResponse redirect(const std::string& location) {
    MockResponse response;
    response.status = 302;
    response.headers = {{"Location", location}};
    response.body = "moved";
    return response;
}

FetchErrorKind rejection(const std::string& url, const FetchOptions& options) {
    const auto result = fetch_model(url, options);
    REQUIRE_FALSE(result.has_value());
    UNSCOPED_INFO(fetch_error_kind_name(result.error().kind) << ": " << result.error().message);
    return result.error().kind;
}

} // namespace

TEST_CASE("Production defaults keep the ADR-010 limits", "[net][fetch]") {
    const FetchOptions options;
    CHECK(options.limits.max_download_bytes == 200ULL * 1024 * 1024);
    CHECK(options.limits.max_redirects == 5);
    CHECK(options.limits.archive.max_compression_ratio == 100);
    CHECK(options.limits.archive.max_entries == 1000);
    CHECK(options.limits.archive.max_total_uncompressed_bytes == 200ULL * 1024 * 1024);
    CHECK(options.ca_bundle_override.empty()); // the system trust store
    CHECK(options.use_cache);
    const auto cache = default_model_cache_dir();
    CHECK(cache.filename() == "models");
    CHECK(cache.is_absolute());
}

TEST_CASE("An https download is detected, hashed and cached", "[net][fetch]") {
    Fixture f;
    const std::string stl = binary_stl(2);
    f.server.route("/download?id=7", body(stl));
    FetchOptions options = f.options();
    options.license_note = "CC-BY 4.0, Example Lab";

    const auto result = fetch_model(f.server.url("/download?id=7"), options);
    REQUIRE(result.has_value());
    const auto& model = result.value();
    CHECK(model.format == ModelFormat::stl_binary);
    CHECK(model.sha256 == sha256_hex(bytes_of(stl)));
    CHECK(model.local_path.extension() == ".stl");
    CHECK(model.local_path.parent_path() == options.cache_dir / model.sha256);
    CHECK(read_file(model.local_path) == stl);
    CHECK(model.source_url == f.server.url("/download?id=7"));
    CHECK(model.license_note == "CC-BY 4.0, Example Lab");
    CHECK(model.archive_member.empty());
    CHECK_FALSE(model.from_cache);
    // The staging (temporary) directory is empty after a successful fetch.
    CHECK(count_files(options.cache_dir / ".staging") == 0);
}

TEST_CASE("A cache hit does not download again", "[net][fetch][cache]") {
    Fixture f;
    f.server.route("/wing.obj", body(obj_text(), "text/plain; charset=utf-8"));
    const FetchOptions options = f.options();

    const auto first = fetch_model(f.server.url("/wing.obj"), options);
    REQUIRE(first.has_value());
    CHECK(f.server.requests("/wing.obj") == 1);

    const auto second = fetch_model(f.server.url("/wing.obj"), options);
    REQUIRE(second.has_value());
    CHECK(second.value().from_cache);
    CHECK(second.value().local_path == first.value().local_path);
    CHECK(second.value().sha256 == first.value().sha256);
    CHECK(f.server.requests("/wing.obj") == 1);

    // use_cache = false downloads again into the same content address.
    FetchOptions refresh = options;
    refresh.use_cache = false;
    const auto third = fetch_model(f.server.url("/wing.obj"), refresh);
    REQUIRE(third.has_value());
    CHECK_FALSE(third.value().from_cache);
    CHECK(third.value().local_path == first.value().local_path);
    CHECK(f.server.requests("/wing.obj") == 2);
}

TEST_CASE("A tampered cache entry is not served", "[net][fetch][cache]") {
    Fixture f;
    f.server.route("/part", body(binary_stl(1)));
    const FetchOptions options = f.options();
    const auto first = fetch_model(f.server.url("/part"), options);
    REQUIRE(first.has_value());

    write_file(first.value().local_path, executable_bytes());
    const auto second = fetch_model(f.server.url("/part"), options);
    REQUIRE(second.has_value());
    CHECK_FALSE(second.value().from_cache);
    CHECK(f.server.requests("/part") == 2);
    CHECK(read_file(second.value().local_path) == binary_stl(1));
}

TEST_CASE("http URLs are rejected before any connection", "[net][fetch][scheme]") {
    Fixture f;
    MockServer plain{MockServer::Mode::plain, nullptr};
    plain.route("/part.stl", body(binary_stl(1)));
    const FetchOptions options = f.options();

    CHECK(rejection(plain.url("/part.stl"), options) == FetchErrorKind::scheme_not_allowed);
    std::string upper = plain.url("/part.stl");
    upper.replace(0, 4, "HTTP");
    CHECK(rejection(upper, options) == FetchErrorKind::scheme_not_allowed);
    CHECK(rejection("ftp://127.0.0.1/part.stl", options) == FetchErrorKind::scheme_not_allowed);
    CHECK(rejection("file:///etc/passwd", options) == FetchErrorKind::scheme_not_allowed);
    CHECK(plain.total_requests() == 0);
    // Nothing was written: not even the cache directory.
    CHECK_FALSE(std::filesystem::exists(options.cache_dir));
}

TEST_CASE("An https to http redirect is rejected without contacting the target",
          "[net][fetch][redirect]") {
    Fixture f;
    MockServer plain{MockServer::Mode::plain, nullptr};
    plain.route("/part.stl", body(binary_stl(1)));
    f.server.route("/start", redirect(plain.url("/part.stl")));
    f.server.route("/ftp", redirect("ftp://127.0.0.1/part.stl"));

    CHECK(rejection(f.server.url("/start"), f.options()) == FetchErrorKind::redirect_not_allowed);
    CHECK(rejection(f.server.url("/ftp"), f.options()) == FetchErrorKind::redirect_not_allowed);
    CHECK(f.server.requests("/start") == 1);
    CHECK(plain.total_requests() == 0);
}

TEST_CASE("https redirects are followed up to the limit", "[net][fetch][redirect]") {
    Fixture f;
    f.server.route("/part.stl", body(binary_stl(1)));
    f.server.route("/a", redirect("/b"));
    f.server.route("/b", redirect(f.server.url("/part.stl")));
    f.server.route("/loop", redirect("/loop"));

    const auto followed = fetch_model(f.server.url("/a"), f.options());
    REQUIRE(followed.has_value());
    CHECK(followed.value().format == ModelFormat::stl_binary);

    FetchOptions options = f.options();
    options.limits.max_redirects = 3;
    CHECK(rejection(f.server.url("/loop"), options) == FetchErrorKind::too_many_redirects);
    CHECK(f.server.requests("/loop") == 4); // the request and three redirects
}

TEST_CASE("A server certificate that is not trusted is rejected", "[net][fetch][tls]") {
    Fixture f;
    f.server.route("/part.stl", body(binary_stl(1)));
    FetchOptions options = f.options();
    options.ca_bundle_override.clear(); // the system store does not know the test CA
    CHECK(rejection(f.server.url("/part.stl"), options) == FetchErrorKind::tls_failure);

    // A bundle with another CA does not vouch for the server either.
    TempDir other;
    const TestCertificate stranger = make_test_certificate(other.path());
    options.ca_bundle_override = stranger.ca_file;
    CHECK(rejection(f.server.url("/part.stl"), options) == FetchErrorKind::tls_failure);
}

TEST_CASE("A declared size over the limit is rejected", "[net][fetch][size]") {
    Fixture f;
    f.server.route("/big.stl", body(binary_stl(200))); // 10084 bytes, Content-Length sent
    MockResponse lying = body(binary_stl(1));
    lying.declared_length = 50ULL * 1024 * 1024 * 1024; // claims 50 GiB
    f.server.route("/claims", lying);
    FetchOptions options = f.options();
    options.limits.max_download_bytes = 4096;

    CHECK(rejection(f.server.url("/big.stl"), options) == FetchErrorKind::too_large);
    CHECK(rejection(f.server.url("/claims"), f.options()) == FetchErrorKind::too_large);
}

TEST_CASE("An undeclared streamed size over the limit is rejected", "[net][fetch][size]") {
    Fixture f;
    MockResponse stream = body(binary_stl(2000)); // 100084 bytes
    stream.send_content_length = false;
    f.server.route("/stream", stream);
    // Content-Length that understates the body.
    MockResponse understated = body(binary_stl(2000));
    understated.declared_length = 1000;
    f.server.route("/understated", understated);
    FetchOptions options = f.options();
    options.limits.max_download_bytes = 4096;

    CHECK(rejection(f.server.url("/stream"), options) == FetchErrorKind::too_large);
    // Only the declared bytes are read: the truncated STL is no model.
    CHECK(rejection(f.server.url("/understated"), options) == FetchErrorKind::unrecognised_content);
    // Within the limit the same streamed body is accepted.
    options.limits.max_download_bytes = 200'000;
    const auto accepted = fetch_model(f.server.url("/stream"), options);
    REQUIRE(accepted.has_value());
    CHECK(accepted.value().format == ModelFormat::stl_binary);
}

TEST_CASE("A fake extension does not make a model", "[net][fetch][mime]") {
    Fixture f;
    f.server.route("/setup.stl", body(executable_bytes()));
    f.server.route("/page.stl", body("<!DOCTYPE html><html><body>Download</body></html>",
                                     "text/html; charset=utf-8"));
    f.server.route("/tool.stl", body(binary_stl(1), "application/x-msdownload"));
    f.server.route("/mislabelled.stl", body(obj_text(), "model/stl"));
    const FetchOptions options = f.options();

    CHECK(rejection(f.server.url("/setup.stl"), options) == FetchErrorKind::unrecognised_content);
    CHECK(rejection(f.server.url("/page.stl"), options) == FetchErrorKind::mime_rejected);
    CHECK(rejection(f.server.url("/tool.stl"), options) == FetchErrorKind::mime_rejected);
    CHECK(rejection(f.server.url("/mislabelled.stl"), options) == FetchErrorKind::mime_rejected);
    // Nothing reached the cache.
    CHECK(count_files(options.cache_dir) == 0);
}

TEST_CASE("A slow server times out", "[net][fetch][timeout]") {
    Fixture f;
    MockResponse slow = body(binary_stl(100));
    slow.stall_after_bytes = 10;
    f.server.route("/slow", slow);
    FetchOptions options = f.options();
    options.limits.total_timeout = 1000ms;

    const auto start = std::chrono::steady_clock::now();
    CHECK(rejection(f.server.url("/slow"), options) == FetchErrorKind::timeout);
    CHECK(std::chrono::steady_clock::now() - start < 8s);
}

TEST_CASE("A trickling server trips the low-speed guard", "[net][fetch][timeout]") {
    Fixture f;
    MockResponse slow = body(binary_stl(100));
    slow.stall_after_bytes = 10;
    f.server.route("/slow", slow);
    FetchOptions options = f.options();
    options.limits.total_timeout = 60s;
    options.limits.low_speed_bytes_per_second = 1000;
    options.limits.low_speed_window = 1s;

    const auto start = std::chrono::steady_clock::now();
    CHECK(rejection(f.server.url("/slow"), options) == FetchErrorKind::timeout);
    CHECK(std::chrono::steady_clock::now() - start < 8s);
}

TEST_CASE("A non-200 status is rejected", "[net][fetch]") {
    Fixture f;
    CHECK(rejection(f.server.url("/missing.stl"), f.options()) == FetchErrorKind::http_status);
}

TEST_CASE("A zip download yields its first model", "[net][fetch][zip]") {
    Fixture f;
    const std::string archive = build_zip({{"README.md", "# Wing\n", true},
                                           {"models/wing.obj", obj_text(), true},
                                           {"models/wing.stl", binary_stl(1)}});
    f.server.route("/wing.zip", body(archive, "application/zip"));

    const auto result = fetch_model(f.server.url("/wing.zip"), f.options());
    REQUIRE(result.has_value());
    CHECK(result.value().format == ModelFormat::obj);
    CHECK(result.value().archive_member == "models/wing.obj");
    CHECK(result.value().sha256 == sha256_hex(bytes_of(archive)));
    CHECK(read_file(result.value().local_path) == obj_text());

    // And from the cache, without a second download.
    const auto again = fetch_model(f.server.url("/wing.zip"), f.options());
    REQUIRE(again.has_value());
    CHECK(again.value().from_cache);
    CHECK(again.value().archive_member == "models/wing.obj");
    CHECK(f.server.requests("/wing.zip") == 1);
}

TEST_CASE("Hostile zip downloads are rejected", "[net][fetch][zip]") {
    Fixture f;
    f.server.route("/bomb.zip", body(build_zip({{"bomb.stl", std::string(1 << 20, '\0'), true}})));
    f.server.route("/traversal.zip",
                   body(build_zip({{"../../evil.obj", obj_text()}}), "application/zip"));
    std::vector<ZipMember> many;
    many.reserve(1001);
    for (int i = 0; i < 1001; ++i) {
        many.emplace_back("m" + std::to_string(i) + ".obj", obj_text());
    }
    f.server.route("/many.zip", body(build_zip(many)));
    const FetchOptions options = f.options();

    CHECK(rejection(f.server.url("/bomb.zip"), options) == FetchErrorKind::archive_bomb);
    CHECK(rejection(f.server.url("/traversal.zip"), options) ==
          FetchErrorKind::archive_unsafe_entry);
    CHECK(rejection(f.server.url("/many.zip"), options) ==
          FetchErrorKind::archive_too_many_entries);
    CHECK(count_files(options.cache_dir) == 0);
}

namespace {

// Replaces a cached model with `replacement` and rewrites every copy of the
// old model digest in entry.txt, as someone who can write the cache could.
void poison_cache_entry(const std::filesystem::path& model, const std::string& original,
                        const std::string& replacement) {
    write_file(model, replacement);
    const auto entry_file = model.parent_path() / "entry.txt";
    std::string text = read_file(entry_file);
    const std::string old_digest = sha256_hex(bytes_of(original));
    for (auto at = text.find(old_digest); at != std::string::npos; at = text.find(old_digest)) {
        text.replace(at, old_digest.size(), sha256_hex(bytes_of(replacement)));
    }
    write_file(entry_file, text);
}

} // namespace

TEST_CASE("A replaced model with a matching recorded digest is never served",
          "[net][fetch][cache]") {
    Fixture f;
    const std::string original = binary_stl(1);
    const std::string replacement = binary_stl(3); // a valid model, only different
    f.server.route("/part", body(original));
    const FetchOptions options = f.options();
    const auto first = fetch_model(f.server.url("/part"), options);
    REQUIRE(first.has_value());
    const std::string address = sha256_hex(bytes_of(original));

    poison_cache_entry(first.value().local_path, original, replacement);
    const auto hit = fetch_model(f.server.url("/part"), options);
    REQUIRE(hit.has_value());
    CHECK(hit.value().sha256 == address);
    CHECK(read_file(hit.value().local_path) == original);

    // A forced refresh must not prefer the poisoned entry over the download.
    poison_cache_entry(first.value().local_path, original, replacement);
    FetchOptions refresh = options;
    refresh.use_cache = false;
    const auto fresh = fetch_model(f.server.url("/part"), refresh);
    REQUIRE(fresh.has_value());
    CHECK(fresh.value().sha256 == address);
    CHECK(read_file(fresh.value().local_path) == original);
}

TEST_CASE("A replaced zip member with a matching recorded digest is never served",
          "[net][fetch][cache][zip]") {
    Fixture f;
    const std::string archive = build_zip({{"wing.obj", obj_text(), true}});
    const std::string replacement = "o other\nv 9 9 9\nv 8 8 8\nv 7 7 7\nf 1 2 3\n";
    f.server.route("/wing.zip", body(archive, "application/zip"));
    const FetchOptions options = f.options();
    const auto first = fetch_model(f.server.url("/wing.zip"), options);
    REQUIRE(first.has_value());

    poison_cache_entry(first.value().local_path, obj_text(), replacement);
    const auto hit = fetch_model(f.server.url("/wing.zip"), options);
    REQUIRE(hit.has_value());
    CHECK(read_file(hit.value().local_path) == obj_text());

    poison_cache_entry(first.value().local_path, obj_text(), replacement);
    FetchOptions refresh = options;
    refresh.use_cache = false;
    const auto fresh = fetch_model(f.server.url("/wing.zip"), refresh);
    REQUIRE(fresh.has_value());
    CHECK(read_file(fresh.value().local_path) == obj_text());

    // An intact entry is still served from the cache.
    const auto intact = fetch_model(f.server.url("/wing.zip"), options);
    REQUIRE(intact.has_value());
    CHECK(intact.value().from_cache);
    CHECK(read_file(intact.value().local_path) == obj_text());
}

TEST_CASE("A cache hit honours the current download size limit", "[net][fetch][cache][size]") {
    Fixture f;
    f.server.route("/big.stl", body(binary_stl(200))); // 10084 bytes
    const auto cached = fetch_model(f.server.url("/big.stl"), f.options());
    REQUIRE(cached.has_value());

    FetchOptions strict = f.options();
    strict.limits.max_download_bytes = 4096;
    CHECK(rejection(f.server.url("/big.stl"), strict) == FetchErrorKind::too_large);
    // Under the permissive limit the entry is still a hit.
    const auto again = fetch_model(f.server.url("/big.stl"), f.options());
    REQUIRE(again.has_value());
    CHECK(again.value().from_cache);
}

TEST_CASE("A cache hit honours the current archive limits", "[net][fetch][cache][zip]") {
    Fixture f;
    // 5000 equal bytes deflate to a few dozen: a ratio far above 100.
    const std::string archive = build_zip({{"README.md", "# Wing\n", true},
                                           {"padding.txt", std::string(5000, 'x'), true},
                                           {"models/wing.obj", obj_text(), true}});
    f.server.route("/wing.zip", body(archive, "application/zip"));
    FetchOptions permissive = f.options();
    permissive.limits.archive.max_compression_ratio = 100'000;
    const auto cached = fetch_model(f.server.url("/wing.zip"), permissive);
    REQUIRE(cached.has_value());
    const auto hit = fetch_model(f.server.url("/wing.zip"), permissive);
    REQUIRE(hit.has_value());
    CHECK(hit.value().from_cache);

    FetchOptions entries = permissive;
    entries.limits.archive.max_entries = 2;
    CHECK(rejection(f.server.url("/wing.zip"), entries) ==
          FetchErrorKind::archive_too_many_entries);

    FetchOptions total = permissive;
    total.limits.archive.max_total_uncompressed_bytes = 1000;
    CHECK(rejection(f.server.url("/wing.zip"), total) == FetchErrorKind::archive_bomb);

    const FetchOptions ratio = f.options(); // the default ratio limit of 100
    CHECK(rejection(f.server.url("/wing.zip"), ratio) == FetchErrorKind::archive_bomb);
}

TEST_CASE("A binary PLY download is accepted", "[net][fetch][ply]") {
    Fixture f;
    f.server.route("/little", body(binary_ply(false)));
    f.server.route("/big", body(binary_ply(true), "model/ply"));
    for (const char* path : {"/little", "/big"}) {
        INFO(path);
        const auto result = fetch_model(f.server.url(path), f.options());
        REQUIRE(result.has_value());
        CHECK(result.value().format == ModelFormat::ply);
        CHECK(result.value().local_path.extension() == ".ply");
    }
    const std::string archive = build_zip({{"scan.ply", binary_ply(false), true}});
    f.server.route("/scan.zip", body(archive, "application/zip"));
    const auto zipped = fetch_model(f.server.url("/scan.zip"), f.options());
    REQUIRE(zipped.has_value());
    CHECK(zipped.value().format == ModelFormat::ply);
    CHECK(read_file(zipped.value().local_path) == binary_ply(false));
}

TEST_CASE("A trusted certificate for another host name is rejected", "[net][fetch][tls]") {
    // The chain is trusted (the certificate is its own CA in the bundle), but
    // its names exclude 127.0.0.1: only host name verification can refuse it.
    TempDir dir;
    const TestCertificate other_host = make_test_certificate(dir.path(), "DNS:other-host.invalid");
    MockServer server{MockServer::Mode::tls, &other_host};
    server.route("/part.stl", body(binary_stl(1)));
    FetchOptions options;
    options.cache_dir = dir.path() / "cache";
    options.ca_bundle_override = other_host.ca_file;
    options.use_environment_proxy = false;
    options.limits.total_timeout = 10s;
    CHECK(rejection(server.url("/part.stl"), options) == FetchErrorKind::tls_failure);
    CHECK(server.requests("/part.stl") == 0); // refused during the handshake
}

TEST_CASE("Linked cache directories are refused", "[net][fetch][cache][link]") {
    for (const char* linked : {".staging", "urls"}) {
        INFO(linked);
        Fixture f;
        f.server.route("/part.stl", body(binary_stl(1)));
        const FetchOptions options = f.options();
        const auto elsewhere = f.dir.path() / "elsewhere";
        std::filesystem::create_directories(elsewhere);
        std::filesystem::create_directories(options.cache_dir);
        REQUIRE(make_directory_link(elsewhere, options.cache_dir / linked));

        CHECK(rejection(f.server.url("/part.stl"), options) == FetchErrorKind::cache_failure);
        CHECK(f.server.total_requests() == 0);
        CHECK(std::filesystem::is_empty(elsewhere));
    }
}

TEST_CASE("A linked cache entry directory is replaced, never followed",
          "[net][fetch][cache][link]") {
    Fixture f;
    const std::string stl = binary_stl(1);
    f.server.route("/part.stl", body(stl));
    const FetchOptions options = f.options();
    const auto first = fetch_model(f.server.url("/part.stl"), options);
    REQUIRE(first.has_value());

    // Move the intact entry elsewhere and leave a link under its address.
    const auto entry_dir = first.value().local_path.parent_path();
    const auto elsewhere = f.dir.path() / "elsewhere";
    std::filesystem::rename(entry_dir, elsewhere);
    REQUIRE(make_directory_link(elsewhere, entry_dir));

    const auto second = fetch_model(f.server.url("/part.stl"), options);
    REQUIRE(second.has_value());
    CHECK_FALSE(second.value().from_cache);
    CHECK(f.server.requests("/part.stl") == 2);
    CHECK(read_file(second.value().local_path) == stl);
    // The entry is a real directory again and the link target is untouched.
    CHECK(std::filesystem::symlink_status(entry_dir).type() ==
          std::filesystem::file_type::directory);
    CHECK(read_file(elsewhere / first.value().local_path.filename()) == stl);
}

namespace {

// True when the cache holds no entry, no URL index and no staging leftover
// (the directories the loader creates may exist, empty).
bool cache_holds_nothing(const std::filesystem::path& cache) {
    if (!std::filesystem::exists(cache)) {
        return true;
    }
    for (const auto& entry : std::filesystem::directory_iterator(cache)) {
        const std::string name = entry.path().filename().string();
        if ((name != ".staging" && name != "urls") || !entry.is_directory() ||
            !std::filesystem::is_empty(entry.path())) {
            UNSCOPED_INFO("left in the cache: " << name);
            return false;
        }
    }
    return true;
}

// A cancellation callback that turns true at `when`.
std::function<bool()> cancel_at(std::chrono::steady_clock::time_point when) {
    return [when] { return std::chrono::steady_clock::now() >= when; };
}

} // namespace

TEST_CASE("A cancelled trickling download ends promptly and leaves nothing behind",
          "[net][fetch][cancel]") {
    using Clock = std::chrono::steady_clock;
    Fixture f;
    // One byte every 200 ms: neither the low-speed guard below nor the total
    // timeout would end it for a minute.
    MockResponse slow = body(binary_stl(4));
    slow.trickle_interval = 200ms;
    f.server.route("/trickle.stl", slow);
    f.server.route("/fast.stl", body(binary_stl(1)));
    FetchOptions options = f.options();
    options.limits.total_timeout = 60s;
    options.limits.low_speed_bytes_per_second = 1;
    options.limits.low_speed_window = 60s;
    const Clock::time_point cancelled_at = Clock::now() + 1s;
    options.cancel_requested = cancel_at(cancelled_at);

    const auto result = fetch_model(f.server.url("/trickle.stl"), options);
    const Clock::time_point ended = Clock::now();
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == FetchErrorKind::cancelled);
    CHECK(ended >= cancelled_at);
    // libcurl reports progress about once a second while nothing arrives.
    CHECK(ended - cancelled_at < 2s);
    CHECK(f.server.requests("/trickle.stl") == 1);
    CHECK(cache_holds_nothing(options.cache_dir));

    // A callback that never cancels leaves a normal download unchanged.
    options.cancel_requested = [] { return false; };
    const auto fast = fetch_model(f.server.url("/fast.stl"), options);
    REQUIRE(fast.has_value());
    CHECK(read_file(fast.value().local_path) == binary_stl(1));
    CHECK(fast.value().sha256 == sha256_hex(bytes_of(binary_stl(1))));
}

TEST_CASE("A cancelled download of a silent server ends promptly", "[net][fetch][cancel]") {
    using Clock = std::chrono::steady_clock;
    Fixture f;
    // Headers and ten bytes, then silence: no body callback comes, so only
    // the progress callback can notice the cancellation.
    MockResponse silent = body(binary_stl(4));
    silent.stall_after_bytes = 10;
    silent.stall_for = 30s;
    f.server.route("/silent.stl", silent);
    FetchOptions options = f.options();
    options.limits.total_timeout = 60s;
    options.limits.low_speed_bytes_per_second = 1;
    options.limits.low_speed_window = 60s;
    const Clock::time_point cancelled_at = Clock::now() + 500ms;
    options.cancel_requested = cancel_at(cancelled_at);

    const auto result = fetch_model(f.server.url("/silent.stl"), options);
    const Clock::time_point ended = Clock::now();
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == FetchErrorKind::cancelled);
    // libcurl reports progress about once a second while nothing arrives.
    CHECK(ended - cancelled_at < 2s);
    CHECK(cache_holds_nothing(options.cache_dir));
}

TEST_CASE("A fetch cancelled before it starts sends no request", "[net][fetch][cancel]") {
    Fixture f;
    f.server.route("/part.stl", body(binary_stl(1)));
    FetchOptions options = f.options();
    options.cancel_requested = [] { return true; };
    CHECK(rejection(f.server.url("/part.stl"), options) == FetchErrorKind::cancelled);
    CHECK(f.server.total_requests() == 0);
    CHECK(cache_holds_nothing(options.cache_dir));
}

TEST_CASE("A download whose digest is not the expected one is discarded unread",
          "[net][fetch][digest]") {
    Fixture f;
    const std::string stl = binary_stl(2);
    const std::string other = binary_stl(3);
    f.server.route("/part.stl", body(stl));
    FetchOptions options = f.options();
    options.expected_sha256 = sha256_hex(bytes_of(other));

    CHECK(rejection(f.server.url("/part.stl"), options) == FetchErrorKind::digest_mismatch);
    CHECK(f.server.requests("/part.stl") == 1);
    CHECK(cache_holds_nothing(options.cache_dir));

    // The expected digest: downloaded and cached as usual.
    options.expected_sha256 = sha256_hex(bytes_of(stl));
    const auto fetched = fetch_model(f.server.url("/part.stl"), options);
    REQUIRE(fetched.has_value());
    CHECK(fetched.value().sha256 == options.expected_sha256);
    const auto hit = fetch_model(f.server.url("/part.stl"), options);
    REQUIRE(hit.has_value());
    CHECK(hit.value().from_cache);
    CHECK(f.server.requests("/part.stl") == 2);

    // A cached entry under another digest is not served for this one: the
    // URL is downloaded again and the mismatch reported.
    options.expected_sha256 = sha256_hex(bytes_of(other));
    CHECK(rejection(f.server.url("/part.stl"), options) == FetchErrorKind::digest_mismatch);
    CHECK(f.server.requests("/part.stl") == 3);
}
