#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

// Local mock HTTP(S) server for the URL loader tests. It listens on an
// ephemeral port of 127.0.0.1 only, serves canned responses per path, counts
// requests, and never touches the network beyond the loopback interface.
// TLS uses a self-signed certificate generated when the test runs; its PEM
// file is what the tests pass as FetchOptions::ca_bundle_override.
namespace orrery::geom::net::test {

struct TestCertificate {
    std::string certificate_pem;
    std::string private_key_pem;
    std::filesystem::path ca_file; // the certificate, written as a PEM file
};

// Self-signed certificate for IP 127.0.0.1 and DNS localhost, valid for a
// day, with its PEM file written into `dir`.
TestCertificate make_test_certificate(const std::filesystem::path& dir);

struct MockResponse {
    int status = 200;
    std::vector<std::pair<std::string, std::string>> headers;
    std::string body;
    // false: no Content-Length; the body ends when the connection closes.
    bool send_content_length = true;
    // Send this Content-Length instead of the real body size.
    std::optional<std::uint64_t> declared_length;
    // Send only this many body bytes, then wait (up to `stall_for`, or until
    // the server stops) before closing: a slow or hanging server.
    std::optional<std::size_t> stall_after_bytes;
    std::chrono::milliseconds stall_for{10'000};
};

class MockServer {
public:
    enum class Mode : std::uint8_t { tls, plain };

    // `certificate` is required for Mode::tls.
    MockServer(Mode mode, const TestCertificate* certificate);
    MockServer(const MockServer&) = delete;
    MockServer& operator=(const MockServer&) = delete;
    MockServer(MockServer&&) = delete;
    MockServer& operator=(MockServer&&) = delete;
    ~MockServer();

    void route(const std::string& path, MockResponse response);

    // "https://127.0.0.1:<port><path>" (or http:// for Mode::plain).
    [[nodiscard]] std::string url(std::string_view path) const;
    [[nodiscard]] int requests(const std::string& path) const;
    [[nodiscard]] int total_requests() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace orrery::geom::net::test
