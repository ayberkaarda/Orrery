#include "orrery/geom/net/url_policy.h"

#include <catch2/catch_test_macros.hpp>

#include <string>

using orrery::geom::net::check_https_url;
using orrery::geom::net::FetchErrorKind;

namespace {

FetchErrorKind rejection(const std::string& url) {
    const auto result = check_https_url(url);
    REQUIRE_FALSE(result.has_value());
    return result.error().kind;
}

} // namespace

TEST_CASE("URL policy accepts absolute https URLs", "[net][url]") {
    for (const char* url :
         {"https://example.org/model.stl", "HTTPS://example.org/a.glb",
          "https://127.0.0.1:8443/x?y=1#z", "https://[::1]:443/m.obj", "https://example.org"}) {
        INFO(url);
        const auto result = check_https_url(url);
        REQUIRE(result.has_value());
        CHECK(result.value() == url);
    }
}

TEST_CASE("URL policy rejects every scheme other than https", "[net][url]") {
    for (const char* url :
         {"http://example.org/model.stl", "HTTP://example.org/model.stl", "ftp://example.org/m.stl",
          "file:///etc/passwd", "file://C:/Windows/win.ini", "data:text/plain,solid",
          "gopher://example.org/", "httpss://example.org/", "javascript:alert(1)",
          "ws://example.org/", "smb://server/share/model.stl"}) {
        INFO(url);
        CHECK(rejection(url) == FetchErrorKind::scheme_not_allowed);
    }
}

TEST_CASE("URL policy rejects malformed and deceptive URLs", "[net][url]") {
    for (const char* url :
         {"", "example.org/model.stl", "https:example.org/model.stl", "https:///model.stl",
          "https://user:secret@example.org/m.stl", "https://evil.org@example.org/m.stl",
          "https://example.org/a model.stl", "https://example.org/\nmodel.stl",
          "https://exa\\mple.org/m.stl", "https://:443/m.stl", "://example.org"}) {
        INFO(url);
        CHECK(rejection(url) == FetchErrorKind::invalid_url);
    }
    CHECK(rejection("https://example.org/" + std::string(2100, 'a')) ==
          FetchErrorKind::invalid_url);
}
