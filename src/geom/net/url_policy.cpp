#include "orrery/geom/net/url_policy.h"

#include <algorithm>
#include <utility>

namespace orrery::geom::net {

namespace {

using core::failure;

FetchError invalid(std::string message) {
    return FetchError{FetchErrorKind::invalid_url, std::move(message)};
}

char lower(char c) noexcept {
    return (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c;
}

} // namespace

core::Result<std::string, FetchError> check_https_url(std::string_view url) {
    if (url.empty()) {
        return failure(invalid("empty URL"));
    }
    if (url.size() > kMaxUrlLength) {
        return failure(invalid("URL longer than 2048 characters"));
    }
    const bool clean = std::all_of(url.begin(), url.end(), [](char c) {
        const auto u = static_cast<unsigned char>(c);
        return u > 0x20 && u != 0x7F;
    });
    if (!clean) {
        return failure(invalid("URL contains whitespace or control characters"));
    }

    const auto colon = url.find(':');
    if (colon == std::string_view::npos || colon == 0) {
        return failure(invalid("URL has no scheme"));
    }
    std::string scheme;
    for (const char c : url.substr(0, colon)) {
        scheme.push_back(lower(c));
    }
    if (scheme != "https") {
        return failure(FetchError{FetchErrorKind::scheme_not_allowed,
                                  "only https URLs are allowed, got scheme '" + scheme + "'"});
    }
    const std::string_view rest = url.substr(colon + 1);
    if (!rest.starts_with("//")) {
        return failure(invalid("https URL without an authority ('https://host/...')"));
    }
    const std::string_view after_slashes = rest.substr(2);
    const std::string_view authority = after_slashes.substr(0, after_slashes.find_first_of("/?#"));
    if (authority.find('@') != std::string_view::npos) {
        return failure(invalid("URLs with user credentials are not allowed"));
    }
    if (authority.find('\\') != std::string_view::npos) {
        return failure(invalid("backslash in the URL authority"));
    }
    // Host without the port; an IPv6 literal keeps its brackets.
    std::string_view host = authority;
    if (!host.starts_with("[")) {
        host = host.substr(0, host.find(':'));
    }
    if (host.empty() || host == "[]") {
        return failure(invalid("URL has no host"));
    }
    return std::string{url};
}

} // namespace orrery::geom::net
