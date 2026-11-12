#pragma once

#include "orrery/core/result.h"
#include "orrery/geom/net/fetch_error.h"

#include <cstddef>
#include <string>
#include <string_view>

// The URL policy of the model loader, applied to the requested URL and again
// to every redirect target before it is contacted.
namespace orrery::geom::net {

inline constexpr std::size_t kMaxUrlLength = 2048;

// Accepts only absolute https URLs with a host, without user credentials,
// whitespace or control characters, and at most kMaxUrlLength long. The
// scheme is matched case-insensitively. Returns the URL unchanged.
[[nodiscard]] core::Result<std::string, FetchError> check_https_url(std::string_view url);

} // namespace orrery::geom::net
