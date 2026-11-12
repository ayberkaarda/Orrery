#pragma once

#include "orrery/core/result.h"
#include "orrery/geom/net/fetch_error.h"
#include "orrery/geom/net/model_fetch.h"

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>

// libcurl transport of the model loader (private to orrery_geom_net).
namespace orrery::geom::net::detail {

struct Download {
    std::uint64_t bytes = 0;
    std::string sha256;       // of the body, lower-case hex
    std::string content_type; // of the final response, as sent
    std::string final_url;    // after redirects
    std::uint32_t redirects = 0;
};

// Downloads `url` into `destination` (created only for a 200 response).
// Redirects are followed by this function, not by libcurl, so that every
// target passes check_https_url() before it is contacted; libcurl is
// additionally restricted to https. The body is counted while streaming and
// the transfer is aborted as soon as it exceeds the size cap, whether or not a
// Content-Length was sent.
[[nodiscard]] core::Result<Download, FetchError>
download_https(std::string_view url, const std::filesystem::path& destination,
               const FetchOptions& options);

} // namespace orrery::geom::net::detail
