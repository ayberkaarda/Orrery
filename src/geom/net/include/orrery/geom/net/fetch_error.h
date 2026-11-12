#pragma once

#include <cstdint>
#include <string>
#include <string_view>

// Errors of the URL model loader. Every rejection carries a kind that tests
// and callers can match, and a one-line message for the user.
namespace orrery::geom::net {

enum class FetchErrorKind : std::uint8_t {
    invalid_url,          // malformed URL, credentials in the URL, control characters
    scheme_not_allowed,   // anything but https
    redirect_not_allowed, // a redirect to a non-https URL
    too_many_redirects,   // more hops than FetchLimits::max_redirects
    tls_failure,          // certificate or handshake failure
    network,              // DNS, connection or transfer failure
    timeout,              // connect, total or low-speed timeout
    http_status,          // a final status other than 200
    too_large,            // body larger than FetchLimits::max_download_bytes
    mime_rejected,        // Content-Type not allowed or inconsistent with the content
    unrecognised_content, // magic bytes match no supported model format
    archive_bomb,         // compression ratio or real size beyond the archive limits
    archive_too_many_entries,
    archive_unsafe_entry,  // path traversal, absolute path, symlink, encryption
    archive_without_model, // no supported model file in the archive
    archive_invalid,       // the archive cannot be read
    cache_failure,         // cache directory or entry cannot be written or verified
    io_failure,            // local file system error
};

struct FetchError {
    FetchErrorKind kind = FetchErrorKind::network;
    std::string message;
};

// Field-by-field copy for forwarding an error out of a core::Result. The
// implicit copy constructor trips a clang-analyzer false positive on the
// Result's variant storage (uninitialized field), so callers use this.
[[nodiscard]] inline FetchError copy_error(const FetchError& error) {
    return FetchError{error.kind, error.message};
}

[[nodiscard]] std::string_view fetch_error_kind_name(FetchErrorKind kind) noexcept;

} // namespace orrery::geom::net
