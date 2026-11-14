#include "orrery/geom/net/fetch_error.h"

namespace orrery::geom::net {

std::string_view fetch_error_kind_name(FetchErrorKind kind) noexcept {
    switch (kind) {
        case FetchErrorKind::invalid_url:
            return "invalid_url";
        case FetchErrorKind::scheme_not_allowed:
            return "scheme_not_allowed";
        case FetchErrorKind::redirect_not_allowed:
            return "redirect_not_allowed";
        case FetchErrorKind::too_many_redirects:
            return "too_many_redirects";
        case FetchErrorKind::tls_failure:
            return "tls_failure";
        case FetchErrorKind::network:
            return "network";
        case FetchErrorKind::timeout:
            return "timeout";
        case FetchErrorKind::http_status:
            return "http_status";
        case FetchErrorKind::too_large:
            return "too_large";
        case FetchErrorKind::mime_rejected:
            return "mime_rejected";
        case FetchErrorKind::unrecognised_content:
            return "unrecognised_content";
        case FetchErrorKind::archive_bomb:
            return "archive_bomb";
        case FetchErrorKind::archive_too_many_entries:
            return "archive_too_many_entries";
        case FetchErrorKind::archive_unsafe_entry:
            return "archive_unsafe_entry";
        case FetchErrorKind::archive_without_model:
            return "archive_without_model";
        case FetchErrorKind::archive_invalid:
            return "archive_invalid";
        case FetchErrorKind::cache_failure:
            return "cache_failure";
        case FetchErrorKind::io_failure:
            return "io_failure";
        case FetchErrorKind::cancelled:
            return "cancelled";
        case FetchErrorKind::digest_mismatch:
            return "digest_mismatch";
    }
    return "unknown";
}

} // namespace orrery::geom::net
