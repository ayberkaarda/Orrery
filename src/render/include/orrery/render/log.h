#pragma once

#include <cstdint>
#include <functional>
#include <string_view>

// Minimal logging for the render library (no logging dependency). Messages go
// to a process-wide sink; the default sink writes one line per message to
// stderr.
//
// The sink is called without any logger lock held. It may therefore call
// set_log_sink (including removing itself) and log_message (reentrancy is the
// sink's own concern: an unconditional re-log recurses). Calls are not
// serialised: a sink must be thread-safe, since messages can arrive from
// several threads at once (driver debug callbacks). A message is delivered to
// the sink installed when log_message read it; a replaced sink can still be
// running a call that started before set_log_sink returned.
namespace orrery::render {

enum class LogLevel : std::uint8_t {
    debug,
    info,
    warning,
    error,
};

[[nodiscard]] std::string_view to_string(LogLevel level) noexcept;

using LogSink = std::function<void(LogLevel, std::string_view)>;

// Replaces the sink; an empty function restores the default stderr sink.
// Returns the previous sink (empty if it was the default).
LogSink set_log_sink(LogSink sink);

void log_message(LogLevel level, std::string_view message);

} // namespace orrery::render
