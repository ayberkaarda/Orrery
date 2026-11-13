#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

// Built-in trace sink of the profiling zones (profile.h) and its Chrome
// trace-event writer (docs/observability.md). Independent of Tracy: it works
// in any build, headless included; zones reach it only in builds with
// ORRERY_PROFILING.
namespace orrery::profile {

// One completed zone; times in nanoseconds since the capture started.
struct ZoneEvent {
    std::string_view name;
    std::int64_t start_ns = 0;
    std::int64_t duration_ns = 0;
    std::uint32_t thread = 0; // 1, 2, ... in the order threads first recorded
};

// Aggregate of every zone with one name, including events beyond the cap.
struct ZoneTotal {
    std::string_view name;
    std::uint64_t count = 0;
    std::int64_t total_ns = 0;
    std::int64_t min_ns = 0;
    std::int64_t max_ns = 0;
};

struct Capture {
    std::vector<ZoneEvent> events; // in completion order
    std::vector<ZoneTotal> totals; // sorted by name
    std::uint64_t dropped = 0;     // events beyond the cap (still in totals)
    std::int64_t duration_ns = 0;  // capture start to stop (or to now)
};

// Events kept per capture by default (about 32 bytes each); totals keep
// counting beyond it.
inline constexpr std::size_t kDefaultMaxEvents = std::size_t{1} << 20U;

// Starts a capture: discards the previous one; zones constructed from now on
// are recorded, on any thread.
void start_capture(std::size_t max_events = kDefaultMaxEvents);
// Stops recording; the capture stays readable. Zones still open are dropped.
void stop_capture() noexcept;
[[nodiscard]] bool capture_active() noexcept;
// A copy of the current or last capture.
[[nodiscard]] Capture capture_snapshot();

// Whether this build compiles the zone macros in (ORRERY_PROFILING), and
// whether they are Tracy zones as well (ORRERY_PROFILING_TRACY).
[[nodiscard]] bool zones_compiled() noexcept;
[[nodiscard]] bool tracy_enabled() noexcept;

// Records one zone directly into the current capture. `name` must outlive
// the capture. Ignored while no capture is active.
void record_zone(const char* name, std::int64_t start_ns, std::int64_t end_ns) noexcept;

// Generation of the current (or last) capture: every start_capture begins a
// new one; 0 before the first.
[[nodiscard]] std::uint64_t capture_generation() noexcept;
// Records one zone of capture `generation` (what Zone does on destruction):
// ignored unless that capture is still the current one and active, checked
// under the sink's lock, so a zone of an earlier capture never lands in a
// restarted one whatever the interleaving of the threads.
void record_zone(std::uint64_t generation, const char* name, std::int64_t start_ns,
                 std::int64_t end_ns) noexcept;
// Nanoseconds since the current capture started (steady clock).
[[nodiscard]] std::int64_t now_ns() noexcept;

// The capture as Chrome trace-event JSON ("JSON Object Format"): complete
// events (ph "X", microseconds), thread name metadata, and the per-zone
// totals under "otherData". `process` names the process.
[[nodiscard]] std::string chrome_trace_json(const Capture& capture, std::string_view process);
// Writes chrome_trace_json(capture_snapshot(), process) to `path`; returns the
// reason on failure.
[[nodiscard]] std::optional<std::string> write_chrome_trace(const std::filesystem::path& path,
                                                            std::string_view process);

} // namespace orrery::profile
