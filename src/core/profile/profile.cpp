#include "orrery/core/profile.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string_view>
#include <thread>
#include <vector>

#include "orrery/core/trace_capture.h"

namespace orrery::profile {

namespace {

// Initial reservation of a capture's event list (it grows up to the cap).
constexpr std::size_t kInitialReserve = 4096;

// The process-wide sink. `active` and `origin_ns` are read without the lock
// by Zone; everything else is guarded by `mutex`.
struct Sink {
    std::mutex mutex;
    std::atomic<bool> active{false};
    std::atomic<std::int64_t> origin_ns{0}; // steady clock at capture start
    std::atomic<std::uint64_t> lost{0};     // events lost to an allocation failure
    std::vector<ZoneEvent> events;
    std::vector<ZoneTotal> totals;
    std::vector<std::thread::id> threads;
    std::size_t max_events = 0;
    std::uint64_t dropped = 0;
    std::int64_t stop_ns = -1; // relative stop time, -1 while running
};

Sink& sink() {
    static Sink instance;
    return instance;
}

std::int64_t steady_ns() noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// Thread number of the calling thread in the current capture (lock held).
std::uint32_t thread_number(Sink& s) {
    const std::thread::id id = std::this_thread::get_id();
    const auto found = std::find(s.threads.begin(), s.threads.end(), id);
    if (found != s.threads.end()) {
        return static_cast<std::uint32_t>(found - s.threads.begin()) + 1U;
    }
    s.threads.push_back(id);
    return static_cast<std::uint32_t>(s.threads.size());
}

// Adds one duration to the total of `name` (lock held).
void add_total(Sink& s, std::string_view name, std::int64_t duration_ns) {
    const auto found = std::find_if(s.totals.begin(), s.totals.end(),
                                    [name](const ZoneTotal& t) { return t.name == name; });
    if (found == s.totals.end()) {
        s.totals.push_back(ZoneTotal{name, 1, duration_ns, duration_ns, duration_ns});
        return;
    }
    ++found->count;
    found->total_ns += duration_ns;
    found->min_ns = std::min(found->min_ns, duration_ns);
    found->max_ns = std::max(found->max_ns, duration_ns);
}

} // namespace

Zone::Zone(const char* name) noexcept {
    if (sink().active.load(std::memory_order_relaxed)) {
        name_ = name;
        start_ns_ = steady_ns();
    }
}

Zone::~Zone() {
    if (name_ == nullptr) {
        return;
    }
    const std::int64_t end = steady_ns();
    const std::int64_t origin = sink().origin_ns.load(std::memory_order_acquire);
    if (start_ns_ >= origin) { // started in this capture, not in an earlier one
        record_zone(name_, start_ns_ - origin, end - origin);
    }
}

void start_capture(std::size_t max_events) {
    Sink& s = sink();
    const std::scoped_lock lock(s.mutex);
    s.active.store(false, std::memory_order_relaxed);
    s.events.clear();
    s.events.shrink_to_fit();
    s.events.reserve(std::min(max_events, kInitialReserve));
    s.totals.clear();
    s.threads.clear();
    s.max_events = max_events;
    s.dropped = 0;
    s.lost.store(0, std::memory_order_relaxed);
    s.stop_ns = -1;
    s.origin_ns.store(steady_ns(), std::memory_order_release);
    s.active.store(true, std::memory_order_release);
}

void stop_capture() noexcept {
    Sink& s = sink();
    try {
        const std::scoped_lock lock(s.mutex);
        if (s.active.exchange(false, std::memory_order_acq_rel)) {
            s.stop_ns = steady_ns() - s.origin_ns.load(std::memory_order_relaxed);
        }
    } catch (...) {
        // Locking failed: still stop recording (the duration then runs to now).
        s.active.store(false, std::memory_order_release);
    }
}

bool capture_active() noexcept {
    return sink().active.load(std::memory_order_acquire);
}

Capture capture_snapshot() {
    Sink& s = sink();
    const std::scoped_lock lock(s.mutex);
    Capture capture;
    capture.events = s.events;
    capture.totals = s.totals;
    std::sort(capture.totals.begin(), capture.totals.end(),
              [](const ZoneTotal& a, const ZoneTotal& b) { return a.name < b.name; });
    capture.dropped = s.dropped + s.lost.load(std::memory_order_relaxed);
    capture.duration_ns =
        s.stop_ns >= 0 ? s.stop_ns : steady_ns() - s.origin_ns.load(std::memory_order_relaxed);
    return capture;
}

bool zones_compiled() noexcept {
#if defined(ORRERY_PROFILING)
    return true;
#else
    return false;
#endif
}

bool tracy_enabled() noexcept {
#if defined(ORRERY_PROFILING) && defined(ORRERY_PROFILING_TRACY)
    return true;
#else
    return false;
#endif
}

void record_zone(const char* name, std::int64_t start_ns, std::int64_t end_ns) noexcept {
    Sink& s = sink();
    if (name == nullptr || !s.active.load(std::memory_order_acquire)) {
        return;
    }
    try {
        const std::scoped_lock lock(s.mutex);
        if (!s.active.load(std::memory_order_relaxed)) {
            return; // stopped while this zone was open
        }
        const std::string_view text(name);
        const std::int64_t duration = std::max<std::int64_t>(end_ns - start_ns, 0);
        add_total(s, text, duration);
        if (s.events.size() >= s.max_events) {
            ++s.dropped;
            return;
        }
        s.events.push_back(ZoneEvent{text, start_ns, duration, thread_number(s)});
    } catch (...) {
        s.lost.fetch_add(1, std::memory_order_relaxed);
    }
}

std::int64_t now_ns() noexcept {
    return steady_ns() - sink().origin_ns.load(std::memory_order_acquire);
}

} // namespace orrery::profile
