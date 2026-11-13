#include <array>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>

#include "orrery/core/trace_capture.h"

// Chrome trace-event JSON of a capture (docs/observability.md, "Trace file").
namespace orrery::profile {

namespace {

constexpr std::string_view kFormatName = "orrery-trace";
constexpr std::string_view kFormatVersion = "1";

void append_integer(std::string& out, std::int64_t value) {
    std::array<char, 24> buffer{};
    const auto written = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value);
    out.append(buffer.data(), written.ptr);
}

// Nanoseconds as microseconds with exactly three decimals (no rounding, no
// locale): 1234567 -> "1234.567".
void append_microseconds(std::string& out, std::int64_t ns) {
    if (ns < 0) {
        out.push_back('-');
        ns = -ns;
    }
    append_integer(out, ns / 1000);
    const std::int64_t fraction = ns % 1000;
    out.push_back('.');
    out.push_back(static_cast<char>('0' + fraction / 100));
    out.push_back(static_cast<char>('0' + (fraction / 10) % 10));
    out.push_back(static_cast<char>('0' + fraction % 10));
}

void append_string(std::string& out, std::string_view text) {
    constexpr std::string_view hex = "0123456789abcdef";
    out.push_back('"');
    for (const char c : text) {
        const std::size_t byte = static_cast<unsigned char>(c);
        if (c == '"' || c == '\\') {
            out.push_back('\\');
            out.push_back(c);
        } else if (byte < 0x20U) {
            out.append(R"(\u00)");
            out.push_back(hex[byte >> 4U]);
            out.push_back(hex[byte & 0x0FU]);
        } else {
            out.push_back(c);
        }
    }
    out.push_back('"');
}

// {"name":<name>,"ph":"M","pid":1,"tid":<tid>,"args":{"name":<value>}}
void append_metadata(std::string& out, std::string_view name, std::uint32_t tid,
                     std::string_view value) {
    out.append(R"({"name":)");
    append_string(out, name);
    out.append(R"(,"ph":"M","pid":1,"tid":)");
    append_integer(out, tid);
    out.append(R"(,"args":{"name":)");
    append_string(out, value);
    out.append("}}");
}

void append_event(std::string& out, const ZoneEvent& event) {
    out.append(R"({"name":)");
    append_string(out, event.name);
    out.append(R"(,"cat":"zone","ph":"X","pid":1,"tid":)");
    append_integer(out, event.thread);
    out.append(R"(,"ts":)");
    append_microseconds(out, event.start_ns);
    out.append(R"(,"dur":)");
    append_microseconds(out, event.duration_ns);
    out.push_back('}');
}

void append_total(std::string& out, const ZoneTotal& total) {
    const auto count = static_cast<std::int64_t>(total.count);
    out.append(R"({"name":)");
    append_string(out, total.name);
    out.append(R"(,"count":)");
    append_integer(out, count);
    out.append(R"(,"total_us":)");
    append_microseconds(out, total.total_ns);
    out.append(R"(,"mean_us":)");
    append_microseconds(out, count > 0 ? total.total_ns / count : 0);
    out.append(R"(,"min_us":)");
    append_microseconds(out, total.min_ns);
    out.append(R"(,"max_us":)");
    append_microseconds(out, total.max_ns);
    out.push_back('}');
}

// "key":"value" pairs of otherData (strings only, as Chrome writes them).
void append_other(std::string& out, std::string_view key, std::string_view value) {
    append_string(out, key);
    out.push_back(':');
    append_string(out, value);
}

std::string thread_label(std::uint32_t thread) {
    return "thread " + std::to_string(thread);
}

std::uint32_t thread_count(const Capture& capture) {
    std::uint32_t threads = 0;
    for (const ZoneEvent& event : capture.events) {
        threads = event.thread > threads ? event.thread : threads;
    }
    return threads;
}

void append_trace_events(std::string& out, const Capture& capture, std::string_view process) {
    out.append(R"("traceEvents":[)");
    out.push_back('\n');
    append_metadata(out, "process_name", 0, process);
    const std::uint32_t threads = thread_count(capture);
    for (std::uint32_t thread = 1; thread <= threads; ++thread) {
        out.append(",\n");
        append_metadata(out, "thread_name", thread, thread_label(thread));
    }
    for (const ZoneEvent& event : capture.events) {
        out.append(",\n");
        append_event(out, event);
    }
    out.append("\n]");
}

void append_other_data(std::string& out, const Capture& capture) {
    std::string duration;
    append_microseconds(duration, capture.duration_ns);
    out.append(R"("otherData":{)");
    append_other(out, "format", kFormatName);
    out.push_back(',');
    append_other(out, "version", kFormatVersion);
    out.push_back(',');
    append_other(out, "zones_compiled", zones_compiled() ? "true" : "false");
    out.push_back(',');
    append_other(out, "tracy", tracy_enabled() ? "true" : "false");
    out.push_back(',');
    append_other(out, "duration_us", duration);
    out.push_back(',');
    append_other(out, "events", std::to_string(capture.events.size()));
    out.push_back(',');
    append_other(out, "dropped", std::to_string(capture.dropped));
    out.push_back('}');
}

void append_totals(std::string& out, const Capture& capture) {
    out.append(R"("zoneTotals":[)");
    for (std::size_t i = 0; i < capture.totals.size(); ++i) {
        out.append(i == 0 ? "\n" : ",\n");
        append_total(out, capture.totals[i]);
    }
    out.append("\n]");
}

} // namespace

std::string chrome_trace_json(const Capture& capture, std::string_view process) {
    std::string out;
    out.reserve(256 + capture.events.size() * 96);
    out.append(R"({"displayTimeUnit":"ms",)");
    out.push_back('\n');
    append_trace_events(out, capture, process);
    out.append(",\n");
    append_other_data(out, capture);
    out.append(",\n");
    append_totals(out, capture);
    out.append("\n}\n");
    return out;
}

std::optional<std::string> write_chrome_trace(const std::filesystem::path& path,
                                              std::string_view process) {
    const std::string text = chrome_trace_json(capture_snapshot(), process);
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!file) {
        return std::string("cannot open the file for writing");
    }
    file.write(text.data(), static_cast<std::streamsize>(text.size()));
    file.close();
    if (!file) {
        return std::string("writing the file failed");
    }
    return std::nullopt;
}

} // namespace orrery::profile
