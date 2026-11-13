// Entry point of every benchmark executable: Catch2 runs the benchmarks
// (each a TEST_CASE; names and tags select them as usual), then the
// measurements are written as JSON to the file given with --json.
//
//   orrery_bench_core [--json results.json] [Catch2 options and filters]
#include <catch2/catch_session.hpp>

#include <cstdio>
#include <exception>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "bench_record.h"

#ifndef ORRERY_BENCH_SUITE
#define ORRERY_BENCH_SUITE "unnamed"
#endif
#ifndef ORRERY_BENCH_LTO
#define ORRERY_BENCH_LTO 0
#endif

namespace orrery::bench {
namespace {

struct Store {
    std::mutex mutex;
    std::vector<nlohmann::json> results;
    std::vector<std::string> devices;
};

Store& store() {
    static Store instance;
    return instance;
}

const char* goal_name(Goal goal) {
    switch (goal) {
        case Goal::below:
            return "below";
        case Goal::at_least:
            return "at_least";
        case Goal::info:
            break;
    }
    return "info";
}

const char* status_of(const Measurement& m) {
    switch (m.goal) {
        case Goal::below:
            return m.value < m.target ? "pass" : "fail";
        case Goal::at_least:
            return m.value >= m.target ? "pass" : "fail";
        case Goal::info:
            break;
    }
    return "info";
}

std::string compiler() {
#if defined(__clang__)
    return "clang " + std::to_string(__clang_major__) + "." + std::to_string(__clang_minor__);
#elif defined(__GNUC__)
    return "gcc " + std::to_string(__GNUC__) + "." + std::to_string(__GNUC_MINOR__);
#elif defined(_MSC_VER)
    return "msvc " + std::to_string(_MSC_VER);
#else
    return "unknown";
#endif
}

nlohmann::json document() {
    const std::scoped_lock lock(store().mutex);
    nlohmann::json doc;
    doc["schema"] = "orrery-bench/1";
    doc["suite"] = ORRERY_BENCH_SUITE;
    doc["build"] = {{"compiler", compiler()},
                    {"optimised", optimised_build()},
                    {"lto", ORRERY_BENCH_LTO != 0},
                    {"hardware_threads", std::thread::hardware_concurrency()}};
    doc["devices"] = store().devices;
    doc["results"] = store().results;
    return doc;
}

// Removes "--json <file>" (or "--json=<file>") from the arguments.
std::string take_json_option(std::vector<char*>& args) {
    std::string path;
    for (std::size_t i = 1; i < args.size();) {
        const std::string arg = args[i];
        if (arg == "--json" && i + 1 < args.size()) {
            path = args[i + 1];
            args.erase(args.begin() + static_cast<std::ptrdiff_t>(i),
                       args.begin() + static_cast<std::ptrdiff_t>(i + 2));
        } else if (arg.rfind("--json=", 0) == 0) {
            path = arg.substr(7);
            args.erase(args.begin() + static_cast<std::ptrdiff_t>(i));
        } else {
            ++i;
        }
    }
    return path;
}

bool write_json(const std::string& path) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out << document().dump(2) << '\n';
    out.close();
    if (!out) {
        std::cerr << "bench: cannot write " << path << '\n';
        return false;
    }
    std::cout << "bench: wrote " << path << '\n';
    return true;
}

int run(int argc, char** argv) {
    std::vector<char*> args(argv, argv + argc);
    const std::string json = take_json_option(args);
    Catch::Session session;
    const int code = session.run(static_cast<int>(args.size()), args.data());
    if (!json.empty() && !write_json(json)) {
        return code == 0 ? 1 : code;
    }
    return code;
}

} // namespace

void record(Measurement m) {
    const char* status = status_of(m);
    nlohmann::json entry = {{"id", m.id},      {"title", m.title},          {"value", m.value},
                            {"unit", m.unit},  {"goal", goal_name(m.goal)}, {"status", status},
                            {"notes", m.notes}};
    if (m.goal != Goal::info) {
        entry["target"] = m.target;
    }
    nlohmann::json context = nlohmann::json::object();
    for (const auto& [key, value] : m.context) {
        context[key] = value;
    }
    entry["context"] = std::move(context);
    std::cout << "bench: " << m.id << " = " << m.value << ' ' << m.unit << " [" << status << "]\n"
              << std::flush;
    const std::scoped_lock lock(store().mutex);
    store().results.push_back(std::move(entry));
}

void record_skipped(const std::string& id, const std::string& title, const std::string& reason) {
    std::cout << "bench: " << id << " skipped: " << reason << '\n' << std::flush;
    const std::scoped_lock lock(store().mutex);
    store().results.push_back(
        {{"id", id}, {"title", title}, {"status", "skipped"}, {"notes", reason}});
}

void record_device(const std::string& device) {
    const std::scoped_lock lock(store().mutex);
    if (std::find(store().devices.begin(), store().devices.end(), device) ==
        store().devices.end()) {
        store().devices.push_back(device);
    }
}

} // namespace orrery::bench

int main(int argc, char** argv) {
    try {
        return orrery::bench::run(argc, argv);
    } catch (const std::exception& e) {
        std::fputs("bench: fatal: ", stderr);
        std::fputs(e.what(), stderr);
        std::fputs("\n", stderr);
    } catch (...) {
        std::fputs("bench: fatal: unknown exception\n", stderr);
    }
    return 2;
}
