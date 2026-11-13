#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "orrery/render/ui_app_bar.h"
#include "orrery/sim/sim_command.h"
#include "window_session.h"

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
// windows.h first: these headers use its types.
#include <psapi.h>
#include <tlhelp32.h>
#else
#include <unistd.h>
#endif

// The real window under load (docs/app.md, "Scripted window sessions"): the
// time-warp budget against a physics load far above it, the threads and the
// memory of whole sessions, and a one-minute soak.

using namespace orrery;
using namespace orrery::app_test;

namespace {

const KeyStroke kOpenBracket{ImGuiKey_LeftBracket, false, false, false, '['};
const KeyStroke kCloseBracket{ImGuiKey_RightBracket, false, false, false, ']'};
const KeyStroke kF1{ImGuiKey_F1};
const KeyStroke kCtrl1{ImGuiKey_1, true};
const KeyStroke kCtrl2{ImGuiKey_2, true};
const KeyStroke kCtrl3{ImGuiKey_3, true};

std::size_t thread_count() {
#if defined(_WIN32)
    const DWORD process = GetCurrentProcessId();
    HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
    REQUIRE(snapshot != INVALID_HANDLE_VALUE);
    THREADENTRY32 entry{};
    entry.dwSize = sizeof(entry);
    std::size_t count = 0;
    for (BOOL more = Thread32First(snapshot, &entry); more != FALSE;
         more = Thread32Next(snapshot, &entry)) {
        count += entry.th32OwnerProcessID == process ? 1U : 0U;
    }
    CloseHandle(snapshot);
    return count;
#else
    std::size_t count = 0;
    for ([[maybe_unused]] const auto& task :
         std::filesystem::directory_iterator("/proc/self/task")) {
        ++count;
    }
    return count;
#endif
}

// Memory the process has committed for itself [bytes].
std::size_t private_bytes() {
#if defined(_WIN32)
    PROCESS_MEMORY_COUNTERS_EX counters{};
    counters.cb = sizeof(counters);
    REQUIRE(GetProcessMemoryInfo(GetCurrentProcess(),
                                 reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&counters),
                                 sizeof(counters)) != FALSE);
    return counters.PrivateUsage;
#else
    std::ifstream statm("/proc/self/statm");
    std::size_t pages = 0;
    std::size_t resident = 0;
    statm >> pages >> resident;
    return resident * static_cast<std::size_t>(sysconf(_SC_PAGESIZE));
#endif
}

// A space experiment of the bundled solar system with a tiny step: at
// 1e7x it would need about 1e7 steps per second of wall time, far beyond
// the physics budget.
std::filesystem::path heavy_experiment(const std::filesystem::path& dir) {
    const std::filesystem::path path = dir / "heavy_solar_system.json";
    std::ofstream out(path);
    out << R"({ "version": 1, "domain": "space",
  "inputs": { "scenario": ")"
        << (scenario_dir() / "solar_system_j2000.json").generic_string() << R"(" },
  "conditions": { "time_warp": 100000 },
  "solver": { "name": "leapfrog_kdk", "version": "1",
              "settings": { "dt_days": 1e-05, "force": "auto" } },
  "result": {} })";
    return path;
}

// A key held for `repeats` operating-system repeats, one every other frame.
std::vector<Input> hold(KeyStroke stroke, int repeats) {
    std::vector<Input> inputs{down(stroke), nothing()};
    for (int i = 0; i < repeats; ++i) {
        inputs.push_back(repeat(stroke));
        inputs.push_back(nothing());
    }
    inputs.push_back(up(stroke));
    return inputs;
}

std::int64_t warp_steps(std::int64_t warp, int levels) {
    for (int i = 0; i < std::abs(levels); ++i) {
        warp = render::step_time_warp(warp, levels > 0 ? +1 : -1);
    }
    return warp;
}

struct ChipWatch {
    bool counting = false;
    std::optional<bool> last;
    int changes = 0;
};

} // namespace

TEST_CASE("The warp budget holds a heavy load down and lets go of it", "[app][window][gl]") {
    const std::filesystem::path dir = session_dir("governor");
    auto chip = std::make_shared<ChipWatch>();
    Session session;
    session.watch([chip](const WindowFrame& f) {
        if (chip->counting && f.drawn) {
            if (chip->last && *chip->last != f.warp_limited_shown) {
                ++chip->changes;
            }
            chip->last = f.warp_limited_shown;
        }
        return true;
    });
    session.then(Step{"the heavy system runs", {}, [](const WindowFrame& f) {
                          return f.drawn && f.status->steps > 1000 && !f.status->paused;
                      }});
    Step raise;
    raise.name = "a held ] requests 1e7x";
    raise.inputs = hold(kCloseBracket, 5);
    raise.done = [](const WindowFrame& f) { return f.status->time_warp == sim::kMaxTimeWarp; };
    session.then(raise);
    Step limited;
    limited.name = "the warp is reduced and the LIMITED chip is shown";
    limited.done = [](const WindowFrame& f) {
        return f.status->warp_reduced && f.status->governed_warp < sim::kMaxTimeWarp &&
               f.warp_limited_shown && f.status->effective_warp > 0.0 &&
               f.status->effective_warp < 0.5 * static_cast<double>(sim::kMaxTimeWarp);
    };
    limited.max_frames = 600;
    session.then(limited);
    auto governed = std::make_shared<std::int64_t>(0);
    Step settle;
    settle.name = "it settles";
    settle.done = [governed](const WindowFrame& f) {
        static_cast<void>(f);
        *governed = f.status->governed_warp;
        return true;
    };
    session.then(settle);
    auto seen = std::make_shared<int>(0);
    auto governed_changes = std::make_shared<int>(0);
    // Frames between two changes of the governed warp: the governor holds a
    // drop for 0.25 s and a rise for 1.5 s, so a change every few frames
    // would be flicker. The machine's other load may move the level itself.
    auto shortest_gap = std::make_shared<int>(1000);
    auto last_change = std::make_shared<std::optional<int>>();
    Step steady;
    steady.name = "and holds for five seconds without flicker";
    steady.inputs = {[chip](WindowControl&) {
        chip->counting = true;
        chip->last.reset();
    }};
    steady.done = [chip, seen, governed, governed_changes, shortest_gap,
                   last_change](const WindowFrame& f) {
        if (f.status->governed_warp != *governed) {
            ++*governed_changes;
            *governed = f.status->governed_warp;
            if (*last_change) {
                *shortest_gap = std::min(*shortest_gap, *seen - **last_change);
            }
            *last_change = *seen;
        }
        CHECK(f.warp_limited_shown);
        CHECK(f.status->warp_reduced);
        return ++*seen >= 300;
    };
    steady.max_frames = 310;
    session.then(steady);
    Step lower;
    lower.name = "a held [ lowers the request to 1000x";
    lower.inputs = hold(kOpenBracket, 11);
    lower.inputs.insert(lower.inputs.begin(), [chip](WindowControl&) { chip->counting = false; });
    lower.done = [](const WindowFrame& f) {
        return f.status->time_warp == warp_steps(sim::kMaxTimeWarp, -12);
    };
    session.then(lower);
    Step recovered;
    recovered.name = "the full warp comes back and the chip goes away";
    recovered.done = [](const WindowFrame& f) {
        return !f.status->warp_reduced && !f.warp_limited_shown &&
               f.status->governed_warp == f.status->time_warp;
    };
    recovered.max_frames = 600;
    session.then(recovered);
    auto after = std::make_shared<int>(0);
    Step stays;
    stays.name = "and stays away";
    stays.inputs = {[chip](WindowControl&) {
        chip->counting = true;
        chip->last.reset();
    }};
    stays.done = [after](const WindowFrame& f) {
        CHECK_FALSE(f.warp_limited_shown);
        CHECK(f.status->effective_warp > 0.8 * static_cast<double>(f.status->time_warp));
        return ++*after >= 180;
    };
    stays.max_frames = 190;
    session.then(stays);

    const SessionRun run = run_session(space_request(heavy_experiment(dir), dir), session);
    if (run.skipped) {
        SKIP("no OpenGL 4.6 context for a hidden window: " << run.err);
    }
    check_session(run, session);
    CHECK(chip->changes == 0);
    CHECK(*governed_changes <= 4);
    CHECK(*shortest_gap >= 15);
    std::cout << "governor: held at " << *governed << "x for 1e7x requested; chip changes "
              << chip->changes << ", governed warp changes " << *governed_changes
              << " (shortest gap " << *shortest_gap << " frames)\n";
}

TEST_CASE("Sessions end with every thread joined and exit code 0", "[app][window][gl]") {
    const std::filesystem::path dir = session_dir("threads");
    const auto visit_all = [] {
        auto session = std::make_shared<Session>();
        session->then(Step{"Space", {}, [](const WindowFrame& f) { return f.index >= 5; }});
        session->then(Step{"Aero", press(kCtrl2), [](const WindowFrame& f) {
                               return f.workspace == app::Workspace::aero;
                           }});
        session->then(Step{"Entry", press(kCtrl3), [](const WindowFrame& f) {
                               return f.workspace == app::Workspace::entry;
                           }});
        session->then(Step{"Space again", press(kCtrl1), [](const WindowFrame& f) {
                               return f.workspace == app::Workspace::space;
                           }});
        return session;
    };
    // The first session starts the driver's own threads, which stay.
    auto warm_up = visit_all();
    const SessionRun first =
        run_session(space_request(experiment_dir() / "solar_system.json", dir), *warm_up);
    if (first.skipped) {
        SKIP("no OpenGL 4.6 context for a hidden window: " << first.err);
    }
    check_session(first, *warm_up);
    const std::size_t before = thread_count();
    for (int i = 0; i < 3; ++i) {
        auto again = visit_all();
        const SessionRun run =
            run_session(space_request(experiment_dir() / "solar_system.json", dir), *again);
        check_session(run, *again);
    }
    const std::size_t after = thread_count();
    std::cout << "threads after one session " << before << ", after three more " << after << '\n';
    CHECK(after <= before);
}

TEST_CASE("A one-minute soak keeps the memory flat", "[app][window][gl][soak]") {
    const std::filesystem::path dir = session_dir("soak");
    constexpr std::int64_t kFrames = 3600; // 60 s at 60 frames per second
    auto early = std::make_shared<std::size_t>(0);
    auto late = std::make_shared<std::size_t>(0);
    auto frames = std::make_shared<std::int64_t>(0);
    Session session;
    Step soak;
    soak.name = "60 seconds with the physics thread running";
    soak.every_frame = [frames](WindowControl&) {
        // Open and close the help now and then: windows come and go.
        if (*frames % 600 == 300) {
            key_down(kF1);
        } else if (*frames % 600 == 301) {
            key_up(kF1);
        }
    };
    soak.done = [early, late, frames](const WindowFrame& f) {
        ++*frames;
        if (*frames == 900) {
            *early = private_bytes(); // trails and plots full by now
        }
        if (*frames == kFrames) {
            *late = private_bytes();
            CHECK(f.status->steps > 0);
            CHECK_FALSE(f.status->paused);
            return true;
        }
        return false;
    };
    soak.max_frames = kFrames + 10;
    session.then(soak);

    const SessionRun run =
        run_session(space_request(experiment_dir() / "solar_system.json", dir), session);
    if (run.skipped) {
        SKIP("no OpenGL 4.6 context for a hidden window: " << run.err);
    }
    check_session(run, session);
    CHECK(session.drawn_frames() >= kFrames);
    const double growth_mb =
        (static_cast<double>(*late) - static_cast<double>(*early)) / (1024.0 * 1024.0);
    std::cout << "soak: private memory " << static_cast<double>(*early) / (1024.0 * 1024.0)
              << " MB after 15 s, " << static_cast<double>(*late) / (1024.0 * 1024.0)
              << " MB after 60 s (" << growth_mb << " MB)\n";
    CHECK(growth_mb < 16.0);
}
