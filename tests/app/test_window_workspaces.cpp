#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "orrery/app/aero_workspace.h"
#include "orrery/app/entry_workspace.h"
#include "orrery/sim/aero_comparison.h"
#include "orrery/sim/entry_experiments.h"
#include "window_session.h"

// The Aero and Entry workspaces through the real window loop with a hidden
// window (docs/app.md, "Scripted window sessions"): runs on their solver
// threads started, paused and stopped with the keys and the bar's buttons,
// saved and opened again.

using namespace orrery;
using namespace orrery::app_test;
using app::Workspace;

namespace {

const KeyStroke kSpace{ImGuiKey_Space, false, false, false, ' '};
const KeyStroke kCtrlS{ImGuiKey_S, true};

// Longest wait from a key or click to the solver thread's new state.
constexpr std::chrono::milliseconds kMaxLatency{500};

struct Timing {
    Clock::time_point sent;
    std::chrono::milliseconds worst{0};

    void note() {
        const auto latency =
            std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - sent);
        worst = std::max(worst, latency);
        CHECK(latency < kMaxLatency);
    }
};

Input timed(const std::shared_ptr<Timing>& timing, Input input) {
    return [timing, input = std::move(input)](WindowControl& control) {
        timing->sent = Clock::now();
        input(control);
    };
}

// A key press whose latency to `done` is measured.
Step timed_press(std::string name, const std::shared_ptr<Timing>& timing, KeyStroke stroke,
                 Condition done, std::int64_t max_frames = 120) {
    Step step;
    step.name = std::move(name);
    step.inputs = {timed(timing, down(stroke)), up(stroke)};
    step.done = [timing, done = std::move(done)](const WindowFrame& f) {
        if (!done(f)) {
            return false;
        }
        timing->note();
        return true;
    };
    step.max_frames = max_frames;
    return step;
}

// The main bar's `ordinal`-th item from the left (1: the first): the probe
// that hovers it. The bars of the Aero and Entry workspaces start with the
// run or pause button and the stop button after the workspace's name; the
// steps that click them check what the click did.
std::shared_ptr<Probe> bar_item(Session& session, const std::string& what, int ordinal) {
    auto probe = std::make_shared<Probe>();
    auto seen = std::make_shared<std::vector<ImGuiID>>();
    session.then(locate(
        "find the bar's " + what, probe,
        []() -> std::optional<ImRect> {
            const ImGuiWindow* bar = find_window("##MainMenuBar");
            if (bar == nullptr) {
                return std::nullopt;
            }
            const float y = bar->Rect().GetCenter().y;
            return ImRect(bar->Pos.x, y - 1.0F, bar->Pos.x + bar->Size.x, y + 1.0F);
        },
        [seen, ordinal] {
            const ImGuiID hovered = ImGui::GetCurrentContext()->HoveredId;
            if (hovered != 0 && (seen->empty() || seen->back() != hovered)) {
                seen->push_back(hovered);
            }
            return std::ssize(*seen) == ordinal && seen->back() == hovered;
        },
        400));
    return probe;
}

Step timed_click(std::string name, const std::shared_ptr<Timing>& timing,
                 const std::shared_ptr<Probe>& probe, Condition done) {
    Step step;
    step.name = std::move(name);
    step.inputs = {mouse_at(probe), timed(timing, mouse_button(true)), mouse_button(false)};
    step.done = [timing, probe, done = std::move(done)](const WindowFrame& f) {
        REQUIRE(probe->found);
        if (!done(f)) {
            return false;
        }
        timing->note();
        return true;
    };
    return step;
}

scene::AeroExperiment aero_experiment(const char* file) {
    auto experiment = sim::load_aero_experiment_file(experiment_dir() / file);
    REQUIRE(experiment.has_value());
    return std::move(experiment).value();
}

sim::AeroRunState aero_state(const WindowFrame& f) {
    return f.aero->progress().status.state;
}

sim::EntryRunState entry_state(const WindowFrame& f) {
    return f.entry->progress().status.state;
}

} // namespace

TEST_CASE("Aero workspace through the hidden window", "[app][window][gl]") {
    const std::filesystem::path dir = session_dir("aero");
    auto timing = std::make_shared<Timing>();
    auto cl = std::make_shared<double>(0.0);
    Session session;
    Step ready;
    ready.name = "the lattice Boltzmann NACA 0012 section is prepared";
    ready.done = [](const WindowFrame& f) {
        if (!f.drawn || !f.aero->geometry_ready() || f.aero->busy()) {
            return false;
        }
        CHECK(f.workspace == Workspace::aero);
        CHECK(f.ui->window_docked(app::kGeometryWindow));
        CHECK(f.ui->window_docked(app::kResultsWindow));
        return true;
    };
    ready.max_frames = 1200;
    session.then(ready);
    // Started is acknowledged by the solver thread: setting up (voxelising,
    // assembling) or already stepping.
    session.then(timed_press("Space starts the run", timing, kSpace, [](const WindowFrame& f) {
        return aero_state(f) == sim::AeroRunState::setting_up ||
               aero_state(f) == sim::AeroRunState::running;
    }));
    session.then(make_step("it steps", {}, [](const WindowFrame& f) {
        return aero_state(f) == sim::AeroRunState::running;
    }));
    session.then(timed_press("Space pauses it", timing, kSpace, [](const WindowFrame& f) {
        return aero_state(f) == sim::AeroRunState::paused;
    }));
    session.then(timed_press("Space resumes it", timing, kSpace, [](const WindowFrame& f) {
        return aero_state(f) == sim::AeroRunState::running;
    }));
    const auto stop = bar_item(session, "stop button (after run and step)", 3);
    session.then(timed_click("Stop ends the run", timing, stop, [](const WindowFrame& f) {
        return aero_state(f) == sim::AeroRunState::idle;
    }));

    Step panel;
    panel.name = "open the panel-method section";
    panel.done = [](const WindowFrame& f) {
        f.aero->open(aero_experiment("aero_naca0012_panel.json"), experiment_dir());
        return true;
    };
    session.then(panel);
    Step panel_ready = ready;
    panel_ready.name = "its geometry is prepared";
    panel_ready.done = [](const WindowFrame& f) {
        return f.aero->geometry_ready() && !f.aero->busy() &&
               f.aero->experiment().solver.solver == std::optional(aero::SolverKind::panel);
    };
    session.then(panel_ready);
    session.then(timed_press(
        "Space solves it", timing, kSpace,
        [cl](const WindowFrame& f) {
            if (aero_state(f) != sim::AeroRunState::finished || !f.aero->shown_result()) {
                return false;
            }
            *cl = f.aero->shown_result()->cl;
            return true;
        },
        600));
    Step save =
        timed_press("Ctrl+S saves it with its result", timing, kCtrlS, [dir](const WindowFrame&) {
            return sim::list_aero_experiments(dir).experiments.size() == 1;
        });
    session.then(save);
    Step reopen;
    reopen.name = "the saved experiment opens with its recorded result";
    reopen.done = [dir, cl](const WindowFrame& f) {
        const auto listing = sim::list_aero_experiments(dir);
        REQUIRE(listing.experiments.size() == 1);
        REQUIRE(listing.experiments.front().experiment.result.has_value());
        f.aero->open(listing.experiments.front().experiment, dir);
        const auto shown = f.aero->shown_result();
        REQUIRE(shown.has_value());
        CHECK(shown->recorded);
        CHECK(std::abs(shown->cl - *cl) <= 1.0e-12 * std::max(1.0, std::abs(*cl)));
        CHECK(*cl > 0.3); // NACA 0012 at 4 degrees: about 2 pi alpha
        return true;
    };
    session.then(reopen);
    session.then(make_step("the view shows it", {}, [](const WindowFrame& f) {
        return f.drawn && f.aero->geometry_ready() && !f.aero->busy();
    }));

    auto request = space_request(experiment_dir() / "sun_earth_moon.json", dir);
    request.aero_experiment = aero_experiment("aero_naca0012_lbm.json");
    request.aero_base_dir = experiment_dir();
    request.workspace = Workspace::aero;
    const SessionRun run = run_session(std::move(request), session);
    if (run.skipped) {
        SKIP("no OpenGL 4.6 context for a hidden window: " << run.err);
    }
    check_session(run, session);
    std::cout << "aero: worst key or click to solver-state latency " << timing->worst.count()
              << " ms\n";
}

TEST_CASE("Entry workspace through the hidden window", "[app][window][gl]") {
    const std::filesystem::path dir = session_dir("entry");
    auto timing = std::make_shared<Timing>();
    Session session;
    Step first;
    first.name = "the Apollo-like entry is shown";
    first.done = [](const WindowFrame& f) {
        if (!f.drawn || f.index < 3) {
            return false;
        }
        CHECK(f.workspace == Workspace::entry);
        CHECK(f.ui->window_docked(app::kEntrySetupWindow));
        CHECK(f.ui->window_docked(app::kEntryProfileWindow));
        return true;
    };
    session.then(first);
    session.then(timed_press("Space starts a run", timing, kSpace, [](const WindowFrame& f) {
        return entry_state(f) == sim::EntryRunState::running;
    }));
    session.then(timed_press("Space pauses it", timing, kSpace, [](const WindowFrame& f) {
        return entry_state(f) == sim::EntryRunState::paused;
    }));
    session.then(timed_press("Space resumes it", timing, kSpace, [](const WindowFrame& f) {
        return entry_state(f) == sim::EntryRunState::running;
    }));
    const auto stop = bar_item(session, "stop button", 2);
    session.then(timed_click("Stop ends it", timing, stop, [](const WindowFrame& f) {
        return entry_state(f) == sim::EntryRunState::stopped;
    }));
    Step to_ground = timed_press(
        "Space flies a new run to the ground", timing, kSpace,
        [](const WindowFrame& f) {
            return entry_state(f) == sim::EntryRunState::finished && f.entry->shown_trace();
        },
        3600);
    // The run takes seconds (its pace); only the start is a latency.
    to_ground.done = [](const WindowFrame& f) {
        return entry_state(f) == sim::EntryRunState::finished && f.entry->shown_trace();
    };
    session.then(to_ground);
    session.then(make_step(
        "Ctrl+S saves it", press(kCtrlS),
        [](const WindowFrame& f) {
            f.entry->refresh_experiments();
            return f.entry->experiments().entries.size() == 1;
        },
        120));
    Step compare;
    compare.name = "the saved run is compared and the table exported";
    compare.done = [](const WindowFrame& f) {
        REQUIRE(f.entry->experiments().entries.size() == 1);
        const sim::StoredEntry& saved = f.entry->experiments().entries.front();
        REQUIRE(saved.result != nullptr);
        f.entry->set_compared(0, true);
        const auto exported = f.entry->export_comparison();
        REQUIRE(exported.has_value());
        CHECK(std::filesystem::file_size(*exported) > 0);
        return true;
    };
    session.then(compare);
    session.then(
        make_step("the comparison is drawn", {}, [](const WindowFrame& f) { return f.drawn; }));

    auto request = space_request(experiment_dir() / "sun_earth_moon.json", dir);
    auto entry = sim::load_entry_experiment_file(experiment_dir() / "entry_apollo_like.json");
    REQUIRE(entry.has_value());
    request.entry_experiment = std::move(entry).value();
    request.workspace = Workspace::entry;
    const SessionRun run = run_session(std::move(request), session);
    if (run.skipped) {
        SKIP("no OpenGL 4.6 context for a hidden window: " << run.err);
    }
    check_session(run, session);
    std::cout << "entry: worst key or click to solver-state latency " << timing->worst.count()
              << " ms\n";
}
