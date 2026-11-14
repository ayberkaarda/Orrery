#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "orrery/app/aero_workspace.h"
#include "orrery/app/entry_workspace.h"
#include "orrery/app/shortcut_input.h"
#include "orrery/app/space_experiment_panel.h"
#include "orrery/core/sim_snapshot.h"
#include "orrery/render/ui_app_bar.h"
#include "orrery/render/ui_panels.h"
#include "orrery/sim/simulation.h"
#include "window_session.h"

// The Space workspace through the real window loop with a hidden window
// (docs/app.md, "Scripted window sessions"): the keyboard shortcuts, the
// help overlay, the workspace switches, a text field, key repeat, saving
// and opening an experiment, and the window's size changes.

using namespace orrery;
using namespace orrery::app_test;
using app::Workspace;

namespace {

const KeyStroke kSpace{ImGuiKey_Space, false, false, false, ' '};
const KeyStroke kEscape{ImGuiKey_Escape};
const KeyStroke kEnter{ImGuiKey_Enter};
const KeyStroke kF1{ImGuiKey_F1};
const KeyStroke kCtrlA{ImGuiKey_A, true};
const KeyStroke kCtrlS{ImGuiKey_S, true};
const KeyStroke kCtrlO{ImGuiKey_O, true};
const KeyStroke kCtrl1{ImGuiKey_1, true};
const KeyStroke kCtrl2{ImGuiKey_2, true};
const KeyStroke kCtrl3{ImGuiKey_3, true};
const KeyStroke kQuestion{ImGuiKey_Slash, false, true, false, '?'};
const KeyStroke kOpenBracket{ImGuiKey_LeftBracket, false, false, false, '['};
const KeyStroke kCloseBracket{ImGuiKey_RightBracket, false, false, false, ']'};

// What the steps remember of the window between frames.
struct Memory {
    bool paused = false;
    std::int64_t warp = 0;
    std::uint64_t steps = 0;
    std::uint64_t generation = 0;
    std::int64_t frames = 0;
    std::filesystem::path saved;
    std::int64_t saved_warp = 0;
    // The loaded system, paused: its step count and body positions.
    std::uint64_t loaded_steps = 0;
    std::vector<core::Vec3<core::space::Length>> loaded_positions;
    std::vector<std::string> notes;
};

Step press_step(std::string name, KeyStroke stroke, Condition done, std::int64_t max_frames = 120) {
    Step step;
    step.name = std::move(name);
    step.inputs = press(stroke);
    step.done = std::move(done);
    step.max_frames = max_frames;
    return step;
}

// Holds frames until `count` have passed in the step, then checks `check`.
Step wait_frames(std::string name, std::int64_t count, Condition check) {
    auto seen = std::make_shared<std::int64_t>(0);
    Step step;
    step.name = std::move(name);
    step.done = [seen, count, check = std::move(check)](const WindowFrame& frame) {
        return ++*seen >= count && check(frame);
    };
    step.max_frames = count + 2;
    return step;
}

std::int64_t warp_steps(std::int64_t warp, int levels) {
    for (int i = 0; i < std::abs(levels); ++i) {
        warp = render::step_time_warp(warp, levels > 0 ? +1 : -1);
    }
    return warp;
}

// A key held for `repeats` operating-system key repeats (one every other
// frame, about 30 per second), then released.
std::vector<Input> hold(KeyStroke stroke, int repeats) {
    std::vector<Input> inputs{down(stroke), nothing()};
    for (int i = 0; i < repeats; ++i) {
        inputs.push_back(repeat(stroke));
        inputs.push_back(nothing());
    }
    inputs.push_back(up(stroke));
    return inputs;
}

void add_shortcut_steps(Session& session, const std::shared_ptr<Memory>& memory) {
    session.then(press_step("Space toggles the run", kSpace, [memory](const WindowFrame& f) {
        return f.status->paused != memory->paused;
    }));
    session.then(press_step("Space toggles it back", kSpace, [memory](const WindowFrame& f) {
        return f.status->paused == memory->paused;
    }));
    session.then(
        press_step("] raises the warp one level", kCloseBracket, [memory](const WindowFrame& f) {
            return f.status->time_warp == warp_steps(memory->warp, 1);
        }));
    session.then(press_step("[ lowers it again", kOpenBracket, [memory](const WindowFrame& f) {
        return f.status->time_warp == memory->warp;
    }));
    Step held;
    held.name = "a held ] raises the warp on every key repeat";
    held.inputs = hold(kCloseBracket, 3);
    held.done = [memory](const WindowFrame& f) {
        return f.status->time_warp == warp_steps(memory->warp, 4);
    };
    session.then(held);
    session.then(
        wait_frames("and no further after the release", 20, [memory](const WindowFrame& f) {
            CHECK(f.status->time_warp == warp_steps(memory->warp, 4));
            return true;
        }));
    Step held_space;
    held_space.name = "a held Space toggles the run once";
    held_space.inputs = hold(kSpace, 4);
    held_space.done = [memory](const WindowFrame& f) { return f.status->paused != memory->paused; };
    session.then(held_space);
    session.then(wait_frames("and stays toggled", 20, [memory](const WindowFrame& f) {
        CHECK(f.status->paused != memory->paused);
        return true;
    }));
    session.then(
        press_step("Space toggles it back once more", kSpace,
                   [memory](const WindowFrame& f) { return f.status->paused == memory->paused; }));
}

void add_overlay_steps(Session& session) {
    session.then(press_step("F1 opens the help", kF1, [](const WindowFrame& f) {
        return f.help_open && find_window(app::kShortcutHelpWindow) != nullptr;
    }));
    session.then(
        press_step("Esc closes it", kEscape, [](const WindowFrame& f) { return !f.help_open; }));
    session.then(
        press_step("? opens it", kQuestion, [](const WindowFrame& f) { return f.help_open; }));
    session.then(
        press_step("F1 closes it", kF1, [](const WindowFrame& f) { return !f.help_open; }));
    session.then(press_step("Ctrl+2 shows the Aero workspace", kCtrl2, [](const WindowFrame& f) {
        return f.workspace == Workspace::aero && tab_shown(app::kGeometryWindow);
    }));
    session.then(press_step("Ctrl+3 shows the Entry workspace", kCtrl3, [](const WindowFrame& f) {
        return f.workspace == Workspace::entry && tab_shown(app::kEntrySetupWindow);
    }));
    session.then(press_step("Ctrl+1 goes back to Space", kCtrl1, [](const WindowFrame& f) {
        return f.workspace == Workspace::space && tab_shown(render::kBodyInspectorWindow);
    }));
}

void add_text_field_steps(Session& session, const std::shared_ptr<Memory>& memory) {
    session.then(press_step("Ctrl+O shows the Experiment panel", kCtrlO, [](const WindowFrame&) {
        return tab_shown(render::kSpaceExperimentWindow);
    }));
    auto field = std::make_shared<Probe>();
    field->step = 3.0F;
    session.then(locate(
        "find the experiment name field", field,
        [] { return window_area(render::kSpaceExperimentWindow); },
        [] { return ImGui::GetMouseCursor() == ImGuiMouseCursor_TextInput; }, 6));
    Step click;
    click.name = "a click gives the name field the keyboard";
    click.inputs = {mouse_at(field), mouse_button(true), mouse_button(false)};
    click.done = [field](const WindowFrame&) {
        REQUIRE(field->found);
        return ImGui::GetIO().WantTextInput;
    };
    session.then(click);
    Step typing;
    typing.name = "shortcut keys type into the field instead";
    typing.inputs = press(kCtrlS);
    for (const Input& input : press(kQuestion)) {
        typing.inputs.push_back(input);
    }
    for (const Input& input : press(kF1)) {
        typing.inputs.push_back(input);
    }
    for (const Input& input : press(kSpace)) {
        typing.inputs.push_back(input);
    }
    typing.done = [memory](const WindowFrame& f) {
        CHECK(f.space_panel->saved().entries.empty());
        CHECK_FALSE(f.help_open);
        CHECK(f.status->paused == memory->paused);
        return f.space_panel->name().find('?') != std::string::npos;
    };
    session.then(typing);
    Step name;
    name.name = "type the experiment's name and press Enter";
    name.inputs = press(kCtrlA);
    for (const Input& input : type_text("smoke run")) {
        name.inputs.push_back(input);
    }
    for (const Input& input : press(kEnter)) {
        name.inputs.push_back(input);
    }
    name.done = [](const WindowFrame& f) {
        return f.space_panel->name() == "smoke run" && !ImGui::GetIO().WantTextInput;
    };
    session.then(name);
}

void add_save_open_steps(Session& session, const std::shared_ptr<Memory>& memory) {
    session.then(press_step("Ctrl+S saves the experiment", kCtrlS, [memory](const WindowFrame& f) {
        if (f.space_panel->saved().entries.size() != 1) {
            return false;
        }
        memory->saved = f.space_panel->saved().entries.front().path;
        memory->saved_warp = f.status->time_warp;
        memory->generation = f.status->generation;
        CHECK(memory->saved.filename() == "smoke_run.json");
        CHECK(f.space_panel->notice() == "Saved smoke_run.json");
        return true;
    }));
    session.then(press_step("] changes the warp before opening", kCloseBracket,
                            [memory](const WindowFrame& f) {
                                return f.status->time_warp == warp_steps(memory->saved_warp, 1);
                            }));
    session.then(press_step("Ctrl+O shows the saved experiment", kCtrlO, [](const WindowFrame& f) {
        return tab_shown(render::kSpaceExperimentWindow) &&
               f.space_panel->saved().entries.size() == 1;
    }));
    auto row = std::make_shared<Probe>();
    row->step = 3.0F;
    session.then(locate(
        "find its row", row, [] { return window_area("Experiment/##experiment_lists"); },
        [] { return ImGui::GetCurrentContext()->HoveredId != 0; }));
    Step open;
    open.name = "a double click opens it";
    open.inputs = {mouse_at(row), mouse_button(true), mouse_button(false), mouse_button(true),
                   mouse_button(false)};
    open.done = [memory, row](const WindowFrame& f) {
        REQUIRE(row->found);
        return f.status->generation != memory->generation &&
               f.status->experiment_file == memory->saved;
    };
    session.then(open);
    session.then(press_step("Space pauses the opened system", kSpace, [](const WindowFrame& f) {
        return f.status->paused && f.snapshot != nullptr;
    }));
    session.then(wait_frames("its state is published", 10, [memory](const WindowFrame& f) {
        CHECK(f.status->time_warp == memory->saved_warp);
        CHECK(f.status->message.empty());
        REQUIRE(f.snapshot != nullptr);
        memory->loaded_steps = f.status->steps;
        CHECK(f.snapshot->sequence() == f.status->steps);
        memory->loaded_positions.assign(f.snapshot->positions().begin(),
                                        f.snapshot->positions().end());
        return true;
    }));
}

// The saved experiment, opened in a fresh simulation and stepped as often
// as the window's was, has the same body positions bit for bit.
void check_reopened_state(const Memory& memory) {
    auto experiment = sim::load_space_experiment(memory.saved);
    REQUIRE(experiment.has_value());
    sim::SimulationConfig config = experiment.value().config;
    config.paused = true;
    auto simulation = sim::Simulation::create(experiment.value().scene, config);
    REQUIRE(simulation.has_value());
    sim::Simulation& sim = simulation.value();
    for (std::uint64_t i = 0; i < memory.loaded_steps; ++i) {
        sim.apply(sim::SingleStep{});
    }
    const auto snapshot = sim.snapshot();
    REQUIRE(snapshot->positions().size() == memory.loaded_positions.size());
    for (std::size_t i = 0; i < memory.loaded_positions.size(); ++i) {
        INFO("body " << i << " after " << memory.loaded_steps << " steps");
        CHECK(snapshot->positions()[i].x.value() == memory.loaded_positions[i].x.value());
        CHECK(snapshot->positions()[i].y.value() == memory.loaded_positions[i].y.value());
        CHECK(snapshot->positions()[i].z.value() == memory.loaded_positions[i].z.value());
    }
}

} // namespace

TEST_CASE("Space workspace through the hidden window", "[app][window][gl]") {
    const std::filesystem::path dir = session_dir("space");
    auto memory = std::make_shared<Memory>();
    Session session;
    Step first;
    first.name = "the first frames are drawn with the docked panels";
    first.done = [memory](const WindowFrame& f) {
        if (!f.drawn || f.index < 3 || f.snapshot == nullptr) {
            return false;
        }
        CHECK(f.width == 1600);
        CHECK(f.height == 900);
        CHECK(f.ui->window_docked(render::kBodyInspectorWindow));
        CHECK(f.ui->window_docked(render::kRenderSettingsWindow));
        CHECK(f.ui->window_docked(render::kSystemMonitorWindow));
        CHECK(f.ui->window_docked(render::kSpaceExperimentWindow));
        CHECK(tab_shown(render::kSystemMonitorWindow));
        memory->paused = f.status->paused;
        memory->warp = f.status->time_warp;
        return true;
    };
    session.then(first);
    add_shortcut_steps(session, memory);
    add_overlay_steps(session);
    add_text_field_steps(session, memory);
    add_save_open_steps(session, memory);

    const SessionRun run =
        run_session(space_request(experiment_dir() / "sun_earth_moon.json", dir), session);
    if (run.skipped) {
        SKIP("no OpenGL 4.6 context for a hidden window: " << run.err);
    }
    check_session(run, session);
    REQUIRE(session.finished());
    check_reopened_state(*memory);
}

TEST_CASE("The window's framebuffer holds the frame with its panels", "[app][window][gl]") {
    const std::filesystem::path dir = session_dir("pixels");
    Session session;
    Step look;
    look.name = "read the back buffer after a frame";
    look.done = [](const WindowFrame& f) {
        if (!f.drawn || f.index < 10) {
            return false;
        }
        const render::ImageRgba8 image = f.gl->read_window_pixels(f.width, f.height);
        REQUIRE(image.valid());
        CHECK(image.width == static_cast<std::uint32_t>(f.width));
        // The panels' background, the scene's black and the bar's text: the
        // frame is far from uniform.
        std::set<std::uint32_t> colours;
        for (std::size_t i = 0; i < image.pixels.size(); i += std::size_t{4} * 97U) {
            colours.insert(static_cast<std::uint32_t>(image.pixels[i]) << 16U |
                           static_cast<std::uint32_t>(image.pixels[i + 1]) << 8U |
                           image.pixels[i + 2]);
        }
        CHECK(colours.size() > 64);
        // The Body Inspector panel (left) is not the scene's black.
        const auto inspector = f.ui->window_rect(render::kBodyInspectorWindow);
        REQUIRE(inspector.has_value());
        const auto x = static_cast<std::uint32_t>(inspector->x + inspector->width * 0.5);
        const auto y = static_cast<std::uint32_t>(inspector->y + inspector->height * 0.9);
        const std::size_t at = image.offset(x, y);
        CHECK(image.pixels[at] + image.pixels[at + 1] + image.pixels[at + 2] > 15);

        // The Sun, framed by the window, is drawn at the centre of the scene
        // area the panels leave free (the dock space's central node), not at
        // the window's centre where the System Monitor would cover the lower
        // part of the system.
        const ImGuiWindow* docked = ImGui::FindWindowByName("Body Inspector");
        REQUIRE(docked != nullptr);
        REQUIRE(docked->DockNode != nullptr);
        const ImGuiDockNode* central = ImGui::DockNodeGetRootNode(docked->DockNode)->CentralNode;
        REQUIRE(central != nullptr);
        double sx = 0.0;
        double sy = 0.0;
        double count = 0.0;
        for (auto py = static_cast<std::uint32_t>(central->Pos.y);
             py < static_cast<std::uint32_t>(central->Pos.y + central->Size.y); ++py) {
            for (auto px = static_cast<std::uint32_t>(central->Pos.x);
                 px < static_cast<std::uint32_t>(central->Pos.x + central->Size.x); ++px) {
                const std::size_t i = image.offset(px, py);
                if (std::min({image.pixels[i], image.pixels[i + 1], image.pixels[i + 2]}) >= 250) {
                    sx += px;
                    sy += py;
                    count += 1.0;
                }
            }
        }
        REQUIRE(count > 20.0); // the Sun's disk and glare, not a few stars
        const double cx =
            static_cast<double>(central->Pos.x) + 0.5 * static_cast<double>(central->Size.x);
        const double cy =
            static_cast<double>(central->Pos.y) + 0.5 * static_cast<double>(central->Size.y);
        INFO("Sun at " << sx / count << ", " << sy / count << "; scene area centre " << cx << ", "
                       << cy);
        CHECK(std::abs(sx / count - cx) < 12.0);
        CHECK(std::abs(sy / count - cy) < 12.0);
        return true;
    };
    session.then(look);
    auto request = space_request(experiment_dir() / "solar_system.json", dir);
    request.width = 1280;
    request.height = 720;
    const SessionRun run = run_session(std::move(request), session);
    if (run.skipped) {
        SKIP("no OpenGL 4.6 context for a hidden window: " << run.err);
    }
    check_session(run, session);
}

TEST_CASE("Window size changes, an empty framebuffer and the content scale", "[app][window][gl]") {
    const std::filesystem::path dir = session_dir("sizes");
    Session session;
    Step first;
    first.name = "the first frame";
    first.done = [](const WindowFrame& f) {
        if (!f.drawn) {
            return false;
        }
        INFO("content scale " << f.content_scale.x << " x " << f.content_scale.y);
        CHECK(std::isfinite(f.content_scale.x));
        CHECK(f.content_scale.x > 0.0);
        CHECK(f.content_scale.y > 0.0);
        std::cout << "window content scale: " << f.content_scale.x << " x " << f.content_scale.y
                  << '\n';
        return f.width == 1280 && f.height == 720;
    };
    session.then(first);
    const auto resize = [](std::string name, int width, int height) {
        Step step;
        step.name = std::move(name);
        step.inputs = {[width, height](WindowControl& c) { c.window_size = {width, height}; }};
        step.done = [width, height](const WindowFrame& f) {
            return f.drawn && f.width == width && f.height == height;
        };
        return step;
    };
    // The window system may keep a window above a minimum size (Windows does
    // for a window with a title bar): the smallest one it gives is drawn.
    Step tiny;
    tiny.name = "the smallest window";
    tiny.inputs = {[](WindowControl& c) { c.window_size = std::pair{1, 1}; }, nothing(), nothing()};
    tiny.done = [](const WindowFrame& f) {
        std::cout << "smallest framebuffer: " << f.width << " x " << f.height << '\n';
        return f.drawn && f.width < 1280 && f.height < 720;
    };
    session.then(tiny);
    session.then(press_step("the Aero workspace at 1 x 1", kCtrl2, [](const WindowFrame& f) {
        return f.drawn && f.workspace == Workspace::aero;
    }));
    session.then(press_step("the Entry workspace at 1 x 1", kCtrl3, [](const WindowFrame& f) {
        return f.drawn && f.workspace == Workspace::entry;
    }));
    session.then(press_step("back to Space at 1 x 1", kCtrl1, [](const WindowFrame& f) {
        return f.drawn && f.workspace == Workspace::space;
    }));
    session.then(resize("a small window", 320, 200));
    session.then(resize("a tall window", 400, 1000));
    session.then(resize("a large window", 1600, 900));
    Step empty;
    empty.name = "an empty framebuffer draws nothing while physics runs on";
    empty.inputs.emplace_back([](WindowControl& c) { c.empty_framebuffer = true; });
    for (int i = 0; i < 40; ++i) {
        empty.inputs.emplace_back([](WindowControl& c) { c.empty_framebuffer = true; });
    }
    auto first_empty = std::make_shared<std::optional<std::uint64_t>>();
    session.watch([first_empty](const WindowFrame& f) {
        if (!f.drawn && !*first_empty) {
            *first_empty = f.status->steps;
        }
        return true;
    });
    empty.done = [first_empty](const WindowFrame& f) {
        REQUIRE(first_empty->has_value());
        CHECK_FALSE(f.drawn);
        CHECK(f.width == 0);
        return f.status->steps > **first_empty;
    };
    session.then(empty);
    Step restored;
    restored.name = "drawn again at the old size";
    restored.done = [](const WindowFrame& f) {
        return f.drawn && f.width == 1600 && f.height == 900;
    };
    session.then(restored);
    session.then(resize("and resized once more", 1280, 720));

    auto request = space_request(experiment_dir() / "sun_earth_moon.json", dir);
    request.width = 1280;
    request.height = 720;
    const SessionRun run = run_session(std::move(request), session);
    if (run.skipped) {
        SKIP("no OpenGL 4.6 context for a hidden window: " << run.err);
    }
    check_session(run, session);
}
