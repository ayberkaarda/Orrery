#pragma once

#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <imgui.h>
#include <imgui_internal.h>

#include "../render/gl_test_support.h"
#include "app_test_support.h"
#include "key_events.h"
#include "orrery/app/cli.h"
#include "orrery/app/config.h"
#include "orrery/app/window_app.h"
#include "orrery/render/png.h"
#include "orrery/render/ui_context.h"
#include "orrery/sim/space_experiment.h"

// Scripted sessions of the real window (docs/app.md, "Scripted window
// sessions"): orrery::app::run_window with a hidden window, driven frame by
// frame. Input is queued on the window's Dear ImGui context with the calls
// the GLFW backend's callbacks make (key_events.h, AddMousePosEvent,
// AddMouseButtonEvent); each step then runs frames until its condition
// holds on the window's state. Frames are paced at 60 per second, like a
// window with vsync on a 60 Hz display.
namespace orrery::app_test {

using Clock = std::chrono::steady_clock;
using app::WindowControl;
using app::WindowFrame;

using Input = std::function<void(WindowControl&)>;
using Condition = std::function<bool(const WindowFrame&)>;

struct Step {
    std::string name;
    // Queued one per frame, from the step's first frame on.
    std::vector<Input> inputs;
    // Checked after every frame once the inputs are queued: the step is done
    // when it holds.
    Condition done;
    std::int64_t max_frames = 600;
    // Queued before every frame of the step, after `inputs`.
    Input every_frame;
};

// With ORRERY_WINDOW_SESSION_SHOTS=<dir> set, writes the frame (the window's
// back buffer) to <dir>/<name>.png, for looking at the states a session
// passes through; otherwise does nothing.
inline void save_frame(const WindowFrame& frame, std::string name) {
    const std::optional<std::string> dir = render::test::environment("ORRERY_WINDOW_SESSION_SHOTS");
    if (!dir || dir->empty() || !frame.drawn) {
        return;
    }
    for (char& c : name) {
        const bool plain =
            (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9');
        c = plain ? c : '_';
    }
    std::filesystem::create_directories(*dir);
    const render::ImageRgba8 image = frame.gl->read_window_pixels(frame.width, frame.height);
    CHECK(render::write_png(std::filesystem::path(*dir) / (name + ".png"), image).has_value());
}

class Session final : public app::WindowDriver {
public:
    Session& then(Step step) {
        steps_.push_back(std::move(step));
        return *this;
    }

    // Called after every frame, whatever the step (monitors).
    void watch(Condition monitor) { monitors_.push_back(std::move(monitor)); }

    void before_frame(std::int64_t /*index*/, WindowControl& control) override {
        const Clock::time_point now = Clock::now();
        if (next_frame_ > now) {
            std::this_thread::sleep_until(next_frame_);
        }
        next_frame_ = std::max(next_frame_, now) + std::chrono::nanoseconds(16'666'667);
        if (current_ >= steps_.size()) {
            control.close = true;
            return;
        }
        Step& step = steps_[current_];
        if (next_input_ < step.inputs.size()) {
            step.inputs[next_input_++](control);
        }
        if (step.every_frame) {
            step.every_frame(control);
        }
    }

    void after_frame(const WindowFrame& frame) override {
        ++frames_;
        drawn_ += frame.drawn ? 1 : 0;
        if (frame.gl_errors > 0 && gl_error_step_.empty()) {
            gl_error_step_ = current_ < steps_.size() ? steps_[current_].name : "after the script";
        }
        gl_errors_ = frame.gl_errors;
        frame.ui->make_current();
        for (const Condition& monitor : monitors_) {
            (void)monitor(frame);
        }
        if (current_ >= steps_.size()) {
            return;
        }
        Step& step = steps_[current_];
        ++step_frames_;
        if (next_input_ >= step.inputs.size() && step.done && step.done(frame)) {
            save_frame(frame, std::to_string(completed_.size() + 1) + "_" + step.name);
            completed_.push_back(step.name);
            ++current_;
            next_input_ = 0;
            step_frames_ = 0;
        } else if (step_frames_ > step.max_frames) {
            failed_ = step.name;
            current_ = steps_.size();
        }
    }

    [[nodiscard]] bool finished() const noexcept {
        return failed_.empty() && completed_.size() == steps_.size();
    }
    [[nodiscard]] const std::string& failed_step() const noexcept { return failed_; }
    [[nodiscard]] const std::vector<std::string>& completed() const noexcept { return completed_; }
    [[nodiscard]] std::int64_t frames() const noexcept { return frames_; }
    [[nodiscard]] std::int64_t drawn_frames() const noexcept { return drawn_; }
    [[nodiscard]] std::size_t gl_errors() const noexcept { return gl_errors_; }
    [[nodiscard]] const std::string& gl_error_step() const noexcept { return gl_error_step_; }

private:
    std::vector<Step> steps_;
    std::vector<Condition> monitors_;
    std::size_t current_ = 0;
    std::size_t next_input_ = 0;
    std::int64_t step_frames_ = 0;
    std::int64_t frames_ = 0;
    std::int64_t drawn_ = 0;
    std::size_t gl_errors_ = 0;
    std::string gl_error_step_;
    std::string failed_;
    std::vector<std::string> completed_;
    Clock::time_point next_frame_ = Clock::now();
};

struct SessionRun {
    int code = app::kExitFailure;
    std::string out;
    std::string err;
    bool skipped = false;
};

// Runs `session` in a hidden window opened from `request`. Without an
// OpenGL 4.6 context the test is skipped (a failure with ORRERY_REQUIRE_GL=1).
inline SessionRun run_session(app::WindowRequest request, Session& session) {
    std::ostringstream out;
    std::ostringstream err;
    app::WindowSession options;
    options.driver = &session;
    options.hidden = true;
    options.gl_debug = true;
    SessionRun run;
    run.code = app::run_window(std::move(request), out, err, options);
    run.out = out.str();
    run.err = err.str();
    if (run.code == app::kExitNoGlContext && !render::test::environment_flag("ORRERY_REQUIRE_GL")) {
        run.skipped = true;
    }
    return run;
}

// Checks a finished session: exit code 0, every step done, no OpenGL error,
// nothing on stderr.
inline void check_session(const SessionRun& run, const Session& session) {
    INFO("stderr: " << run.err);
    INFO("steps done: " << session.completed().size() << ", failed at: '" << session.failed_step()
                        << "'");
    CHECK(run.code == app::kExitOk);
    CHECK(session.failed_step().empty());
    CHECK(session.finished());
    INFO("first OpenGL error during: " << session.gl_error_step());
    CHECK(session.gl_errors() == 0);
    CHECK(run.err.empty());
}

inline std::filesystem::path session_dir(const char* name) {
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() / "orrery_test_window" / name;
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}

// The request main() builds for `--experiment <file>` of the Space
// workspace, with the default configuration and `dir` as experiments folder.
inline app::WindowRequest space_request(const std::filesystem::path& experiment_file,
                                        const std::filesystem::path& dir) {
    app::WindowRequest request;
    auto experiment = sim::load_space_experiment(experiment_file);
    REQUIRE(experiment.has_value());
    request.experiment = std::move(experiment).value();
    request.scenarios = sim::list_scenarios(request.experiment.scenario_path.parent_path());
    request.config = app::default_config();
    request.config.experiment_dir = dir;
    request.config.model_cache_dir = dir / "models";
    request.config_path = dir / "config.toml";
    request.bundled_experiment_dir = experiment_dir();
    request.workspace = app::Workspace::space;
    return request;
}

// --- Input -----------------------------------------------------------------

inline Input down(KeyStroke stroke) {
    return [stroke](WindowControl&) { key_down(stroke); };
}
inline Input repeat(KeyStroke stroke) {
    return [stroke](WindowControl&) { key_repeat(stroke); };
}
inline Input up(KeyStroke stroke) {
    return [stroke](WindowControl&) { key_up(stroke); };
}
inline Input nothing() {
    return [](WindowControl&) {};
}
// A key pressed in one frame and released in the next.
inline std::vector<Input> press(KeyStroke stroke) {
    return {down(stroke), up(stroke)};
}
// Typed text, one character per key press (no key position: characters
// only, as a layout types them).
inline std::vector<Input> type_text(std::string_view text) {
    std::vector<Input> inputs;
    for (const char c : text) {
        inputs.emplace_back([c](WindowControl&) {
            ImGui::GetIO().AddInputCharacter(static_cast<unsigned int>(c));
        });
    }
    return inputs;
}
inline Input mouse_at(float x, float y) {
    return [x, y](WindowControl&) { ImGui::GetIO().AddMousePosEvent(x, y); };
}
inline Input mouse_button(bool pressed) {
    return [pressed](WindowControl&) { ImGui::GetIO().AddMouseButtonEvent(0, pressed); };
}

// --- Dear ImGui state of the last frame ------------------------------------

inline ImGuiWindow* find_window(std::string_view prefix) {
    for (ImGuiWindow* window : ImGui::GetCurrentContext()->Windows) {
        if (std::string_view(window->Name).starts_with(prefix)) {
            return window;
        }
    }
    return nullptr;
}

// The docked window `name` (exact) is its node's shown tab.
inline bool tab_shown(std::string_view name) {
    const ImGuiWindow* window = ImGui::FindWindowByName(std::string(name).c_str());
    return window != nullptr && window->DockIsActive && window->DockNode != nullptr &&
           window->DockNode->VisibleWindow == window;
}

// A pointer position found by moving the pointer over an area, one probe
// per frame, until the frame under it satisfies `hit`.
struct Probe {
    std::optional<ImRect> area;
    ImVec2 at;
    float step = 4.0F;      // between rows [px]
    float x_offset = 24.0F; // of a single column from the area's left edge [px]
    bool found = false;
    bool exhausted = false;
};

// Scans `area_of(frame)` row by row (rows `step` pixels apart, `columns`
// probes per row across its width) until `hit` holds for the frame drawn
// with the pointer at a probe.
inline Step locate(std::string name, std::shared_ptr<Probe> probe,
                   std::function<std::optional<ImRect>()> area_of, std::function<bool()> hit,
                   int columns = 1) {
    Step step;
    step.name = std::move(name);
    step.max_frames = 2000;
    step.every_frame = [probe](WindowControl&) {
        if (probe->area && !probe->found) {
            ImGui::GetIO().AddMousePosEvent(probe->at.x, probe->at.y);
        }
    };
    step.done = [probe, area_of = std::move(area_of), hit = std::move(hit),
                 columns](const WindowFrame&) {
        if (!probe->area) {
            probe->area = area_of();
            if (probe->area) {
                probe->at = ImVec2(probe->area->Min.x + probe->x_offset, probe->area->Min.y + 1.0F);
            }
            return false;
        }
        if (hit()) {
            probe->found = true;
            return true;
        }
        const float dx = probe->area->GetWidth() / static_cast<float>(columns + 1);
        probe->at.x += columns > 1 ? dx : 0.0F;
        if (columns <= 1 || probe->at.x >= probe->area->Max.x - 1.0F) {
            probe->at.x = probe->area->Min.x + (columns > 1 ? dx : probe->x_offset);
            probe->at.y += probe->step;
        }
        if (probe->at.y >= probe->area->Max.y) {
            probe->exhausted = true;
            return true;
        }
        return false;
    };
    return step;
}

// The pointer at the probe's position (once it is found).
inline Input mouse_at(std::shared_ptr<const Probe> probe) {
    return [probe](WindowControl&) { ImGui::GetIO().AddMousePosEvent(probe->at.x, probe->at.y); };
}

inline std::optional<ImRect> window_area(std::string_view prefix) {
    const ImGuiWindow* window = find_window(prefix);
    if (window == nullptr || !window->WasActive) {
        return std::nullopt;
    }
    return window->Rect();
}

} // namespace orrery::app_test
