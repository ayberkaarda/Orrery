#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <ostream>
#include <string>
#include <thread>
#include <utility>

#include "orrery/app/aero_workspace.h"
#include "orrery/app/cli.h"
#include "orrery/app/entry_workspace.h"
#include "orrery/app/window_app.h"
#include "orrery/render/png.h"

// --capture (docs/screenshots.md): the window opened hidden, paced at 60
// frames per second, its framebuffer written after the UI pass.
namespace orrery::app {

namespace {

using Clock = std::chrono::steady_clock;

constexpr std::chrono::nanoseconds kFramePeriod{16'666'667};
// Frames drawn after a run finished before the image is taken: the view
// data refreshes and the layout settles.
constexpr std::int64_t kSettleFrames = 45;
// A run that has not finished by then fails the capture.
constexpr std::chrono::minutes kRunTimeout{20};

class CaptureDriver final : public WindowDriver {
public:
    CaptureDriver(std::int64_t frames, std::filesystem::path out_png, std::ostream& out,
                  std::ostream& err)
        : frames_(frames), out_png_(std::move(out_png)), out_(out), err_(err) {}

    void before_frame(std::int64_t /*index*/, WindowControl& control) override {
        // 60 frames per second, as on a 60 Hz display with vsync.
        const Clock::time_point now = Clock::now();
        if (next_frame_ > now) {
            std::this_thread::sleep_until(next_frame_);
        }
        next_frame_ = std::max(next_frame_, now) + kFramePeriod;
        control.close = done_;
    }

    void after_frame(const WindowFrame& frame) override {
        if (done_ || !frame.drawn || frame.index < frames_) {
            return;
        }
        if (!run_finished(frame)) {
            return;
        }
        if (++settled_ < kSettleFrames) {
            return;
        }
        write(frame);
        done_ = true;
    }

    [[nodiscard]] int code() const noexcept { return code_; }

private:
    // Starts the shown workspace's run once, then reports whether it ended.
    bool run_finished(const WindowFrame& frame) {
        if (frame.workspace == Workspace::space) {
            return true;
        }
        if (!run_start_) {
            run_start_ = Clock::now();
        }
        if (Clock::now() - *run_start_ > kRunTimeout) {
            fail(started_ ? "the run did not finish in time" : "the run could not be started");
            return false;
        }
        if (!started_) {
            started_ = frame.workspace == Workspace::aero ? start_aero(*frame.aero)
                                                          : frame.entry->start_run();
            return false;
        }
        if (frame.workspace == Workspace::aero) {
            const sim::AeroRunState state = frame.aero->progress().status.state;
            if (state == sim::AeroRunState::failed) {
                fail("the aero run failed: " + frame.aero->progress().status.message);
            }
            return state == sim::AeroRunState::finished;
        }
        const sim::EntryRunState state = frame.entry->progress().status.state;
        if (state == sim::EntryRunState::failed) {
            fail("the entry run failed: " + frame.entry->progress().status.message);
        }
        return state == sim::EntryRunState::finished;
    }

    static bool start_aero(AeroWorkspace& aero) {
        // The geometry is prepared in the background first.
        return aero.geometry_ready() && !aero.busy() && aero.start_run();
    }

    void write(const WindowFrame& frame) {
        const render::ImageRgba8 image = frame.gl->read_window_pixels(frame.width, frame.height);
        if (frame.gl_errors != 0) {
            err_ << "orrery_app: capture: " << frame.gl_errors << " OpenGL errors\n";
            code_ = kExitFailure;
            return;
        }
        if (auto written = render::write_png(out_png_, image); !written) {
            err_ << "orrery_app: capture: " << written.error() << '\n';
            code_ = kExitFailure;
            return;
        }
        out_ << "wrote " << out_png_.string() << " (" << frame.width << 'x' << frame.height
             << ", window frame " << frame.index << ")\n";
        code_ = kExitOk;
    }

    void fail(const std::string& message) {
        err_ << "orrery_app: capture: " << message << '\n';
        code_ = kExitFailure;
        done_ = true;
    }

    std::int64_t frames_;
    std::filesystem::path out_png_;
    std::ostream& out_;
    std::ostream& err_;
    Clock::time_point next_frame_ = Clock::now();
    std::optional<Clock::time_point> run_start_;
    bool started_ = false;
    std::int64_t settled_ = 0;
    bool done_ = false;
    int code_ = kExitFailure;
};

} // namespace

int capture_window(WindowRequest request, std::int64_t frames, const std::filesystem::path& out_png,
                   std::ostream& out, std::ostream& err) {
    CaptureDriver driver(frames, out_png, out, err);
    WindowSession session;
    session.driver = &driver;
    session.hidden = true;
    session.gl_debug = true;
    const int code = run_window(std::move(request), out, err, session);
    return code != kExitOk ? code : driver.code();
}

} // namespace orrery::app
