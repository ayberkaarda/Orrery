#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>

#include "orrery/core/result.h"

// Command line of orrery_app (docs/app.md, "Command line").
namespace orrery::app {

enum class RunMode : std::uint8_t {
    window,   // interactive window (default)
    headless, // render one frame offscreen and write a PNG
    help,
    version,
    modules, // print the linked module graph
};

struct CliOptions {
    RunMode mode = RunMode::window;
    // Experiment name or path (default for the window: the bundled solar
    // system experiment).
    std::optional<std::string> experiment;
    // Headless: frames advanced before the image is taken (60 per second of
    // wall-clock time at the experiment's time warp).
    std::int64_t frame = 0;
    std::filesystem::path out;
    // Image (headless) or window size [px].
    std::optional<int> width;
    std::optional<int> height;
    // Window: render this many frames without vsync, print the mean frame
    // time and exit (0: run until the window is closed).
    std::int64_t benchmark_frames = 0;
};

inline constexpr std::int64_t kMaxFrames = 1'000'000;
inline constexpr int kMinImageSize = 16;
inline constexpr int kMaxImageSize = 8192;

// Exit codes of orrery_app.
inline constexpr int kExitOk = 0;
inline constexpr int kExitFailure = 1;     // runtime failure (file, render, write)
inline constexpr int kExitUsage = 2;       // invalid command line
inline constexpr int kExitNoGlContext = 3; // no OpenGL 4.6 context could be created

// Parses the arguments after the program name. Options take their value as
// the next argument or after '=' (--frame=10). --help and --version win over
// everything else; otherwise every argument must be valid, no option may be
// repeated, and the combination must make sense (--frame and --out only with
// --headless, which requires --experiment and --out). The error is one line
// suitable for stderr.
[[nodiscard]] core::Result<CliOptions, std::string>
parse_command_line(std::span<const std::string_view> args);

// Multi-line usage text for --help.
[[nodiscard]] std::string usage_text();

} // namespace orrery::app
