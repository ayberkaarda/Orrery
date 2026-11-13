#include "orrery/app/cli.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <set>
#include <system_error>
#include <utility>

namespace orrery::app {

namespace {

enum class Option : std::uint8_t {
    help,
    version,
    modules,
    headless,
    experiment,
    frame,
    out,
    size,
    benchmark,
    trace,
    capture,
};

struct OptionSpec {
    Option option = Option::help;
    std::string_view name;
    std::string_view short_name;
    bool takes_value = false;
};

constexpr std::array<OptionSpec, 11> kOptions{{
    {Option::help, "--help", "-h", false},
    {Option::version, "--version", "", false},
    {Option::modules, "--modules", "", false},
    {Option::headless, "--headless", "", false},
    {Option::experiment, "--experiment", "-e", true},
    {Option::frame, "--frame", "", true},
    {Option::out, "--out", "-o", true},
    {Option::size, "--size", "", true},
    {Option::benchmark, "--benchmark", "", true},
    {Option::trace, "--trace", "", true},
    {Option::capture, "--capture", "", true},
}};

const OptionSpec* find_option(std::string_view name) {
    for (const OptionSpec& spec : kOptions) {
        if (spec.name == name || (!spec.short_name.empty() && spec.short_name == name)) {
            return &spec;
        }
    }
    return nullptr;
}

std::optional<std::int64_t> parse_integer(std::string_view text) {
    std::int64_t value = 0;
    const char* const end = text.data() + text.size();
    const auto parsed = std::from_chars(text.data(), end, value);
    if (text.empty() || parsed.ec != std::errc{} || parsed.ptr != end) {
        return std::nullopt;
    }
    return value;
}

bool ends_with_png(const std::filesystem::path& path) {
    std::string extension = path.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return extension == ".png";
}

core::Result<std::pair<int, int>, std::string> parse_size(std::string_view text) {
    const std::size_t x = text.find_first_of("xX");
    const auto bad = [&] {
        return core::failure("--size: expected WIDTHxHEIGHT with each in [" +
                             std::to_string(kMinImageSize) + ", " + std::to_string(kMaxImageSize) +
                             "], got '" + std::string(text) + "'");
    };
    if (x == std::string_view::npos) {
        return bad();
    }
    const auto width = parse_integer(text.substr(0, x));
    const auto height = parse_integer(text.substr(x + 1));
    if (!width || !height || *width < kMinImageSize || *width > kMaxImageSize ||
        *height < kMinImageSize || *height > kMaxImageSize) {
        return bad();
    }
    return std::pair<int, int>(static_cast<int>(*width), static_cast<int>(*height));
}

core::Result<CliOptions, std::string> check_capture(const CliOptions& options,
                                                    const std::set<Option>& seen) {
    if (!seen.contains(Option::capture)) {
        return options;
    }
    if (options.mode == RunMode::headless) {
        return core::failure(std::string("--capture is not valid with --headless (use --out)"));
    }
    if (seen.contains(Option::benchmark)) {
        return core::failure(std::string("--capture is not valid with --benchmark"));
    }
    if (!ends_with_png(options.capture)) {
        return core::failure("--capture: the image is written as PNG; use a .png file name, got '" +
                             options.capture.string() + "'");
    }
    return options;
}

} // namespace

core::Result<CliOptions, std::string> parse_command_line(std::span<const std::string_view> args) {
    // --help and --version answer even when the rest of the line is wrong.
    for (const std::string_view arg : args) {
        if (arg == "--help" || arg == "-h") {
            CliOptions options;
            options.mode = RunMode::help;
            return options;
        }
    }
    for (const std::string_view arg : args) {
        if (arg == "--version") {
            CliOptions options;
            options.mode = RunMode::version;
            return options;
        }
    }

    CliOptions options;
    std::set<Option> seen;
    for (std::size_t i = 0; i < args.size(); ++i) {
        std::string_view arg = args[i];
        std::optional<std::string_view> inline_value;
        if (const std::size_t eq = arg.find('=');
            arg.starts_with("--") && eq != std::string_view::npos) {
            inline_value = arg.substr(eq + 1);
            arg = arg.substr(0, eq);
        }
        const OptionSpec* spec = find_option(arg);
        if (spec == nullptr) {
            if (!arg.starts_with("-")) {
                return core::failure("unexpected argument '" + std::string(arg) +
                                     "' (options start with --; see --help)");
            }
            return core::failure("unknown option '" + std::string(arg) + "' (see --help)");
        }
        if (!seen.insert(spec->option).second) {
            return core::failure("option " + std::string(spec->name) + " given more than once");
        }
        std::string_view value;
        if (spec->takes_value) {
            if (inline_value) {
                value = *inline_value;
            } else if (i + 1 < args.size()) {
                value = args[++i];
            } else {
                return core::failure("option " + std::string(spec->name) + " requires a value");
            }
            if (value.empty()) {
                return core::failure("option " + std::string(spec->name) +
                                     " requires a non-empty value");
            }
        } else if (inline_value) {
            return core::failure("option " + std::string(spec->name) + " takes no value");
        }

        switch (spec->option) {
            case Option::help:
            case Option::version:
                break;
            case Option::modules:
                options.mode = RunMode::modules;
                break;
            case Option::headless:
                options.mode = RunMode::headless;
                break;
            case Option::experiment:
                options.experiment = std::string(value);
                break;
            case Option::frame: {
                const auto frame = parse_integer(value);
                if (!frame || *frame < 0 || *frame > kMaxFrames) {
                    return core::failure("--frame: expected an integer in [0, " +
                                         std::to_string(kMaxFrames) + "], got '" +
                                         std::string(value) + "'");
                }
                options.frame = *frame;
                break;
            }
            case Option::out:
                options.out = std::filesystem::path(std::string(value));
                break;
            case Option::size: {
                const auto size = parse_size(value);
                if (!size) {
                    return core::failure(size.error());
                }
                options.width = size.value().first;
                options.height = size.value().second;
                break;
            }
            case Option::benchmark: {
                const auto frames = parse_integer(value);
                if (!frames || *frames < 1 || *frames > kMaxFrames) {
                    return core::failure("--benchmark: expected a frame count in [1, " +
                                         std::to_string(kMaxFrames) + "], got '" +
                                         std::string(value) + "'");
                }
                options.benchmark_frames = *frames;
                break;
            }
            case Option::trace:
                options.trace = std::filesystem::path(std::string(value));
                break;
            case Option::capture:
                options.capture = std::filesystem::path(std::string(value));
                break;
        }
    }

    if (seen.contains(Option::modules) && seen.size() > 1) {
        return core::failure(std::string("--modules takes no other options"));
    }
    const bool headless = options.mode == RunMode::headless;
    if (headless) {
        if (!options.experiment) {
            return core::failure(std::string("--headless requires --experiment <name-or-path>"));
        }
        if (options.out.empty()) {
            return core::failure(std::string("--headless requires --out <file.png>"));
        }
        if (!ends_with_png(options.out)) {
            return core::failure("--out: the image is written as PNG; use a .png file name, got '" +
                                 options.out.string() + "'");
        }
    } else {
        if (seen.contains(Option::frame) && !seen.contains(Option::capture)) {
            return core::failure(std::string("--frame is only valid with --headless or --capture"));
        }
        if (seen.contains(Option::out)) {
            return core::failure(std::string("--out is only valid with --headless"));
        }
    }
    return check_capture(options, seen);
}

std::string usage_text() {
    return "Usage:\n"
           "  orrery_app [--experiment <name-or-path>] [--size WxH] [--benchmark <frames>]\n"
           "  orrery_app --headless --experiment <name-or-path> --out <file.png>\n"
           "             [--frame <N>] [--size WxH] [--benchmark <frames>]\n"
           "  orrery_app [--experiment <name-or-path>] --capture <file.png> [--frame <N>]\n"
           "             [--size WxH]\n"
           "  orrery_app --version | --help | --modules\n"
           "\n"
           "Options:\n"
           "  -e, --experiment <x>  experiment file, or the name of a bundled experiment\n"
           "                        (assets/experiments/<x>.json), e.g. solar_system\n"
           "      --headless        render offscreen and write a PNG instead of opening a window\n"
           "      --frame <N>       advance N frames (1/60 s each at the experiment's time\n"
           "                        warp) before rendering; default 0\n"
           "  -o, --out <file.png>  output image of --headless\n"
           "      --capture <file.png>\n"
           "                        open the window hidden (never shown or focused), run\n"
           "                        --frame frames at 60 per second and the shown\n"
           "                        workspace's run, write the window's framebuffer with\n"
           "                        its panels to the PNG and exit\n"
           "      --size WxH        image or window size in pixels (default 1280x720 headless,\n"
           "                        1600x900 window)\n"
           "      --benchmark <N>   window: render N frames without vsync, print the mean\n"
           "                        frame time and exit; headless: also time N offscreen\n"
           "                        frames of the written view (GPU work included)\n"
           "      --trace <file>    write the run's profiling zones as Chrome trace JSON\n"
           "                        (zones need an ORRERY_PROFILING build)\n"
           "      --modules         print the linked module graph\n"
           "      --version         print the version\n"
           "  -h, --help            print this help\n"
           "\n"
           "Exit codes: 0 success, 1 failure, 2 invalid command line, 3 no OpenGL 4.6 context.\n";
}

} // namespace orrery::app
