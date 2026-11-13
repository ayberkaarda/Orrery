#include <cstddef>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <iostream>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "build_info.h"
#include "orrery/app/cli.h"
#include "orrery/app/config.h"
#include "orrery/app/error_surface.h"
#include "orrery/app/headless.h"
#include "orrery/app/run_guard.h"
#include "orrery/app/space_view.h"
#include "orrery/scene/aero_experiment.h"
#include "orrery/scene/experiment.h"
#include "orrery/scene/scenario.h"
#include "orrery/sim/entry_experiments.h"
#include "orrery/sim/space_experiment.h"

#if !defined(ORRERY_HEADLESS)
#include "window_app.h"
#endif

namespace {

void print_modules(std::ostream& out) {
    out << "modules:\n";
    for (const orrery::core::ModuleInfo& module : orrery::app::linked_modules()) {
        out << "  " << module.name << " ->";
        if (module.dependencies.empty()) {
            out << " (none)";
        }
        for (const std::string_view dependency : module.dependencies) {
            out << ' ' << dependency;
        }
        out << '\n';
    }
}

#if !defined(ORRERY_HEADLESS)
// The aero experiment at `path`, or nothing when the file is not one.
std::optional<orrery::scene::AeroExperiment> aero_experiment_at(const std::filesystem::path& path,
                                                                std::string& error) {
    using namespace orrery;
    const auto text = scene::read_text_file(path);
    if (!text) {
        return std::nullopt;
    }
    const auto generic = scene::parse_experiment(text.value());
    if (!generic || generic.value().domain != scene::Domain::aero) {
        return std::nullopt;
    }
    auto typed = scene::aero_from_experiment(generic.value());
    if (!typed) {
        error = "experiment " + path.string() + ": " + typed.error();
        return std::nullopt;
    }
    return std::move(typed).value();
}

// The entry experiment at `path`, or nothing when the file is not one.
std::optional<orrery::sim::StoredEntry> entry_experiment_at(const std::filesystem::path& path,
                                                            std::string& error) {
    using namespace orrery;
    const auto text = scene::read_text_file(path);
    if (!text) {
        return std::nullopt;
    }
    const auto generic = scene::parse_experiment(text.value());
    if (!generic || generic.value().domain != scene::Domain::entry) {
        return std::nullopt;
    }
    auto typed = sim::entry_from_experiment(generic.value());
    if (!typed) {
        error = "experiment " + path.string() + ": " + typed.error();
        return std::nullopt;
    }
    return std::move(typed).value();
}

int run_window_mode(const orrery::app::CliOptions& options,
                    const std::filesystem::path& executable) {
    using namespace orrery;
    app::AppConfig config = app::default_config();
    const std::filesystem::path config_path = app::default_config_path();
    if (auto loaded = app::load_config(config_path)) {
        config = loaded.value().config;
        for (const std::string& warning : loaded.value().warnings) {
            std::cerr << "orrery_app: config: " << warning << '\n';
        }
    } else {
        std::cerr << "orrery_app: config: " << loaded.error() << " (using the defaults)\n";
    }
    const std::vector<std::filesystem::path> dirs = app::experiment_search_dirs(executable);
    app::WindowRequest request;
    request.config = config;
    request.config_path = config_path;
    request.workspace = config.start_workspace;
    std::string space_name = "solar_system";
    if (options.experiment) {
        const auto path = sim::resolve_experiment(*options.experiment, dirs);
        if (!path) {
            std::cerr << "orrery_app: " << path.error() << '\n';
            return app::kExitFailure;
        }
        std::string aero_error;
        request.aero_experiment = aero_experiment_at(path.value(), aero_error);
        if (!aero_error.empty()) {
            std::cerr << "orrery_app: " << aero_error << '\n';
            return app::kExitFailure;
        }
        request.entry_experiment = entry_experiment_at(path.value(), aero_error);
        if (!aero_error.empty()) {
            std::cerr << "orrery_app: " << aero_error << '\n';
            return app::kExitFailure;
        }
        if (request.aero_experiment) {
            request.aero_base_dir = path.value().parent_path();
            request.workspace = app::Workspace::aero;
        } else if (request.entry_experiment) {
            request.workspace = app::Workspace::entry;
        } else {
            space_name = path.value().string();
            request.workspace = app::Workspace::space;
        }
    }
    const auto path = sim::resolve_experiment(space_name, dirs);
    if (!path) {
        std::cerr << "orrery_app: " << path.error() << '\n';
        return app::kExitFailure;
    }
    auto experiment = sim::load_space_experiment(path.value());
    if (!experiment) {
        std::cerr << "orrery_app: " << experiment.error() << '\n';
        return app::kExitFailure;
    }
    request.experiment = std::move(experiment).value();
    // The bundled experiments folder: the one the default experiment came
    // from when it was found there.
    for (const std::filesystem::path& dir : dirs) {
        std::error_code error;
        if (std::filesystem::is_directory(dir, error)) {
            request.bundled_experiment_dir = dir;
            break;
        }
    }
    request.scenarios = sim::list_scenarios(request.experiment.scenario_path.parent_path());
    request.width = options.width.value_or(config.window_width);
    request.height = options.height.value_or(config.window_height);
    request.benchmark_frames = options.benchmark_frames;
    return app::run_window(std::move(request), std::cout, std::cerr);
}
#endif

int dispatch(const orrery::app::CliOptions& options, const std::filesystem::path& executable);

int run(int argc, char** argv) {
    using namespace orrery;
    const std::span<char*> raw(argv, static_cast<std::size_t>(argc));
    const std::filesystem::path executable = app::executable_path(
        raw.empty() ? std::filesystem::path() : std::filesystem::path(raw.front()));
    std::vector<std::string_view> args;
    for (const char* arg : raw.empty() ? raw : raw.subspan(1)) {
        args.emplace_back(arg);
    }

    const auto parsed = app::parse_command_line(args);
    if (!parsed) {
        std::cerr << "orrery_app: " << parsed.error() << "\nTry 'orrery_app --help'.\n";
        return app::kExitUsage;
    }
    const app::CliOptions& options = parsed.value();
    if (!options.trace.empty()) {
        // The trace is written however the run ends (docs/observability.md).
        return app::run_traced(
            options.trace, [&] { return dispatch(options, executable); }, std::cerr);
    }
    return dispatch(options, executable);
}

int dispatch(const orrery::app::CliOptions& options, const std::filesystem::path& executable) {
    using namespace orrery;
    switch (options.mode) {
        case app::RunMode::help:
            std::cout << "Orrery " << core::version_string() << "\n\n" << app::usage_text();
            return app::kExitOk;
        case app::RunMode::version:
            std::cout << "Orrery " << core::version_string() << '\n';
            return app::kExitOk;
        case app::RunMode::modules:
            std::cout << "Orrery " << core::version_string() << '\n';
            if constexpr (app::kHeadless) {
                std::cout << "build: headless\n";
            }
            print_modules(std::cout);
            return app::kExitOk;
        case app::RunMode::headless:
            return app::run_headless_command(options, app::experiment_search_dirs(executable),
                                             std::cout, std::cerr);
        case app::RunMode::window:
#if defined(ORRERY_HEADLESS)
            std::cerr << "orrery_app: this is a headless build without a window; use --headless "
                         "(see --help)\n";
            return app::kExitUsage;
#else
            return run_window_mode(options, executable);
#endif
    }
    return app::kExitFailure;
}

// An exception that reached main is recorded in the crash log as well, like
// a termination (docs/app.md, "Error surface"). Never throws.
void report_to_crash_log(const char* what) noexcept {
    try {
        const std::string reason = std::string("unexpected error: ") + what;
        (void)orrery::app::append_crash_log(
            orrery::app::default_crash_log(),
            orrery::app::crash_log_entry(reason, orrery::core::version_string()));
    } catch (...) {
        std::fputs("orrery_app: the crash report could not be written\n", stderr);
    }
}

} // namespace

int main(int argc, char** argv) {
    // No silent crash: std::terminate reports its reason on stderr and in
    // the crash log (docs/app.md, "Error surface").
    orrery::app::install_terminate_handler(orrery::app::default_crash_log(),
                                           orrery::core::version_string());
    try {
        return run(argc, argv);
    } catch (const std::exception& error) {
        // C stdio: reporting must not throw again.
        std::fputs("orrery_app: unexpected error: ", stderr);
        std::fputs(error.what(), stderr);
        std::fputs("\n", stderr);
        report_to_crash_log(error.what());
    } catch (...) {
        std::fputs("orrery_app: unexpected error\n", stderr);
        report_to_crash_log("non-standard exception");
    }
    return orrery::app::kExitFailure;
}
