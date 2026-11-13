#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "orrery/core/result.h"

// User configuration of orrery_app (docs/app.md, "Configuration"): a TOML
// file in the platform's configuration directory, read with toml++.
//
//   [paths]
//   experiments = "..."        # saved aero experiments (default: data dir)
//   model_cache = "..."        # downloaded models (default: platform cache)
//   [window]
//   width = 1600               # [px], 320 .. 8192
//   height = 900
//   workspace = "space"        # first workspace: "space" or "aero"
//   [aero]
//   steps_per_frame = 100      # lattice steps per frame, 1 .. 100000
//
// A missing file means the defaults. Unknown keys and invalid values are
// reported as warnings and ignored (the rest of the file still applies); a
// file that is not valid TOML is an error, and the caller falls back to the
// defaults with that error shown.
namespace orrery::app {

enum class Workspace : std::uint8_t { space, aero, entry };

struct AppConfig {
    std::filesystem::path experiment_dir;
    std::filesystem::path model_cache_dir;
    int window_width = 1600;
    int window_height = 900;
    Workspace start_workspace = Workspace::space;
    std::int64_t steps_per_frame = 100;

    friend bool operator==(const AppConfig&, const AppConfig&) = default;
};

struct ConfigLoad {
    AppConfig config;
    std::vector<std::string> warnings; // "config.toml: window.width: ..." lines
};

// Platform directories, in the manner of the `directories` convention:
//   config  Linux $XDG_CONFIG_HOME/orrery (else ~/.config/orrery),
//           macOS ~/Library/Application Support/orrery,
//           Windows %APPDATA%\orrery\config
//   data    Linux $XDG_DATA_HOME/orrery (else ~/.local/share/orrery),
//           macOS ~/Library/Application Support/orrery,
//           Windows %APPDATA%\orrery\data
// Both fall back to <temp>/orrery when no home directory is known.
[[nodiscard]] std::filesystem::path config_dir();
[[nodiscard]] std::filesystem::path data_dir();
[[nodiscard]] std::filesystem::path default_config_path(); // config_dir()/config.toml

// Defaults: experiments in data_dir()/experiments, the platform model cache.
[[nodiscard]] AppConfig default_config();

[[nodiscard]] core::Result<ConfigLoad, std::string>
parse_config(std::string_view text, std::string_view source_name);
[[nodiscard]] core::Result<ConfigLoad, std::string> load_config(const std::filesystem::path& path);
// The configuration as TOML (every key, so the file documents itself).
[[nodiscard]] std::string config_to_toml(const AppConfig& config);
[[nodiscard]] core::Result<std::filesystem::path, std::string>
save_config(const std::filesystem::path& path, const AppConfig& config);

// An environment variable, nullopt when unset (portable getenv).
[[nodiscard]] std::optional<std::string> environment_variable(const char* name);

} // namespace orrery::app
