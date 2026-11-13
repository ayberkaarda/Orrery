#include "orrery/app/config.h"

#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string_view>
#include <system_error>
#include <utility>

#include <toml++/toml.hpp>

#include "orrery/sim/aero_import.h"
#include "orrery/scene/scenario.h"

namespace orrery::app {

namespace {

constexpr int kMinWindow = 320;
constexpr int kMaxWindow = 8192;
constexpr std::int64_t kMaxStepsPerFrame = 100'000;

std::filesystem::path home_dir() {
#if defined(_WIN32)
    if (auto profile = environment_variable("USERPROFILE")) {
        return std::filesystem::path(*profile);
    }
#else
    if (auto home = environment_variable("HOME")) {
        return std::filesystem::path(*home);
    }
#endif
    return {};
}

std::filesystem::path fallback_dir() {
    std::error_code error;
    const std::filesystem::path temp = std::filesystem::temp_directory_path(error);
    return (error ? std::filesystem::path(".") : temp) / "orrery";
}

// Platform base directory: kind "config" or "data".
std::filesystem::path platform_dir(std::string_view kind) {
#if defined(_WIN32)
    if (auto appdata = environment_variable("APPDATA"); appdata && !appdata->empty()) {
        return std::filesystem::path(*appdata) / "orrery" / std::string(kind);
    }
    if (const std::filesystem::path home = home_dir(); !home.empty()) {
        return home / "AppData" / "Roaming" / "orrery" / std::string(kind);
    }
#elif defined(__APPLE__)
    if (const std::filesystem::path home = home_dir(); !home.empty()) {
        return home / "Library" / "Application Support" / "orrery";
    }
#else
    const char* variable = kind == "config" ? "XDG_CONFIG_HOME" : "XDG_DATA_HOME";
    if (auto xdg = environment_variable(variable); xdg && !xdg->empty()) {
        return std::filesystem::path(*xdg) / "orrery";
    }
    if (const std::filesystem::path home = home_dir(); !home.empty()) {
        return kind == "config" ? home / ".config" / "orrery" : home / ".local" / "share" / "orrery";
    }
#endif
    return fallback_dir() / std::string(kind);
}

std::string where(std::string_view source, const toml::source_region& region) {
    std::ostringstream out;
    out << source << ':' << region.begin.line << ':' << region.begin.column;
    return out.str();
}

std::string_view workspace_name(Workspace workspace) noexcept {
    switch (workspace) {
        case Workspace::space:
            return "space";
        case Workspace::aero:
            return "aero";
        case Workspace::entry:
            return "entry";
    }
    return "space";
}

} // namespace

std::optional<std::string> environment_variable(const char* name) {
#if defined(_MSC_VER)
    char* raw = nullptr;
    std::size_t length = 0;
    if (_dupenv_s(&raw, &length, name) != 0 || raw == nullptr) {
        return std::nullopt;
    }
    const std::unique_ptr<char, decltype(&std::free)> owner(raw, &std::free);
    return std::string(owner.get());
#else
    const char* value = std::getenv(name); // NOLINT(concurrency-mt-unsafe): read at start-up
    if (value == nullptr) {
        return std::nullopt;
    }
    return std::string(value);
#endif
}

std::filesystem::path config_dir() {
    return platform_dir("config");
}

std::filesystem::path data_dir() {
    return platform_dir("data");
}

std::filesystem::path default_config_path() {
    return config_dir() / "config.toml";
}

AppConfig default_config() {
    AppConfig config;
    config.experiment_dir = data_dir() / "experiments";
    config.model_cache_dir = sim::default_model_cache();
    return config;
}

core::Result<ConfigLoad, std::string> parse_config(std::string_view text,
                                                   std::string_view source_name) {
    ConfigLoad out;
    out.config = default_config();
    toml::table root;
    try {
        root = toml::parse(text, source_name);
    } catch (const toml::parse_error& error) {
        return core::failure(where(source_name, error.source()) + ": " +
                             std::string(error.description()));
    }
    const auto warn = [&](const toml::node& node, const std::string& key,
                          const std::string& message) {
        out.warnings.push_back(where(source_name, node.source()) + ": " + key + ": " + message);
    };
    const auto table_keys = [&](const toml::table& table, const std::string& prefix,
                                std::initializer_list<std::string_view> allowed) {
        for (const auto& [key, node] : table) {
            bool known = false;
            for (const std::string_view name : allowed) {
                known = known || key.str() == name;
            }
            if (!known) {
                warn(node, prefix + std::string(key.str()), "unknown key, ignored");
            }
        }
    };
    table_keys(root, "", {"paths", "window", "aero"});

    if (const toml::table* paths = root["paths"].as_table()) {
        table_keys(*paths, "paths.", {"experiments", "model_cache"});
        for (const auto& [key, target] :
             {std::pair{"experiments", &out.config.experiment_dir},
              std::pair{"model_cache", &out.config.model_cache_dir}}) {
            const toml::node* node = paths->get(key);
            if (node == nullptr) {
                continue;
            }
            if (const auto value = node->value<std::string>(); value && !value->empty()) {
                *target = std::filesystem::path(*value);
            } else {
                warn(*node, std::string("paths.") + key, "expected a non-empty string, ignored");
            }
        }
    }
    if (const toml::table* window = root["window"].as_table()) {
        table_keys(*window, "window.", {"width", "height", "workspace"});
        for (const auto& [key, target] : {std::pair{"width", &out.config.window_width},
                                          std::pair{"height", &out.config.window_height}}) {
            const toml::node* node = window->get(key);
            if (node == nullptr) {
                continue;
            }
            const auto value = node->value<std::int64_t>();
            if (value && *value >= kMinWindow && *value <= kMaxWindow) {
                *target = static_cast<int>(*value);
            } else {
                warn(*node, std::string("window.") + key,
                     "expected an integer in [" + std::to_string(kMinWindow) + ", " +
                         std::to_string(kMaxWindow) + "], ignored");
            }
        }
        if (const toml::node* node = window->get("workspace")) {
            const auto value = node->value<std::string>();
            if (value && *value == "space") {
                out.config.start_workspace = Workspace::space;
            } else if (value && *value == "aero") {
                out.config.start_workspace = Workspace::aero;
            } else {
                warn(*node, "window.workspace", "expected \"space\" or \"aero\", ignored");
            }
        }
    }
    if (const toml::table* aero = root["aero"].as_table()) {
        table_keys(*aero, "aero.", {"steps_per_frame"});
        if (const toml::node* node = aero->get("steps_per_frame")) {
            const auto value = node->value<std::int64_t>();
            if (value && *value >= 1 && *value <= kMaxStepsPerFrame) {
                out.config.steps_per_frame = *value;
            } else {
                warn(*node, "aero.steps_per_frame",
                     "expected an integer in [1, " + std::to_string(kMaxStepsPerFrame) +
                         "], ignored");
            }
        }
    }
    return out;
}

core::Result<ConfigLoad, std::string> load_config(const std::filesystem::path& path) {
    std::error_code error;
    if (!std::filesystem::exists(path, error)) {
        return ConfigLoad{default_config(), {}};
    }
    const auto text = scene::read_text_file(path);
    if (!text) {
        return core::failure(path.string() + ": " + scene::to_string(text.error()));
    }
    return parse_config(text.value(), path.filename().string());
}

std::string config_to_toml(const AppConfig& config) {
    toml::table paths;
    paths.insert("experiments", config.experiment_dir.generic_string());
    paths.insert("model_cache", config.model_cache_dir.generic_string());
    toml::table window;
    window.insert("width", static_cast<std::int64_t>(config.window_width));
    window.insert("height", static_cast<std::int64_t>(config.window_height));
    window.insert("workspace", std::string(workspace_name(config.start_workspace)));
    toml::table aero;
    aero.insert("steps_per_frame", config.steps_per_frame);
    toml::table root;
    root.insert("paths", std::move(paths));
    root.insert("window", std::move(window));
    root.insert("aero", std::move(aero));
    std::ostringstream out;
    out << "# Orrery configuration (docs/app.md, \"Configuration\").\n" << root << '\n';
    return out.str();
}

core::Result<std::filesystem::path, std::string> save_config(const std::filesystem::path& path,
                                                             const AppConfig& config) {
    std::error_code error;
    std::filesystem::create_directories(path.parent_path(), error);
    if (error) {
        return core::failure("cannot create " + path.parent_path().string() + ": " +
                             error.message());
    }
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    const std::string text = config_to_toml(config);
    out.write(text.data(), static_cast<std::streamsize>(text.size()));
    out.close();
    if (!out) {
        return core::failure("cannot write " + path.string());
    }
    return path;
}

} // namespace orrery::app
