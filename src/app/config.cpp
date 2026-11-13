#include "orrery/app/config.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

// Floating-point values through std::to_chars and std::from_chars (a stack
// buffer), as toml++ already does with MSVC. Its fallback for GCC and Clang
// formats through a std::ostringstream of its own, which keeps the digits it
// has when an allocation fails (0.6 saved as 0.5999999999999), so a saved
// configuration could silently change. Both standard libraries the project
// builds with (MSVC STL, libstdc++ 11 or later) implement them.
#if !defined(TOML_FLOAT_CHARCONV)
#define TOML_FLOAT_CHARCONV 1
#endif
#include <toml++/toml.hpp>

#include "orrery/scene/scenario.h"
#include "orrery/sim/aero_import.h"
#include "orrery/sim/warp_governor.h"

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
        return {*home};
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
        return kind == "config" ? home / ".config" / "orrery"
                                : home / ".local" / "share" / "orrery";
    }
#endif
    return fallback_dir() / std::string(kind);
}

std::string where(std::string_view source, const toml::source_region& region) {
    std::ostringstream out;
    out << source << ':' << region.begin.line << ':' << region.begin.column;
    return out.str();
}

void read_space(const toml::table& space, std::string_view source, ConfigLoad& out) {
    const toml::node* node = space.get("physics_budget");
    if (node == nullptr) {
        return;
    }
    // Integers are accepted as numbers too (physics_budget = 1 is out of
    // range either way, but reported as such).
    std::optional<double> value = node->value<double>();
    if (value && std::isfinite(*value) && *value >= sim::kMinLoadLimit &&
        *value <= sim::kMaxLoadLimit) {
        out.config.physics_budget = *value;
        return;
    }
    out.warnings.push_back(where(source, node->source()) +
                           ": space.physics_budget: expected a number in [0.05, 0.95], ignored");
}

// One [shortcuts] entry: a chord string or an array of them (empty:
// unbound). Any invalid chord rejects the whole entry.
std::optional<std::vector<KeyChord>> read_chords(const toml::node& node, std::string& problem) {
    std::vector<KeyChord> chords;
    const auto add = [&](const toml::node& item) {
        const auto text = item.value<std::string>();
        if (!text) {
            problem = "expected a key chord string or an array of them";
            return false;
        }
        auto chord = parse_key_chord(*text);
        if (!chord) {
            problem = chord.error();
            return false;
        }
        if (std::find(chords.begin(), chords.end(), chord.value()) == chords.end()) {
            chords.push_back(chord.value());
        }
        return true;
    };
    if (const toml::array* list = node.as_array()) {
        for (const toml::node& item : *list) {
            if (!add(item)) {
                return std::nullopt;
            }
        }
        return chords;
    }
    if (!add(node)) {
        return std::nullopt;
    }
    return chords;
}

void read_shortcuts(const toml::table& shortcuts, std::string_view source, ConfigLoad& out) {
    std::vector<ShortcutOverride> overrides;
    for (const auto& [key, node] : shortcuts) {
        const std::string name(key.str());
        const auto action = shortcut_action_from_name(name);
        if (!action) {
            std::string warning = where(source, node.source());
            warning += ": shortcuts.";
            warning += name;
            warning += ": unknown action, ignored";
            out.warnings.push_back(std::move(warning));
            continue;
        }
        std::string problem;
        auto chords = read_chords(node, problem);
        if (!chords) {
            std::string warning = where(source, node.source());
            warning += ": shortcuts.";
            warning += name;
            warning += ": ";
            warning += problem;
            warning += "; keeping the default";
            out.warnings.push_back(std::move(warning));
            continue;
        }
        overrides.push_back({*action, std::move(*chords), where(source, node.source())});
    }
    for (std::string& warning : apply_shortcut_overrides(out.config.shortcuts, overrides)) {
        out.warnings.push_back(std::move(warning));
    }
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
    table_keys(root, "", {"paths", "window", "aero", "space", "shortcuts"});
    for (const char* section : {"paths", "window", "aero", "space", "shortcuts"}) {
        if (const toml::node* node = root.get(section); node != nullptr && !node->is_table()) {
            warn(*node, section, "expected a table, ignored");
        }
    }

    if (const toml::table* paths = root["paths"].as_table()) {
        table_keys(*paths, "paths.", {"experiments", "model_cache"});
        for (const auto& [key, target] : {std::pair{"experiments", &out.config.experiment_dir},
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
            } else if (value && *value == "entry") {
                out.config.start_workspace = Workspace::entry;
            } else {
                warn(*node, "window.workspace", R"(expected "space", "aero" or "entry", ignored)");
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
    if (const toml::table* space = root["space"].as_table()) {
        table_keys(*space, "space.", {"physics_budget"});
        read_space(*space, source_name, out);
    }
    if (const toml::table* shortcuts = root["shortcuts"].as_table()) {
        read_shortcuts(*shortcuts, source_name, out);
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
    toml::table space;
    space.insert("physics_budget", config.physics_budget);
    toml::table shortcuts;
    for (const ShortcutInfo& info : shortcut_actions()) {
        toml::array chords;
        for (const KeyChord& chord : config.shortcuts.of(info.action)) {
            chords.push_back(format_key_chord(chord));
        }
        shortcuts.insert(info.name, std::move(chords));
    }
    toml::table root;
    root.insert("paths", std::move(paths));
    root.insert("window", std::move(window));
    root.insert("aero", std::move(aero));
    root.insert("space", std::move(space));
    root.insert("shortcuts", std::move(shortcuts));
    std::ostringstream out;
    // A stream swallows a failure (memory exhausted) and keeps what it has:
    // rethrown instead, so that a truncated text is never saved.
    out.exceptions(std::ios::badbit | std::ios::failbit);
    out << "# Orrery configuration (docs/app.md, \"Configuration\").\n" << root << '\n';
    return out.str();
}

namespace {

// A temporary file removed when it goes out of scope, unless it was renamed
// into place (or never created).
class TemporaryFile {
public:
    explicit TemporaryFile(std::filesystem::path path) : path_(std::move(path)) {}
    TemporaryFile(const TemporaryFile&) = delete;
    TemporaryFile& operator=(const TemporaryFile&) = delete;
    TemporaryFile(TemporaryFile&&) = delete;
    TemporaryFile& operator=(TemporaryFile&&) = delete;
    ~TemporaryFile() {
        if (created_) {
            std::error_code ignored;
            std::filesystem::remove(path_, ignored);
        }
    }

    [[nodiscard]] const std::filesystem::path& path() const noexcept { return path_; }
    void created() noexcept { created_ = true; }
    void renamed() noexcept { created_ = false; }

private:
    std::filesystem::path path_;
    bool created_ = false;
};

// Writes `text` to the temporary file; false when it cannot.
bool write_whole(TemporaryFile& file, const std::string& text) {
    std::ofstream out(file.path(), std::ios::binary | std::ios::trunc);
    if (!out) {
        return false;
    }
    file.created();
    out.write(text.data(), static_cast<std::streamsize>(text.size()));
    out.close();
    return static_cast<bool>(out);
}

} // namespace

core::Result<std::filesystem::path, std::string> save_config(const std::filesystem::path& path,
                                                             const AppConfig& config) {
    // Serialised before anything on disk changes; written to a sibling
    // temporary file that then replaces the configuration in one rename, so
    // a failure at any point leaves the previous file whole.
    const std::string text = config_to_toml(config);
    // A bare file name has no directory to create (the working directory).
    if (const std::filesystem::path dir = path.parent_path(); !dir.empty()) {
        std::error_code error;
        std::filesystem::create_directories(dir, error);
        if (error) {
            return core::failure("cannot create " + dir.string() + ": " + error.message());
        }
    }
    std::filesystem::path temporary_path = path;
    temporary_path += ".tmp";
    TemporaryFile temporary(std::move(temporary_path));
    if (!write_whole(temporary, text)) {
        return core::failure("cannot write " + temporary.path().string());
    }
    std::error_code error;
    std::filesystem::rename(temporary.path(), path, error);
    if (error) {
        return core::failure("cannot replace " + path.string() + ": " + error.message());
    }
    temporary.renamed();
    return path;
}

} // namespace orrery::app
