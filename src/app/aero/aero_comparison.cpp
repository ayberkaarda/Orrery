#include "orrery/sim/aero_comparison.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <cmath>
#include <fstream>
#include <iterator>
#include <numbers>
#include <sstream>
#include <system_error>
#include <utility>

#include "orrery/scene/experiment.h"
#include "orrery/scene/scenario.h"

namespace orrery::sim {

namespace {

constexpr double kDegree = std::numbers::pi / 180.0;

// Shortest round-trip decimal text of a double.
std::string exact(double value) {
    std::array<char, 64> buffer{};
    const auto written = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value);
    return written.ec == std::errc{} ? std::string(buffer.data(), written.ptr) : std::string("0");
}

std::string trimmed_number(double value) {
    std::ostringstream out;
    out.precision(3);
    out << value;
    return out.str();
}

std::string csv_field(const std::string& text) {
    if (text.find_first_of(",\"\r\n") == std::string::npos) {
        return text;
    }
    std::string quoted = "\"";
    for (const char c : text) {
        quoted += c;
        if (c == '"') {
            quoted += '"';
        }
    }
    return quoted + "\"";
}

} // namespace

std::string geometry_label(const scene::GeometryRef& ref) {
    if (ref.source != scene::GeometrySource::primitive) {
        const std::string name = std::filesystem::path(ref.path).filename().string();
        if (!name.empty()) {
            return name;
        }
        return ref.url.empty() ? std::string("User model") : ref.url;
    }
    const scene::ShapeDimensions& d = ref.dimensions;
    const auto m = [](double value) { return trimmed_number(value) + " m"; };
    switch (ref.shape) {
        case scene::AeroShape::sphere:
            return "Sphere, d " + m(d.diameter_m);
        case scene::AeroShape::cube:
            return "Cube, side " + m(d.side_m);
        case scene::AeroShape::cylinder:
            return "Cylinder, d " + m(d.diameter_m) + ", span " + m(d.length_m);
        case scene::AeroShape::cone:
            return "Cone, d " + m(d.diameter_m) + ", length " + m(d.length_m);
        case scene::AeroShape::ellipsoid:
            return "Ellipsoid " + m(d.length_m) + " x " + m(d.width_m) + " x " + m(d.height_m);
        case scene::AeroShape::flat_plate:
            return "Flat plate " + m(d.chord_m) + " x " + m(d.span_m);
        case scene::AeroShape::wedge:
            return "Wedge " + trimmed_number(d.half_angle_deg) + " deg, " + m(d.length_m);
        case scene::AeroShape::capsule:
            return (ref.capsule == "orion" ? "Orion-type capsule, d " : "Apollo-type capsule, d ") +
                   m(d.diameter_m);
        case scene::AeroShape::naca_wing:
            return "NACA " + ref.naca + " wing, chord " + m(d.chord_m) + ", span " + m(d.span_m);
        case scene::AeroShape::naca_section:
            return "NACA " + ref.naca + " section, chord " + m(d.chord_m);
    }
    return "Geometry";
}

std::vector<ComparisonTable> compare_by_geometry(std::span<const scene::AeroExperiment> experiments) {
    std::vector<ComparisonTable> tables;
    for (const scene::AeroExperiment& e : experiments) {
        if (!e.result) {
            continue;
        }
        auto table = std::find_if(tables.begin(), tables.end(), [&](const ComparisonTable& t) {
            return t.geometry == e.geometry;
        });
        if (table == tables.end()) {
            ComparisonTable fresh;
            fresh.geometry = e.geometry;
            fresh.label = geometry_label(e.geometry);
            tables.push_back(std::move(fresh));
            table = std::prev(tables.end());
        }
        const scene::AeroResultRecord& r = *e.result;
        ComparisonRow row;
        row.name = e.name;
        row.angle_of_attack_deg = e.conditions.angle_of_attack_deg;
        row.sideslip_deg = e.conditions.sideslip_deg;
        row.speed_m_s = e.conditions.speed_m_s;
        row.solver = r.solver;
        row.cd = r.cd;
        row.cl = r.cl;
        row.cm = r.cm;
        row.lift_to_drag = r.cd != 0.0 ? r.cl / r.cd : 0.0;
        row.converged = r.converged;
        table->rows.push_back(std::move(row));
    }
    for (ComparisonTable& table : tables) {
        std::stable_sort(table.rows.begin(), table.rows.end(),
                         [](const ComparisonRow& a, const ComparisonRow& b) {
                             if (a.angle_of_attack_deg != b.angle_of_attack_deg) {
                                 return a.angle_of_attack_deg < b.angle_of_attack_deg;
                             }
                             return a.name < b.name;
                         });
        const ComparisonRow base = table.rows.front();
        double mean_a = 0.0;
        double mean_cl = 0.0;
        for (ComparisonRow& row : table.rows) {
            row.delta_cd = row.cd - base.cd;
            row.delta_cl = row.cl - base.cl;
            row.delta_cm = row.cm - base.cm;
            mean_a += row.angle_of_attack_deg;
            mean_cl += row.cl;
        }
        const auto n = static_cast<double>(table.rows.size());
        mean_a /= n;
        mean_cl /= n;
        double sxx = 0.0;
        double sxy = 0.0;
        for (const ComparisonRow& row : table.rows) {
            sxx += (row.angle_of_attack_deg - mean_a) * (row.angle_of_attack_deg - mean_a);
            sxy += (row.angle_of_attack_deg - mean_a) * (row.cl - mean_cl);
        }
        if (table.rows.size() >= 2 && sxx > 0.0) {
            table.lift_slope_per_deg = sxy / sxx;
            table.lift_slope_per_rad = sxy / sxx / kDegree;
        }
    }
    return tables;
}

std::string comparison_csv(const ComparisonTable& table) {
    std::string out = "geometry,name,solver,aoa_deg,sideslip_deg,speed_m_s,cd,cl,cm,"
                      "delta_cd,delta_cl,delta_cm,l_over_d,converged\r\n";
    for (const ComparisonRow& row : table.rows) {
        out += csv_field(table.label) + ',' + csv_field(row.name) + ',' +
               std::string(aero::solver_name(row.solver)) + ',' + exact(row.angle_of_attack_deg) +
               ',' + exact(row.sideslip_deg) + ',' + exact(row.speed_m_s) + ',' + exact(row.cd) +
               ',' + exact(row.cl) + ',' + exact(row.cm) + ',' + exact(row.delta_cd) + ',' +
               exact(row.delta_cl) + ',' + exact(row.delta_cm) + ',' + exact(row.lift_to_drag) +
               ',' + (row.converged ? "1" : "0") + "\r\n";
    }
    return out;
}

core::Result<scene::AeroExperiment, std::string>
load_aero_experiment_file(const std::filesystem::path& path) {
    const auto text = scene::read_text_file(path);
    if (!text) {
        return core::failure(path.filename().string() + ": " + scene::to_string(text.error()));
    }
    auto parsed = scene::parse_aero_experiment(text.value());
    if (!parsed) {
        return core::failure(path.filename().string() + ": " + parsed.error());
    }
    return std::move(parsed).value();
}

core::Result<std::filesystem::path, std::string>
save_aero_experiment_file(const std::filesystem::path& path,
                          const scene::AeroExperiment& experiment) {
    std::error_code error;
    if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path(), error);
        if (error) {
            return core::failure("cannot create " + path.parent_path().string() + ": " +
                                 error.message());
        }
    }
    std::filesystem::path temporary = path;
    temporary += ".tmp";
    {
        std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
        if (!out) {
            return core::failure("cannot write " + temporary.string());
        }
        const std::string text = scene::save_aero_experiment(experiment);
        out.write(text.data(), static_cast<std::streamsize>(text.size()));
        out.close();
        if (!out) {
            std::filesystem::remove(temporary, error);
            return core::failure("cannot write " + temporary.string());
        }
    }
    std::filesystem::rename(temporary, path, error);
    if (error) {
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
        return core::failure("cannot replace " + path.string() + ": " + error.message());
    }
    return path;
}

ExperimentListing list_aero_experiments(const std::filesystem::path& directory) {
    ExperimentListing listing;
    std::error_code error;
    std::filesystem::directory_iterator it(directory, error);
    if (error) {
        return listing;
    }
    std::vector<std::filesystem::path> files;
    for (const std::filesystem::directory_iterator end; it != end; it.increment(error)) {
        if (error) {
            break;
        }
        const std::filesystem::path& path = it->path();
        std::error_code type_error;
        if (path.extension() == ".json" && std::filesystem::is_regular_file(path, type_error)) {
            files.push_back(path);
        }
    }
    std::sort(files.begin(), files.end());
    for (const std::filesystem::path& path : files) {
        const auto text = scene::read_text_file(path);
        if (!text) {
            listing.problems.push_back(path.filename().string() + ": " +
                                       scene::to_string(text.error()));
            continue;
        }
        const auto generic = scene::parse_experiment(text.value());
        if (!generic) {
            listing.problems.push_back(path.filename().string() + ": " +
                                       scene::to_string(generic.error()));
            continue;
        }
        if (generic.value().domain != scene::Domain::aero) {
            continue;
        }
        auto typed = scene::aero_from_experiment(generic.value());
        if (!typed) {
            listing.problems.push_back(path.filename().string() + ": " + typed.error());
            continue;
        }
        listing.experiments.push_back({path, std::move(typed).value()});
    }
    return listing;
}

std::filesystem::path new_experiment_path(const std::filesystem::path& directory,
                                          const std::string& name) {
    std::string stem;
    for (const char c : name) {
        const auto u = static_cast<unsigned char>(c);
        if (std::isalnum(u) != 0) {
            stem += static_cast<char>(std::tolower(u));
        } else if (!stem.empty() && stem.back() != '_') {
            stem += '_';
        }
    }
    while (!stem.empty() && stem.back() == '_') {
        stem.pop_back();
    }
    if (stem.empty()) {
        stem = "experiment";
    }
    constexpr std::size_t kMaxStem = 64;
    if (stem.size() > kMaxStem) {
        stem.resize(kMaxStem);
    }
    std::filesystem::path candidate = directory / (stem + ".json");
    std::error_code error;
    for (int suffix = 2; std::filesystem::exists(candidate, error) && suffix < 10'000; ++suffix) {
        candidate = directory / (stem + "_" + std::to_string(suffix) + ".json");
    }
    return candidate;
}

} // namespace orrery::sim
