#pragma once

#include <filesystem>
#include <string>
#include <string_view>

#include "orrery/core/result.h"
#include "orrery/sim/aero_geometry.h"
#include "orrery/sim/background_task.h"

// Loading user models for the Aero workspace (docs/aero-workspace.md,
// "Geometry"): from disk through the importer of orrery_geom, or from an
// https URL through the secure loader of orrery_geom_net (ADR-010) and then
// the same importer. Both are blocking and meant for a BackgroundTask; every
// failure carries a title and the full detail for the error modal. A task
// whose progress reports stop_requested() ends at the next stage boundary
// with the title "Cancelled"; a running download, parse or repair call is
// not interrupted (orrery_geom has no cancellation hook): a stalled download
// ends at the fetch limits (geom::net::FetchLimits, 30 s below 1 KiB/s,
// 300 s in all).
namespace orrery::sim {

using ImportResult = core::Result<ImportedModel, GeometryError>;
using PrepareResult = core::Result<PreparedGeometry, GeometryError>;

// True when `progress` asks the work to stop.
[[nodiscard]] bool cancelled(const TaskProgress* progress) noexcept;
// The failure of cancelled work.
[[nodiscard]] GeometryError cancelled_error();

// Imports a model file (raw units and axes) with its unit and up-axis
// estimates. `license_note` is carried into the geometry reference.
[[nodiscard]] ImportResult load_model_file(const std::filesystem::path& path,
                                           std::string license_note,
                                           TaskProgress* progress = nullptr);

// The platform cache of downloaded models (geom::net::default_model_cache_dir).
[[nodiscard]] std::filesystem::path default_model_cache();

struct UrlImportOptions {
    // Cache root; empty uses the platform cache (geom::net::default_model_cache_dir).
    std::filesystem::path cache_dir;
    std::string license_note;
};

// Downloads (or takes from the cache) and imports a model from an https URL.
[[nodiscard]] ImportResult fetch_model_url(std::string_view url, const UrlImportOptions& options,
                                           TaskProgress* progress = nullptr);

// The geometry of an experiment: built for a primitive, imported and
// prepared with the recorded unit scale and up axis for a user model (a URL
// model is read from its cached file `path`; it is downloaded again when the
// file is gone). Relative paths are resolved against `base_dir`.
[[nodiscard]] PrepareResult load_experiment_geometry(const scene::GeometryRef& ref,
                                                     const std::filesystem::path& base_dir,
                                                     TaskProgress* progress = nullptr);

} // namespace orrery::sim
