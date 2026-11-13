#include "orrery/aero/lbm3d/lbm_aero_solver3d.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <numbers>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>

#include "orrery/geom/mesh_validation.h"
#include "orrery/geom/voxel_grid.h"

namespace orrery::aero::lbm {

namespace {

constexpr double kLesReynolds = 2000.0;     // automatic Smagorinsky above this Re
constexpr double kMinCellsPerLength = 16.0; // below: resolution warning
constexpr double kMinBodyCells = 4.0;       // thinnest body extent below: resolution warning
constexpr double kMarginalTau = 0.51;       // below: BGK stability warning
constexpr double kLocalMachWarning = 0.2;   // largest lattice Mach in the flow
constexpr double kBlockageWarning = 0.05;   // frontal area / channel cross-section
constexpr double kSteadyResidual = 1e-3;    // steady: force ranges / mean drag
constexpr double kMaxPeriodSpread = 0.05;   // periodic: period spread
constexpr double kStationarity = 0.02;      // periodic: amplitude and mean drag drift
// Largest step count (and averaging window in samples): 2^53, below which
// every integer is a double, far above any run that fits in memory and time.
constexpr double kMaxSteps = 9007199254740992.0;
// Largest lattice extent per axis checked before conversion to an integer.
constexpr double kMaxExtent = 1e7;
// Offset of the body from the lattice planes, in nodes.
constexpr double kPlacementShift = 0.25;

bool positive_finite(double v) noexcept {
    return std::isfinite(v) && v > 0.0;
}

std::string format(double value) {
    std::ostringstream out;
    out << value;
    return out.str();
}

// Body-axes vector of wind-frame components (v[0] along axes[0], ...).
geom::Vector3 to_body(const std::array<geom::Vector3, 3>& axes,
                      const std::array<double, 3>& v) noexcept {
    return axes[0] * v[0] + axes[1] * v[1] + axes[2] * v[2];
}

// The largest cells per reference length whose lattice stays within
// `budget` nodes, assuming the node count scales with its cube.
double fitting_resolution(double cells_per_length, double needed, double budget) {
    return std::floor(10.0 * cells_per_length * std::cbrt(budget / needed)) / 10.0;
}

} // namespace

bool forces_settled_3d(std::span<const double> time, std::span<const double> drag,
                       std::span<const double> lift, std::span<const double> side, double from,
                       const SettledCriteria& criteria) {
    const double bound = criteria.steady_tolerance * std::abs(mean_since(time, drag, from));
    const auto stationary = [&](std::span<const double> values) {
        // Steady: at least two samples, range within the bound.
        const std::size_t n = std::min(time.size(), values.size());
        std::size_t count = 0;
        double lo = 0.0;
        double hi = 0.0;
        for (std::size_t k = 0; k < n; ++k) {
            if (time[k] >= from) {
                lo = count == 0 ? values[k] : std::min(lo, values[k]);
                hi = count == 0 ? values[k] : std::max(hi, values[k]);
                ++count;
            }
        }
        if (count >= 2 && hi - lo <= bound) {
            return true;
        }
        return settled_oscillation(time, drag, values, from, criteria).has_value();
    };
    return stationary(drag) && stationary(lift) && stationary(side);
}

void LbmAeroSolver3D::setup(const AeroGeometry& geometry, const FlowConditions& conditions) {
    solver_.reset();
    units_.reset();
    result_ = AeroResult{};
    setup_warnings_.clear();
    const geom::TriangleMesh& mesh = geometry.mesh;
    if (auto invalid = geom::validate_mesh(mesh, geom::MeshLimits{})) {
        throw std::invalid_argument("lbm3d: invalid mesh: " + invalid->message);
    }
    const ReferenceValues& reference = geometry.reference;
    if (!positive_finite(conditions.speed) || !positive_finite(conditions.density) ||
        !positive_finite(conditions.viscosity) || !std::isfinite(conditions.angle_of_attack) ||
        !std::isfinite(conditions.sideslip)) {
        throw std::invalid_argument(
            "lbm3d: speed, density and viscosity must be positive and finite, angles finite");
    }
    if (!positive_finite(reference.length) || !positive_finite(reference.area) ||
        !std::isfinite(reference.moment_centre.x) || !std::isfinite(reference.moment_centre.y) ||
        !std::isfinite(reference.moment_centre.z)) {
        throw std::invalid_argument(
            "lbm3d: reference length and area must be positive and finite, the moment centre "
            "finite");
    }
    if (!positive_finite(options_.upstream) || !positive_finite(options_.downstream) ||
        !positive_finite(options_.width) || !positive_finite(options_.duration) ||
        !positive_finite(options_.average_window) || options_.steps_per_call == 0 ||
        options_.sample_every == 0) {
        throw std::invalid_argument("lbm3d: domain, duration and step options must be positive");
    }
    if (!std::isfinite(options_.sponge_strength) || options_.sponge_strength < 0.0 ||
        options_.sponge_strength > 1.0 || !(options_.sponge_inlet >= 0.0) ||
        !(options_.sponge_outlet >= 0.0) || !(options_.sponge_sides >= 0.0) ||
        !(options_.start_ramp >= 0.0) || !std::isfinite(options_.start_ramp) ||
        !std::isfinite(options_.sponge_inlet + options_.sponge_outlet + options_.sponge_sides)) {
        throw std::invalid_argument(
            "lbm3d: the absorbing-layer strength must be in [0, 1], the thicknesses finite and "
            "non-negative");
    }
    if (options_.sides == WallKind::no_slip) {
        throw std::invalid_argument("lbm3d: the channel sides are free slip or periodic");
    }
    if (options_.max_cells > geom::max_grid_cells) {
        throw std::invalid_argument("lbm3d: max_cells above geom::max_grid_cells");
    }
    const double length = reference.length;
    auto units =
        LatticeUnits::create(SiFlow{length, conditions.speed,
                                    conditions.viscosity / conditions.density, conditions.density},
                             options_.resolution);
    if (!units) {
        throw std::invalid_argument(units.error());
    }
    const LatticeUnits lattice_units = units.value();
    const double dx = lattice_units.dx();

    // Wind frame: lattice x along the free stream, y along the side force,
    // z along the lift (a right-handed set: drag x side = lift).
    axes_ = {conditions.drag_direction(), conditions.side_direction(), conditions.lift_direction()};
    geom::TriangleMesh wind = mesh;
    std::array<double, 3> lo{};
    std::array<double, 3> hi{};
    for (std::size_t k = 0; k < wind.vertices.size(); ++k) {
        const geom::Vector3 p = mesh.vertices[k];
        const std::array<double, 3> w{geom::dot(p, axes_[0]), geom::dot(p, axes_[1]),
                                      geom::dot(p, axes_[2])};
        for (std::size_t a = 0; a < 3; ++a) {
            lo[a] = k == 0 ? w[a] : std::min(lo[a], w[a]);
            hi[a] = k == 0 ? w[a] : std::max(hi[a], w[a]);
        }
        wind.vertices[k] = geom::Vector3{w[0], w[1], w[2]};
    }
    const std::array<double, 3> extent{hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]};
    const std::array<double, 3> span_real{
        (options_.upstream * length + extent[0] + options_.downstream * length) / dx,
        std::max(options_.width * length, extent[1] + 4.0 * dx) / dx,
        std::max(options_.width * length, extent[2] + 4.0 * dx) / dx};
    for (const double s : span_real) {
        if (!(s < kMaxExtent)) {
            throw std::invalid_argument("lbm3d: the domain is too large for the resolution");
        }
    }
    const auto nx = static_cast<std::size_t>(std::ceil(span_real[0])) + 1;
    const auto ny = static_cast<std::size_t>(std::ceil(span_real[1])) + 1;
    const auto nz = static_cast<std::size_t>(std::ceil(span_real[2])) + 1;
    // The solver needs three nodes per axis, and the body placement below
    // clamps to the interior nodes [1, n - 2], an empty range below three.
    if (nx < 3 || ny < 3 || nz < 3) {
        throw std::invalid_argument("lbm3d: the lattice " + std::to_string(nx) + " x " +
                                    std::to_string(ny) + " x " + std::to_string(nz) +
                                    " needs at least 3 nodes along each axis; raise the "
                                    "resolution or the domain");
    }

    // Cell budget: the caller's, else the device memory the backend reports,
    // else the conservative cap; checked before anything is allocated.
    Lbm3DConfig config;
    config.nx = nx;
    config.ny = ny;
    config.nz = nz;
    std::size_t budget = options_.max_cells != 0 ? options_.max_cells : default_max_cells_3d;
    std::string budget_reason = options_.max_cells != 0
                                    ? "LbmSolver3DOptions::max_cells"
                                    : "the default cap of 256^3 nodes (device memory unknown)";
    if (options_.backend != nullptr && options_.max_cells == 0) {
        const compute::DeviceMemory memory = options_.backend->memory();
        // Bytes per node on the device: two population buffers, the node
        // class and four moment planes.
        constexpr std::size_t kDeviceBytesPerCell = (2 * d3q19::q + 1 + 4) * 4;
        if (memory.available_bytes) {
            budget = std::min(geom::max_grid_cells, *memory.available_bytes / kDeviceBytesPerCell);
            budget_reason = "the free device memory (" +
                            format(static_cast<double>(*memory.available_bytes) / 1048576.0) +
                            " MiB)";
        }
        if (memory.max_binding_bytes && *memory.max_binding_bytes / (d3q19::q * 4) < budget) {
            budget = *memory.max_binding_bytes / (d3q19::q * 4);
            budget_reason = "the largest buffer the device can bind (" +
                            format(static_cast<double>(*memory.max_binding_bytes) / 1048576.0) +
                            " MiB)";
        }
    }
    const std::optional<std::size_t> cells = lattice_cells_3d(nx, ny, nz);
    const double needed =
        static_cast<double>(nx) * static_cast<double>(ny) * static_cast<double>(nz);
    if (!cells || *cells > budget) {
        throw std::invalid_argument(
            "lbm3d: the lattice " + std::to_string(nx) + " x " + std::to_string(ny) + " x " +
            std::to_string(nz) + " (" + format(needed) + " nodes) exceeds " + budget_reason +
            ", a budget of " + std::to_string(budget) + " nodes; use at most " +
            format(fitting_resolution(lattice_units.cells_per_length(), needed,
                                      static_cast<double>(budget))) +
            " cells per reference length or a smaller domain");
    }

    // Lattice node (i, j, k) sits at origin + dx (i, j, k) in the wind frame.
    // The body is shifted a quarter node off the lattice planes, so a face
    // aligned with the axes never passes through nodes (whose winding number
    // would sit at the threshold 1/2) and the discrete body is not symmetric
    // about the channel axis.
    origin_ = {lo[0] - options_.upstream * length - kPlacementShift * dx,
               0.5 * (lo[1] + hi[1]) - (0.5 * static_cast<double>(ny - 1) + kPlacementShift) * dx,
               0.5 * (lo[2] + hi[2]) - (0.5 * static_cast<double>(nz - 1) + kPlacementShift) * dx};
    // Voxelize the bounding box of the body only (plus a margin of one
    // node): the winding number outside it is zero.
    std::array<std::size_t, 3> first{};
    std::array<std::size_t, 3> count{};
    const std::array<std::size_t, 3> dims{nx, ny, nz};
    for (std::size_t a = 0; a < 3; ++a) {
        const double from = std::floor((lo[a] - origin_[a]) / dx) - 1.0;
        const double to = std::ceil((hi[a] - origin_[a]) / dx) + 1.0;
        const auto last = static_cast<double>(dims[a] - 2);
        const double begin = std::clamp(from, 1.0, last);
        const double end = std::clamp(to, begin, last);
        first[a] = static_cast<std::size_t>(begin);
        count[a] = static_cast<std::size_t>(end) - first[a] + 1;
    }
    geom::GridSpec spec;
    spec.origin = geom::Vector3{origin_[0] + (static_cast<double>(first[0]) - 0.5) * dx,
                                origin_[1] + (static_cast<double>(first[1]) - 0.5) * dx,
                                origin_[2] + (static_cast<double>(first[2]) - 0.5) * dx};
    spec.spacing = dx;
    spec.nx = count[0];
    spec.ny = count[1];
    spec.nz = count[2];
    const geom::VoxelGrid voxels = geom::voxelize(wind, spec);
    SolidMask3D mask = fluid_mask_3d(nx, ny, nz);
    place_voxels(mask, voxels, first);
    if (mask.count() == 0) {
        throw std::invalid_argument("lbm3d: the mesh covers no lattice node (resolution too low)");
    }
    if (options_.wall == SolidWall::interpolated) {
        // Exact distances within three cells of the surface, signed by the
        // voxel classification: the link fractions of the interpolated wall.
        place_distance(mask, geom::signed_distance_band(wind, voxels, 3.0), first);
    }

    // Step counts, checked before they are converted to integers (a double
    // outside the integer range does not convert) and before allocating.
    const double cells_per_length = lattice_units.cells_per_length();
    const double steps_per_time = cells_per_length / lattice_units.lattice_velocity();
    const double total_real = std::ceil(options_.duration * steps_per_time);
    const double window_real =
        options_.average_window * steps_per_time / static_cast<double>(options_.sample_every);
    const double ramp_real = options_.start_ramp * steps_per_time;
    if (!(total_real < kMaxSteps) || !(window_real < kMaxSteps) || !(ramp_real < kMaxSteps)) {
        throw std::invalid_argument("lbm3d: duration and averaging window must stay below " +
                                    format(kMaxSteps) + " lattice steps");
    }

    config.tau = lattice_units.tau();
    config.x_boundary = XBoundary::inflow_outflow;
    config.outlet = Outlet::convective;
    config.y_low = Wall{options_.sides, 0.0};
    config.y_high = Wall{options_.sides, 0.0};
    config.z_low = Wall{options_.sides, 0.0};
    config.z_high = Wall{options_.sides, 0.0};
    config.inlet_ux = lattice_units.lattice_velocity();
    const double reynolds = lattice_units.reynolds();
    const bool les = options_.les == LesMode::on ||
                     (options_.les == LesMode::automatic && reynolds > kLesReynolds);
    config.smagorinsky = Smagorinsky{les, options_.smagorinsky_constant};
    config.solid_wall = options_.wall;
    // Layer thicknesses in nodes, at most a quarter of the axis (the body
    // stays outside them for the default domain).
    const auto layer = [&](double lengths, std::size_t axis) {
        const double nodes = std::round(lengths * cells_per_length);
        return static_cast<std::size_t>(
            std::min(nodes, std::floor(0.25 * static_cast<double>(axis))));
    };
    config.sponge =
        Sponge{options_.sponge_strength, layer(options_.sponge_inlet, nx),
               layer(options_.sponge_outlet, nx), layer(options_.sponge_sides, std::min(ny, nz))};

    // Frontal area of the voxelized body: (y, z) columns with a solid node.
    std::size_t frontal = 0;
    for (std::size_t z = 0; z < nz; ++z) {
        for (std::size_t y = 0; y < ny; ++y) {
            for (std::size_t x = first[0]; x < first[0] + count[0]; ++x) {
                if (mask.at(x, y, z)) {
                    ++frontal;
                    break;
                }
            }
        }
    }
    blockage_ = static_cast<double>(frontal) / (static_cast<double>(ny) * static_cast<double>(nz));

    auto made = options_.backend != nullptr ? make_gpu_solver(*options_.backend, config, mask)
                                            : make_cpu_solver(config, std::move(mask));
    if (!made) {
        throw std::invalid_argument(made.error());
    }
    solver_ = std::move(made).value();
    units_ = lattice_units;
    ramp_steps_ = ramp_real;
    if (ramp_steps_ > 0.0) {
        const auto rest = [](std::size_t, std::size_t, std::size_t) {
            return d3q19::Moments{1.0, 0.0, 0.0, 0.0};
        };
        if (auto started = solver_->initialize(rest); !started) {
            throw std::invalid_argument(started.error());
        }
        if (auto still = solver_->set_inlet_velocity({0.0, 0.0, 0.0}); !still) {
            throw std::invalid_argument(still.error());
        }
    }

    const geom::Vector3& c = reference.moment_centre;
    centre_ = {(geom::dot(c, axes_[0]) - origin_[0]) / dx,
               (geom::dot(c, axes_[1]) - origin_[1]) / dx,
               (geom::dot(c, axes_[2]) - origin_[2]) / dx};
    surface_area_ = geom::surface_area(mesh);

    conditions_ = conditions;
    reference_ = reference;
    total_steps_ = static_cast<std::uint64_t>(total_real);
    window_samples_ = std::max<std::size_t>(1, static_cast<std::size_t>(window_real));
    history_ = CoefficientHistory(window_samples_);
    side_.clear();
    samples_.clear();
    instantaneous_ = Coefficients3D{};
    previous_cd_ = 0.0;

    // Envelope of the method plus the lattice-specific conditions.
    setup_warnings_ = check_validity(SolverKind::lattice_boltzmann, conditions, length);
    const double thinnest = std::min({extent[0], extent[1], extent[2]}) / dx;
    if (cells_per_length < kMinCellsPerLength || thinnest < kMinBodyCells) {
        setup_warnings_.push_back(
            {"lbm.resolution", format(cells_per_length) + " cells per reference length and " +
                                   format(thinnest) +
                                   " cells across the thinnest body extent; below " +
                                   format(kMinCellsPerLength) + " and " + format(kMinBodyCells) +
                                   " the staircase wall dominates the force error (A4)"});
    }
    if (config.tau < kMarginalTau) {
        setup_warnings_.push_back(
            {"lbm.tau", "relaxation time " + format(config.tau) + " is within " +
                            format(kMarginalTau - 0.5) +
                            " of 1/2: BGK is close to instability; raise the resolution or "
                            "lower the Reynolds number"});
    }
    if (les) {
        setup_warnings_.push_back(
            {"lbm.les", "Smagorinsky subgrid closure active at Re " + format(reynolds) +
                            " (APPROXIMATION A2: eddy viscosity, not resolved turbulence)"});
    } else if (reynolds > kLesReynolds) {
        setup_warnings_.push_back(
            {"lbm.les_off", "Re " + format(reynolds) + " above " + format(kLesReynolds) +
                                " without the Smagorinsky closure: the flow is under-resolved"});
    }
    if (blockage_ > kBlockageWarning) {
        setup_warnings_.push_back(
            {"lbm.blockage", "the body blocks " + format(100.0 * blockage_) +
                                 " percent of the channel cross-section (above " +
                                 format(100.0 * kBlockageWarning) +
                                 "); confinement raises the drag"});
    }
    result_.warnings = setup_warnings_;
}

bool LbmAeroSolver3D::step() {
    if (!solver_ || !units_) {
        throw std::logic_error("lbm3d: step() called before setup()");
    }
    const std::uint64_t target =
        solver_->steps() +
        std::min<std::uint64_t>(total_steps_ - solver_->steps(), options_.steps_per_call);
    const std::uint64_t every = options_.sample_every;
    while (solver_->steps() < target) {
        // Forces are sampled on the absolute schedule (lattice steps that are
        // multiples of sample_every), whatever the call size, so the samples
        // and their averages do not depend on how the run is split into calls.
        const std::uint64_t now = solver_->steps();
        const std::uint64_t next_sample = now - now % every + every;
        const std::uint64_t until = std::min(target, next_sample);
        const std::size_t chunk = until - now; // at most sample_every
        if (ramp_steps_ > 0.0) {
            // The inlet speed of the gradual start, constant over each
            // sampling interval (the absolute schedule again).
            const auto start = static_cast<double>(now - now % every);
            const double fraction =
                start >= ramp_steps_
                    ? 1.0
                    : 0.5 * (1.0 - std::cos(std::numbers::pi * start / ramp_steps_));
            if (auto set =
                    solver_->set_inlet_velocity({fraction * solver_->config().inlet_ux, 0.0, 0.0});
                !set) {
                throw std::runtime_error(set.error());
            }
        }
        if (auto stepped = solver_->step(chunk); !stepped) {
            throw std::runtime_error(stepped.error());
        }
        if (solver_->steps() % every != 0) {
            continue; // the call ends between two samples
        }
        auto force = solver_->force();
        if (!force) {
            throw std::runtime_error(force.error());
        }
        Force3D f = force.value();
        // Moment about the moment centre instead of the lattice origin:
        // M_c = M_o - r_c x F.
        const std::array<double, 3>& r = centre_;
        f.moment[0] -= r[1] * f.force[2] - r[2] * f.force[1];
        f.moment[1] -= r[2] * f.force[0] - r[0] * f.force[2];
        f.moment[2] -= r[0] * f.force[1] - r[1] * f.force[0];
        samples_.push_back(f);
        AeroResult sample;
        apply_force(sample, f, *units_);
        instantaneous_ = Coefficients3D{sample.cd, sample.cl, sample.cy};
        history_.push(static_cast<double>(solver_->steps()), Coefficients{sample.cd, sample.cl});
        side_.push_back(sample.cy);
    }
    update_result();
    return result_.iterations >= total_steps_;
}

void LbmAeroSolver3D::apply_force(AeroResult& result, const Force3D& lattice,
                                  const LatticeUnits& units) const {
    // One lattice force unit is rho_ref dx^4 / dt^2 newtons (the per-span
    // unit of the 2D conversion times dx); moments carry one more dx.
    const double dx = units.dx();
    const double force_unit = units.force_per_span_to_si(1.0) * dx;
    result.force = to_body(axes_, {lattice.force[0] * force_unit, lattice.force[1] * force_unit,
                                   lattice.force[2] * force_unit});
    result.moment =
        to_body(axes_, {lattice.moment[0] * force_unit * dx, lattice.moment[1] * force_unit * dx,
                        lattice.moment[2] * force_unit * dx});
    set_coefficients(result, conditions_, reference_);
}

void LbmAeroSolver3D::update_result() {
    if (!solver_ || !units_) {
        return;
    }
    const LatticeUnits units = *units_;
    const double dx = units.dx();
    AeroResult next;
    next.iterations = solver_->steps();
    next.residuals = result_.residuals;

    // Moving average of force and moment over the window.
    const std::size_t n = std::min(window_samples_, samples_.size());
    Force3D mean;
    for (std::size_t k = samples_.size() - n; k < samples_.size(); ++k) {
        for (std::size_t a = 0; a < 3; ++a) {
            mean.force[a] += samples_[k].force[a];
            mean.moment[a] += samples_[k].moment[a];
        }
    }
    if (n > 0) {
        for (std::size_t a = 0; a < 3; ++a) {
            mean.force[a] /= static_cast<double>(n);
            mean.moment[a] /= static_cast<double>(n);
        }
    }
    apply_force(next, mean, units);

    const double residual =
        next.cd != 0.0 ? std::abs(next.cd - previous_cd_) / std::abs(next.cd) : 0.0;
    next.residuals.push_back(residual);
    previous_cd_ = next.cd;

    // Wall samples: one per fluid node next to the body.
    auto fields = solver_->macroscopic();
    if (!fields) {
        throw std::runtime_error(fields.error());
    }
    const MacroFields3D& macro = fields.value();
    const Lbm3DConfig& config = solver_->config();
    const SolidMask3D& mask = solver_->mask();
    const double q_lattice = 0.5 * units.lattice_velocity() * units.lattice_velocity();
    // Free-stream pressure: the mean density of the plane next to the inlet
    // (the pressure level of the channel floats with the drag it carries).
    double rho_inf = 0.0;
    for (std::size_t z = 0; z < config.nz; ++z) {
        for (std::size_t y = 0; y < config.ny; ++y) {
            rho_inf += macro.density[mask.index(1, y, z)];
        }
    }
    rho_inf /= static_cast<double>(config.ny * config.nz);
    std::map<std::uint32_t, std::array<double, 3>> wall;
    for (const BoundaryLink3D& link : solver_->links()) {
        std::array<double, 3>& normal = wall[link.cell];
        normal[0] += d3q19::cx[link.direction];
        normal[1] += d3q19::cy[link.direction];
        normal[2] += d3q19::cz[link.direction];
    }
    // APPROXIMATION D7: every wall sample carries an equal share of the mesh
    // area (the staircase wall has no better-defined element area).
    const double share = wall.empty() ? 0.0 : surface_area_ / static_cast<double>(wall.size());
    next.surface.points.reserve(wall.size());
    next.surface.normals.reserve(wall.size());
    next.surface.areas.reserve(wall.size());
    next.surface.cp.reserve(wall.size());
    for (const auto& [cell, sum] : wall) {
        // Node position of the cell: the wall point of a link with q = 0.
        const std::array<double, 3> node = link_wall_point(config, BoundaryLink3D{cell, 0, 0.0});
        const double norm = std::sqrt(sum[0] * sum[0] + sum[1] * sum[1] + sum[2] * sum[2]);
        const std::array<double, 3> normal =
            norm > 0.0 ? std::array<double, 3>{sum[0] / norm, sum[1] / norm, sum[2] / norm}
                       : std::array<double, 3>{0.0, 0.0, 0.0};
        next.surface.points.push_back(
            to_body(axes_, {origin_[0] + node[0] * dx, origin_[1] + node[1] * dx,
                            origin_[2] + node[2] * dx}));
        next.surface.normals.push_back(to_body(axes_, normal));
        next.surface.areas.push_back(share);
        next.surface.cp.push_back(d3q19::cs2 * (macro.density[cell] - rho_inf) / q_lattice);
    }

    // Velocity field on the lattice: body axes coincide with the lattice
    // axes only at zero angle of attack and sideslip (see docs/lbm.md).
    if (conditions_.angle_of_attack == 0.0 && conditions_.sideslip == 0.0) {
        VelocityField field;
        field.dims = {config.nx, config.ny, config.nz};
        field.origin = geom::Vector3{origin_[0], origin_[1], origin_[2]};
        field.spacing = dx;
        field.values.reserve(macro.velocity_x.size());
        for (std::size_t k = 0; k < macro.velocity_x.size(); ++k) {
            field.values.push_back({units.velocity_to_si(macro.velocity_x[k]),
                                    units.velocity_to_si(macro.velocity_y[k]),
                                    units.velocity_to_si(macro.velocity_z[k])});
        }
        next.velocity = std::move(field);
    }

    next.warnings = setup_warnings_;
    const double local_mach = lattice_mach(max_speed(macro));
    if (local_mach > kLocalMachWarning) {
        next.warnings.push_back(
            {"lbm.local_mach", "largest lattice Mach number in the flow " + format(local_mach) +
                                   " (compressibility error of order Ma^2, A3)"});
    }
    // Converged: over the last half of the run (or the averaging window, if
    // longer) drag, lift and side force are each steady or settled periodic
    // (consistent periods, stationary amplitude and mean drag;
    // forces_settled_3d). A 3D wake may shed in any plane, and a sphere's
    // drag oscillates with it.
    const bool finished = next.iterations >= total_steps_;
    if (finished) {
        const std::span<const double> time = history_.time();
        double from = 0.5 * static_cast<double>(next.iterations);
        if (history_.size() > window_samples_) {
            from = std::min(from, time[history_.size() - window_samples_]);
        }
        const SettledCriteria criteria{kSteadyResidual, kMaxPeriodSpread, kStationarity};
        next.converged =
            forces_settled_3d(time, history_.drag(), history_.lift(), side_, from, criteria);
        if (!next.converged) {
            next.warnings.push_back(
                {"lbm.unconverged",
                 "the force history of the last half of the run is neither steady (drag, lift "
                 "and side-force ranges within " +
                     format(kSteadyResidual) +
                     " of the mean drag) nor settled periodic shedding (period spread within " +
                     format(kMaxPeriodSpread) + ", amplitude and mean drag within " +
                     format(kStationarity) + " between the first and the last period)"});
        }
    }
    if (!std::isfinite(next.cd) || !std::isfinite(next.cl) || !std::isfinite(next.cy)) {
        next.warnings.push_back(
            {"lbm.diverged", "the lattice state is no longer finite (numerical instability)"});
    }
    result_ = std::move(next);
}

} // namespace orrery::aero::lbm
