#include "orrery/aero/lbm/lbm_aero_solver.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

namespace orrery::aero::lbm {

namespace {

constexpr double kLesReynolds = 2000.0;     // automatic Smagorinsky above this Re
constexpr double kMinCellsPerLength = 20.0; // below: resolution warning
constexpr double kMarginalTau = 0.51;       // below: BGK stability warning
constexpr double kLocalMachWarning = 0.2;   // largest lattice Mach in the flow
constexpr double kBlockageWarning = 0.1;    // body height / channel height
constexpr double kSteadyResidual = 1e-3;    // relative change of the averaged cd
// Largest step count (and averaging window in samples): 2^53, below which
// every integer is a double, far above any run that fits in memory and time.
constexpr double kMaxSteps = 9007199254740992.0;

bool positive_finite(double v) noexcept {
    return std::isfinite(v) && v > 0.0;
}

std::string format(double value) {
    std::ostringstream out;
    out << value;
    return out.str();
}

} // namespace

void LbmAeroSolver2D::setup(const AeroGeometry& geometry, const FlowConditions& conditions) {
    solver_.reset();
    units_.reset();
    result_ = AeroResult{};
    setup_warnings_.clear();
    const std::vector<geom::SectionPoint>& section = geometry.section;
    if (section.size() < 3) {
        throw std::invalid_argument("lbm: the 2D solver needs a section of at least 3 points");
    }
    for (const geom::SectionPoint& p : section) {
        if (!std::isfinite(p.x) || !std::isfinite(p.z)) {
            throw std::invalid_argument("lbm: section points must be finite");
        }
    }
    const ReferenceValues& reference = geometry.reference;
    if (!positive_finite(conditions.speed) || !positive_finite(conditions.density) ||
        !positive_finite(conditions.viscosity) || !std::isfinite(conditions.angle_of_attack) ||
        !std::isfinite(conditions.sideslip)) {
        throw std::invalid_argument(
            "lbm: speed, density and viscosity must be positive and finite, angles finite");
    }
    if (!positive_finite(reference.length) || !positive_finite(reference.area)) {
        throw std::invalid_argument("lbm: reference length and area must be positive and finite");
    }
    if (!positive_finite(options_.upstream) || !positive_finite(options_.downstream) ||
        !positive_finite(options_.height) || !positive_finite(options_.duration) ||
        !positive_finite(options_.average_window) || options_.steps_per_call == 0 ||
        options_.sample_every == 0) {
        throw std::invalid_argument("lbm: domain, duration and step options must be positive");
    }
    const double length = reference.length;
    auto units =
        LatticeUnits::create(SiFlow{length, conditions.speed,
                                    conditions.viscosity / conditions.density, conditions.density},
                             options_.resolution);
    if (!units) {
        throw std::invalid_argument(units.error());
    }
    units_ = units.value();
    const double dx = units_->dx();

    // Wind frame: the free stream along +x, lift along +z.
    const double ca = std::cos(conditions.angle_of_attack);
    const double sa = std::sin(conditions.angle_of_attack);
    std::vector<geom::SectionPoint> wind;
    wind.reserve(section.size());
    double xmin = 0.0;
    double xmax = 0.0;
    double zmin = 0.0;
    double zmax = 0.0;
    for (std::size_t k = 0; k < section.size(); ++k) {
        const geom::SectionPoint& p = section[k];
        const geom::SectionPoint w{p.x * ca + p.z * sa, -p.x * sa + p.z * ca};
        xmin = k == 0 ? w.x : std::min(xmin, w.x);
        xmax = k == 0 ? w.x : std::max(xmax, w.x);
        zmin = k == 0 ? w.z : std::min(zmin, w.z);
        zmax = k == 0 ? w.z : std::max(zmax, w.z);
        wind.push_back(w);
    }
    const double nx_real =
        (options_.upstream * length + (xmax - xmin) + options_.downstream * length) / dx;
    const double ny_real = std::max(options_.height * length, (zmax - zmin) + 4.0 * dx) / dx;
    if (!(nx_real < 1e7) || !(ny_real < 1e7)) {
        throw std::invalid_argument("lbm: the domain is too large for the resolution");
    }
    const auto nx = static_cast<std::size_t>(std::ceil(nx_real)) + 1;
    const auto ny = static_cast<std::size_t>(std::ceil(ny_real)) + 1;
    if (nx > geom::max_grid_cells / ny) {
        throw std::invalid_argument("lbm: the lattice exceeds geom::max_grid_cells nodes");
    }
    origin_ = {xmin - options_.upstream * length,
               0.5 * (zmin + zmax) - 0.5 * static_cast<double>(ny - 1) * dx};
    for (geom::SectionPoint& p : wind) {
        p = geom::SectionPoint{(p.x - origin_[0]) / dx, (p.z - origin_[1]) / dx};
    }
    SolidMask2D mask = polygon_mask(nx, ny, wind);
    if (mask.count() == 0) {
        throw std::invalid_argument("lbm: the section covers no lattice node (resolution too low)");
    }

    // Step counts, checked before they are converted to integers (a double
    // outside the integer range does not convert) and before allocating.
    const double cells = units_->cells_per_length();
    const double steps_per_time = cells / units_->lattice_velocity();
    const double total_real = std::ceil(options_.duration * steps_per_time);
    const double window_real =
        options_.average_window * steps_per_time / static_cast<double>(options_.sample_every);
    if (!(total_real < kMaxSteps) || !(window_real < kMaxSteps)) {
        throw std::invalid_argument("lbm: duration and averaging window must stay below " +
                                    format(kMaxSteps) + " lattice steps");
    }

    Lbm2DConfig config;
    config.nx = nx;
    config.ny = ny;
    config.tau = units_->tau();
    config.x_boundary = XBoundary::inflow_outflow;
    config.outlet = Outlet::convective;
    config.bottom = Wall{WallKind::slip, 0.0};
    config.top = Wall{WallKind::slip, 0.0};
    config.inlet_ux = units_->lattice_velocity();
    const double reynolds = units_->reynolds();
    const bool les = options_.les == LesMode::on ||
                     (options_.les == LesMode::automatic && reynolds > kLesReynolds);
    config.smagorinsky = Smagorinsky{les, options_.smagorinsky_constant};

    auto made = options_.backend != nullptr ? make_gpu_solver(*options_.backend, config, mask)
                                            : make_cpu_solver(config, std::move(mask));
    if (!made) {
        throw std::invalid_argument(made.error());
    }
    solver_ = std::move(made).value();

    const geom::Vector3& c = reference.moment_centre;
    centre_ = {(c.x * ca + c.z * sa - origin_[0]) / dx, (-c.x * sa + c.z * ca - origin_[1]) / dx};

    conditions_ = conditions;
    reference_ = reference;
    total_steps_ = static_cast<std::uint64_t>(total_real);
    window_samples_ = std::max<std::size_t>(1, static_cast<std::size_t>(window_real));
    history_ = CoefficientHistory(window_samples_);
    samples_.clear();
    instantaneous_ = Coefficients{};
    previous_cd_ = 0.0;

    // Envelope of the method plus the lattice-specific conditions.
    setup_warnings_ = check_validity(SolverKind::lattice_boltzmann, conditions, length);
    if (conditions.sideslip != 0.0) {
        setup_warnings_.push_back({"lbm.sideslip", "the 2D solver ignores the sideslip angle " +
                                                       format(conditions.sideslip) + " rad"});
    }
    if (cells < kMinCellsPerLength) {
        setup_warnings_.push_back(
            {"lbm.resolution", format(cells) + " cells per reference length; below " +
                                   format(kMinCellsPerLength) +
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
    const double blockage = (zmax - zmin) / (static_cast<double>(ny - 1) * dx);
    if (blockage > kBlockageWarning) {
        setup_warnings_.push_back(
            {"lbm.blockage", "the body fills " + format(100.0 * blockage) +
                                 " percent of the channel height; confinement raises the drag"});
    }
    result_.warnings = setup_warnings_;
}

bool LbmAeroSolver2D::step() {
    if (!solver_ || !units_) {
        throw std::logic_error("lbm: step() called before setup()");
    }
    const LatticeUnits units = *units_;
    const std::uint64_t target =
        std::min<std::uint64_t>(total_steps_, solver_->steps() + options_.steps_per_call);
    const double cells = units.cells_per_length();
    const double u = units.lattice_velocity();
    while (solver_->steps() < target) {
        const std::uint64_t left = target - solver_->steps();
        const std::size_t chunk = left < options_.sample_every ? left : options_.sample_every;
        if (auto stepped = solver_->step(chunk); !stepped) {
            throw std::runtime_error(stepped.error());
        }
        auto force = solver_->force();
        if (!force) {
            throw std::runtime_error(force.error());
        }
        Force2D f = force.value();
        // Moment about the moment centre instead of the lattice origin.
        f.moment -= centre_[0] * f.y - centre_[1] * f.x;
        samples_.push_back(f);
        instantaneous_ = force_coefficients(f.x, f.y, u, cells);
        history_.push(static_cast<double>(solver_->steps()), instantaneous_);
    }
    update_result();
    return result_.iterations >= total_steps_;
}

std::optional<double> LbmAeroSolver2D::strouhal() const {
    if (!units_ || history_.size() < 2) {
        return std::nullopt;
    }
    const std::span<const double> time = history_.time();
    const std::size_t first =
        history_.size() > window_samples_ ? history_.size() - window_samples_ : 0;
    const auto shedding = dominant_oscillation(time, history_.lift(), time[first]);
    if (!shedding) {
        return std::nullopt;
    }
    return shedding->frequency * units_->cells_per_length() / units_->lattice_velocity();
}

void LbmAeroSolver2D::update_result() {
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
    Force2D mean;
    for (std::size_t k = samples_.size() - n; k < samples_.size(); ++k) {
        mean.x += samples_[k].x;
        mean.y += samples_[k].y;
        mean.moment += samples_[k].moment;
    }
    if (n > 0) {
        mean.x /= static_cast<double>(n);
        mean.y /= static_cast<double>(n);
        mean.moment /= static_cast<double>(n);
    }
    const double drag = units.force_per_span_to_si(mean.x);
    const double lift = units.force_per_span_to_si(mean.y);
    next.force = drag * conditions_.drag_direction() + lift * conditions_.lift_direction();
    // Lattice z moment (x right, y up) is minus the body pitching moment
    // about +y (x downstream, z up); one more length unit than the force.
    next.moment = geom::Vector3{0.0, -units.force_per_span_to_si(mean.moment) * dx, 0.0};
    set_coefficients(next, conditions_, reference_);

    const double residual =
        next.cd != 0.0 ? std::abs(next.cd - previous_cd_) / std::abs(next.cd) : 0.0;
    next.residuals.push_back(residual);
    previous_cd_ = next.cd;

    // Wall samples: one per fluid node next to the body.
    auto fields = solver_->macroscopic();
    if (!fields) {
        throw std::runtime_error(fields.error());
    }
    const MacroFields2D& macro = fields.value();
    const Lbm2DConfig& config = solver_->config();
    const double ca = std::cos(conditions_.angle_of_attack);
    const double sa = std::sin(conditions_.angle_of_attack);
    const double q_lattice = 0.5 * units.lattice_velocity() * units.lattice_velocity();
    // Free-stream pressure: the mean density of the column next to the inlet
    // (the pressure level of the channel floats with the drag it carries).
    double rho_inf = 0.0;
    for (std::size_t y = 0; y < config.ny; ++y) {
        rho_inf += macro.density[1 + config.nx * y];
    }
    rho_inf /= static_cast<double>(config.ny);
    std::map<std::uint32_t, std::array<double, 2>> wall;
    for (const BoundaryLink& link : boundary_links(config, solver_->mask())) {
        std::array<double, 2>& normal = wall[link.cell];
        normal[0] += d2q9::cx[link.direction];
        normal[1] += d2q9::cy[link.direction];
    }
    next.surface.points.reserve(wall.size());
    next.surface.normals.reserve(wall.size());
    next.surface.areas.reserve(wall.size());
    next.surface.cp.reserve(wall.size());
    for (const auto& [cell, sum] : wall) {
        const std::size_t column = cell % config.nx;
        const std::size_t row = cell / config.nx;
        const double xw = origin_[0] + static_cast<double>(column) * dx;
        const double zw = origin_[1] + static_cast<double>(row) * dx;
        const double norm = std::hypot(sum[0], sum[1]);
        const double nxw = norm > 0.0 ? sum[0] / norm : 0.0;
        const double nzw = norm > 0.0 ? sum[1] / norm : 0.0;
        next.surface.points.push_back({xw * ca - zw * sa, 0.0, xw * sa + zw * ca});
        next.surface.normals.push_back({nxw * ca - nzw * sa, 0.0, nxw * sa + nzw * ca});
        next.surface.areas.push_back(dx); // per metre of span
        next.surface.cp.push_back(d2q9::cs2 * (macro.density[cell] - rho_inf) / q_lattice);
    }

    // Velocity field on the lattice: body axes coincide with the lattice
    // axes only at zero angle of attack (see docs/lbm.md).
    if (conditions_.angle_of_attack == 0.0) {
        VelocityField field;
        field.dims = {config.nx, 1, config.ny};
        field.origin = geom::Vector3{origin_[0], 0.0, origin_[1]};
        field.spacing = dx;
        field.values.reserve(macro.velocity_x.size());
        for (std::size_t k = 0; k < macro.velocity_x.size(); ++k) {
            field.values.push_back({units.velocity_to_si(macro.velocity_x[k]), 0.0,
                                    units.velocity_to_si(macro.velocity_y[k])});
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
    const bool finished = next.iterations >= total_steps_;
    const auto shedding = strouhal();
    next.converged = finished && (residual < kSteadyResidual || shedding.has_value());
    if (finished && !next.converged) {
        next.warnings.push_back(
            {"lbm.unconverged", "the averaged drag still changes by " + format(residual) +
                                    " per call and no periodic shedding was found"});
    }
    if (!std::isfinite(next.cd) || !std::isfinite(next.cl)) {
        next.warnings.push_back(
            {"lbm.diverged", "the lattice state is no longer finite (numerical instability)"});
    }
    result_ = std::move(next);
}

} // namespace orrery::aero::lbm
