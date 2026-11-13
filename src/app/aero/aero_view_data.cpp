#include "orrery/sim/aero_view_data.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <utility>

#include "orrery/aero/lbm/lbm_aero_solver.h"
#include "orrery/aero/lbm3d/lbm_aero_solver3d.h"

namespace orrery::sim {

namespace {

// The 3D solver shifts its lattice a quarter node off the body
// (lbm_aero_solver3d.cpp, kPlacementShift); the placement below repeats its
// rule, which the tests check against the solver's own velocity field.
constexpr double kPlacementShift3D = 0.25;

// Index of the sample nearest to `point` (squared distance), or the size
// when there is none.
std::size_t nearest(const std::vector<geom::Vector3>& samples, const geom::Vector3& point) {
    std::size_t best = samples.size();
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const double d = geom::squared_norm(samples[i] - point);
        if (d < best_distance) {
            best_distance = d;
            best = i;
        }
    }
    return best;
}

} // namespace

ViewFrame view_frame(const aero::FlowConditions& conditions, bool two_dimensional) {
    aero::FlowConditions c = conditions;
    if (two_dimensional) {
        c.sideslip = 0.0;
    }
    ViewFrame frame;
    frame.two_dimensional = two_dimensional;
    frame.drag = c.drag_direction();
    frame.side = c.side_direction();
    frame.lift = c.lift_direction();
    return frame;
}

geom::Vector3 to_view(const geom::Vector3& body, const ViewFrame& frame) noexcept {
    if (frame.two_dimensional) {
        return {geom::dot(body, frame.drag), body.y, geom::dot(body, frame.lift)};
    }
    return {geom::dot(body, frame.drag), geom::dot(body, frame.side), geom::dot(body, frame.lift)};
}

geom::SectionPoint section_to_wind(const geom::SectionPoint& point,
                                   const ViewFrame& frame) noexcept {
    const geom::Vector3 body{point.x, 0.0, point.z};
    return {geom::dot(body, frame.drag), geom::dot(body, frame.lift)};
}

std::vector<double> triangle_cp(const geom::TriangleMesh& mesh, const aero::SurfaceField& surface) {
    if (surface.cp.empty() || surface.points.size() != surface.cp.size()) {
        return {};
    }
    if (surface.cp.size() == mesh.triangles.size()) {
        return surface.cp;
    }
    std::vector<double> cp;
    cp.reserve(mesh.triangles.size());
    for (const geom::Triangle& t : mesh.triangles) {
        const geom::Vector3 centre =
            (mesh.vertices[t[0]] + mesh.vertices[t[1]] + mesh.vertices[t[2]]) / 3.0;
        cp.push_back(surface.cp[nearest(surface.points, centre)]);
    }
    return cp;
}

std::vector<double> panel_cp(const std::vector<geom::SectionPoint>& section,
                             const aero::SurfaceField& surface) {
    if (surface.cp.empty() || surface.points.size() != surface.cp.size() || section.size() < 3) {
        return {};
    }
    if (surface.cp.size() == section.size()) {
        return surface.cp;
    }
    std::vector<double> cp;
    cp.reserve(section.size());
    for (std::size_t i = 0; i < section.size(); ++i) {
        const geom::SectionPoint& a = section[i];
        const geom::SectionPoint& b = section[(i + 1) % section.size()];
        const geom::Vector3 middle{0.5 * (a.x + b.x), 0.0, 0.5 * (a.z + b.z)};
        cp.push_back(surface.cp[nearest(surface.points, middle)]);
    }
    return cp;
}

core::Result<FlowSnapshot, std::string> capture_flow(aero::IAeroSolver& solver,
                                                     const PreparedGeometry& geometry,
                                                     const aero::FlowConditions& conditions,
                                                     const aero::ReferenceValues& reference) {
    FlowSnapshot out;
    if (auto* lbm2 = dynamic_cast<aero::lbm::LbmAeroSolver2D*>(&solver)) {
        aero::lbm::Lbm2DSolver* lattice = lbm2->lattice();
        const std::optional<aero::lbm::LatticeUnits>& lattice_units = lbm2->units();
        if (lattice == nullptr || !lattice_units.has_value()) {
            return core::failure(std::string("the lattice is not set up"));
        }
        auto fields = lattice->macroscopic();
        if (!fields) {
            return core::failure(std::string(fields.error()));
        }
        const aero::lbm::LatticeUnits& units = lattice_units.value();
        const ViewFrame frame = view_frame(conditions, true);
        const geom::SectionPoint centre =
            section_to_wind({reference.moment_centre.x, reference.moment_centre.z}, frame);
        const std::array<double, 2> lattice_centre = lbm2->moment_centre();
        out.spacing = units.dx();
        out.origin = {centre.x - lattice_centre[0] * out.spacing, 0.0,
                      centre.z - lattice_centre[1] * out.spacing};
        out.velocity_scale = units.velocity_to_si(1.0);
        out.reference_speed = units.lattice_velocity();
        out.step = lattice->steps();
        out.fields = std::move(fields).value();
        return out;
    }
    if (auto* lbm3 = dynamic_cast<aero::lbm::LbmAeroSolver3D*>(&solver)) {
        aero::lbm::Lbm3DSolver* lattice = lbm3->lattice();
        const std::optional<aero::lbm::LatticeUnits>& lattice_units = lbm3->units();
        if (lattice == nullptr || !lattice_units.has_value() || geometry.mesh.vertices.empty()) {
            return core::failure(std::string("the lattice is not set up"));
        }
        auto fields = lattice->macroscopic();
        if (!fields) {
            return core::failure(std::string(fields.error()));
        }
        const aero::lbm::LatticeUnits& units = lattice_units.value();
        const ViewFrame frame = view_frame(conditions, false);
        geom::Vector3 lo = to_view(geometry.mesh.vertices.front(), frame);
        geom::Vector3 hi = lo;
        for (const geom::Vector3& v : geometry.mesh.vertices) {
            const geom::Vector3 w = to_view(v, frame);
            lo = {std::min(lo.x, w.x), std::min(lo.y, w.y), std::min(lo.z, w.z)};
            hi = {std::max(hi.x, w.x), std::max(hi.y, w.y), std::max(hi.z, w.z)};
        }
        const aero::lbm::Lbm3DConfig& config = lattice->config();
        const double dx = units.dx();
        const double upstream = aero::lbm::LbmSolver3DOptions{}.upstream;
        out.spacing = dx;
        out.origin = {lo.x - upstream * reference.length - kPlacementShift3D * dx,
                      0.5 * (lo.y + hi.y) -
                          (0.5 * static_cast<double>(config.ny - 1) + kPlacementShift3D) * dx,
                      0.5 * (lo.z + hi.z) -
                          (0.5 * static_cast<double>(config.nz - 1) + kPlacementShift3D) * dx};
        out.velocity_scale = units.velocity_to_si(1.0);
        out.reference_speed = units.lattice_velocity();
        out.step = lattice->steps();
        out.fields = std::move(fields).value();
        return out;
    }
    return core::failure(std::string("only the lattice Boltzmann solvers have a flow field"));
}

} // namespace orrery::sim
