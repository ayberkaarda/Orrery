#include <cmath>
#include <cstddef>
#include <numbers>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "orrery/aero/panel_method.h"

namespace orrery::aero {

namespace {

constexpr double two_pi = 2.0 * std::numbers::pi;

// Laminar-to-turbulent transition Reynolds number of the flat-plate estimate.
constexpr double transition_reynolds = 5e5;

struct Vec2 {
    double x = 0.0;
    double z = 0.0;
};

Vec2 operator-(Vec2 a, Vec2 b) noexcept {
    return {a.x - b.x, a.z - b.z};
}
double dot2(Vec2 a, Vec2 b) noexcept {
    return a.x * b.x + a.z * b.z;
}

// Straight panel from a to b; t is the unit tangent along the traversal,
// m the unit left normal (t rotated by +90 degrees in the x-z plane). For a
// counter-clockwise contour m points into the body; the outward normal is -m.
struct Panel2D {
    Vec2 a;
    Vec2 b;
    Vec2 mid;
    Vec2 t;
    Vec2 m;
    double length = 0.0;
};

// Velocity at a point induced by a unit source and a unit (counter-clockwise)
// vortex distribution on a panel, in global x-z components.
struct Induced {
    Vec2 source;
    Vec2 vortex;
};

// Constant-strength source and vortex panels (Katz and Plotkin, sec. 10.2 and
// 11.2): in panel coordinates (xi along t, eta along m) the unit source gives
// u_xi = ln(r1 / r2) / (2 pi), u_eta = beta / (2 pi), and the unit vortex
// u_xi = -beta / (2 pi), u_eta = ln(r1 / r2) / (2 pi), beta being the angle
// the panel subtends at the point (positive on the +m side). `outside_self`
// evaluates a panel at its own midpoint, approached from outside the body.
Induced induced(const Panel2D& p, Vec2 at, bool outside_self) noexcept {
    const Vec2 d = at - p.a;
    const double xi = dot2(d, p.t);
    const double eta = dot2(d, p.m);
    double log_ratio = 0.0;
    double beta = -std::numbers::pi;
    if (!outside_self) {
        const double r1 = std::hypot(xi, eta);
        const double r2 = std::hypot(xi - p.length, eta);
        log_ratio = std::log(r1 / r2);
        beta = std::atan2(eta * p.length, eta * eta - xi * (p.length - xi));
    }
    const double us = log_ratio / two_pi;
    const double vs = beta / two_pi;
    Induced out;
    out.source = {us * p.t.x + vs * p.m.x, us * p.t.z + vs * p.m.z};
    out.vortex = {-vs * p.t.x + us * p.m.x, -vs * p.t.z + us * p.m.z};
    return out;
}

std::vector<Panel2D> make_panels(const std::vector<geom::SectionPoint>& section) {
    const std::size_t n = section.size();
    std::vector<Panel2D> panels;
    panels.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        const geom::SectionPoint& s0 = section[i];
        const geom::SectionPoint& s1 = section[(i + 1) % n];
        Panel2D p;
        p.a = {s0.x, s0.z};
        p.b = {s1.x, s1.z};
        p.mid = {0.5 * (s0.x + s1.x), 0.5 * (s0.z + s1.z)};
        const Vec2 d = p.b - p.a;
        p.length = std::hypot(d.x, d.z);
        if (!(p.length > 0.0) || !std::isfinite(p.length)) {
            throw std::invalid_argument("PanelSolver2D: section has a zero-length panel");
        }
        p.t = {d.x / p.length, d.z / p.length};
        p.m = {-p.t.z, p.t.x};
        panels.push_back(p);
    }
    return panels;
}

double signed_area(const std::vector<geom::SectionPoint>& s) noexcept {
    double twice = 0.0;
    for (std::size_t i = 0; i < s.size(); ++i) {
        const geom::SectionPoint& a = s[i];
        const geom::SectionPoint& b = s[(i + 1) % s.size()];
        twice += a.x * b.z - b.x * a.z;
    }
    return 0.5 * twice;
}

void check_conditions(const FlowConditions& c, const char* who) {
    if (!std::isfinite(c.speed) || !(c.speed > 0.0) || !std::isfinite(c.density) ||
        !(c.density > 0.0) || !std::isfinite(c.angle_of_attack) || !std::isfinite(c.sideslip) ||
        !std::isfinite(c.mach()) || !std::isfinite(c.reynolds(1.0))) {
        throw std::invalid_argument(std::string(who) +
                                    ": speed and density must be positive and finite, "
                                    "angles, temperature and viscosity finite");
    }
}

void check_reference(const ReferenceValues& r, const char* who) {
    if (!std::isfinite(r.area) || !(r.area > 0.0) || !std::isfinite(r.length) ||
        !(r.length > 0.0) || !geom::isfinite(r.moment_centre)) {
        throw std::invalid_argument(std::string(who) +
                                    ": reference area and length must be positive and finite");
    }
}

} // namespace

double flat_plate_skin_friction(double reynolds) {
    if (!std::isfinite(reynolds) || !(reynolds > 0.0)) {
        throw std::invalid_argument("flat_plate_skin_friction: Reynolds number must be positive");
    }
    // APPROXIMATION (A6): zero-pressure-gradient flat plate, transition at
    // Re = 5e5 (Schlichting, Boundary-Layer Theory, chapters 7 and 21).
    if (reynolds <= transition_reynolds) {
        return 1.328 / std::sqrt(reynolds);
    }
    return 0.455 / std::pow(std::log10(reynolds), 2.58) - 1700.0 / reynolds;
}

AeroGeometry naca_section_geometry(const geom::Naca4& airfoil, double chord, int stations) {
    if (!std::isfinite(chord) || !(chord > 0.0)) {
        throw std::invalid_argument("naca_section_geometry: chord must be positive");
    }
    AeroGeometry g;
    g.section = airfoil.section(stations);
    for (geom::SectionPoint& p : g.section) {
        p.x *= chord;
        p.z *= chord;
    }
    g.reference.area = chord;
    g.reference.length = chord;
    g.reference.moment_centre = {0.25 * chord, 0.0, 0.0};
    return g;
}

void PanelSolver2D::setup(const AeroGeometry& geometry, const FlowConditions& conditions) {
    if (geometry.section.size() < 3) {
        throw std::invalid_argument("PanelSolver2D: the section needs at least 3 points");
    }
    for (const geom::SectionPoint& p : geometry.section) {
        if (!std::isfinite(p.x) || !std::isfinite(p.z)) {
            throw std::invalid_argument("PanelSolver2D: section point is not finite");
        }
    }
    if (!(signed_area(geometry.section) > 0.0)) {
        throw std::invalid_argument("PanelSolver2D: the section must be counter-clockwise");
    }
    check_conditions(conditions, "PanelSolver2D");
    check_reference(geometry.reference, "PanelSolver2D");
    static_cast<void>(make_panels(geometry.section)); // rejects zero-length panels
    section_ = geometry.section;
    reference_ = geometry.reference;
    conditions_ = conditions;
    result_ = AeroResult{};
    circulation_ = 0.0;
    ready_ = true;
}

bool PanelSolver2D::step() {
    if (!ready_) {
        throw std::logic_error("PanelSolver2D::step called before setup");
    }
    const std::vector<Panel2D> panels = make_panels(section_);
    const auto n = static_cast<Eigen::Index>(panels.size());
    const std::size_t count = panels.size();
    const double alpha = conditions_.angle_of_attack;
    const Vec2 v_inf{std::cos(alpha), std::sin(alpha)}; // unit free stream

    // Velocities induced at every midpoint by every panel (unit strengths).
    std::vector<Induced> influence;
    influence.reserve(count * count);
    for (std::size_t i = 0; i < count; ++i) {
        for (std::size_t j = 0; j < count; ++j) {
            influence.push_back(induced(panels[j], panels[i].mid, i == j));
        }
    }
    const auto at = [&](std::size_t i, std::size_t j) -> const Induced& {
        return influence[i * count + j];
    };

    // Unknowns: n source strengths and the common vorticity gamma (per unit
    // free-stream speed). Rows: zero normal velocity at each midpoint (outward
    // normal -m) and the Kutta condition at the trailing edge, where the
    // tangential velocities of the first and last panels (traversed in
    // opposite directions) cancel.
    Eigen::MatrixXd a = Eigen::MatrixXd::Zero(n + 1, n + 1);
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n + 1);
    const std::size_t first = 0;
    const std::size_t last = count - 1;
    for (std::size_t i = 0; i < count; ++i) {
        const auto row = static_cast<Eigen::Index>(i);
        const Vec2 normal{-panels[i].m.x, -panels[i].m.z};
        double vortex_sum = 0.0;
        for (std::size_t j = 0; j < count; ++j) {
            a(row, static_cast<Eigen::Index>(j)) = dot2(at(i, j).source, normal);
            vortex_sum += dot2(at(i, j).vortex, normal);
        }
        a(row, n) = vortex_sum;
        rhs(row) = -dot2(v_inf, normal);
    }
    double kutta_vortex = 0.0;
    for (std::size_t j = 0; j < count; ++j) {
        a(n, static_cast<Eigen::Index>(j)) =
            dot2(at(first, j).source, panels[first].t) + dot2(at(last, j).source, panels[last].t);
        kutta_vortex +=
            dot2(at(first, j).vortex, panels[first].t) + dot2(at(last, j).vortex, panels[last].t);
    }
    a(n, n) = kutta_vortex;
    rhs(n) = -(dot2(v_inf, panels[first].t) + dot2(v_inf, panels[last].t));

    const Eigen::PartialPivLU<Eigen::MatrixXd> lu(a);
    const Eigen::VectorXd x = lu.solve(rhs);
    const double residual = (a * x - rhs).norm() / std::max(rhs.norm(), 1e-300);

    const double gamma = x(n);
    const double q = conditions_.dynamic_pressure();
    AeroResult r;
    r.surface.points.reserve(count);
    r.surface.normals.reserve(count);
    r.surface.areas.reserve(count);
    r.surface.cp.reserve(count);
    double perimeter = 0.0;
    for (std::size_t i = 0; i < count; ++i) {
        double vt = dot2(v_inf, panels[i].t);
        for (std::size_t j = 0; j < count; ++j) {
            vt += x(static_cast<Eigen::Index>(j)) * dot2(at(i, j).source, panels[i].t) +
                  gamma * dot2(at(i, j).vortex, panels[i].t);
        }
        const double cp = 1.0 - vt * vt;
        const geom::Vector3 centre{panels[i].mid.x, 0.0, panels[i].mid.z};
        const geom::Vector3 normal{-panels[i].m.x, 0.0, -panels[i].m.z};
        const geom::Vector3 f = normal * (-cp * q * panels[i].length);
        r.force += f;
        r.moment += geom::cross(centre - reference_.moment_centre, f);
        r.surface.points.push_back(centre);
        r.surface.normals.push_back(normal);
        r.surface.areas.push_back(panels[i].length);
        r.surface.cp.push_back(cp);
        perimeter += panels[i].length;
    }
    // Pressure is evaluated with the angle of attack only (no sideslip): the
    // coefficients use the zero-sideslip axes.
    FlowConditions axes = conditions_;
    axes.sideslip = 0.0;
    set_coefficients(r, axes, reference_);
    r.cd_pressure = r.cd;
    // Counter-clockwise vorticity gamma on a counter-clockwise contour is a
    // counter-clockwise circulation; lift needs a clockwise one.
    circulation_ = -gamma * perimeter * conditions_.speed;

    const double re = conditions_.reynolds(reference_.length);
    r.cd_friction = flat_plate_skin_friction(re) * perimeter / reference_.area;

    r.residuals.push_back(residual);
    r.iterations = 1;
    r.converged = std::isfinite(residual) && residual < 1e-8;
    r.warnings = check_validity(SolverKind::panel, conditions_, reference_.length);
    if (conditions_.sideslip != 0.0) {
        r.warnings.push_back({"panel.sideslip_ignored",
                              "panel: the 2D solver ignores sideslip; the section is "
                              "computed at the angle of attack alone"});
    }
    result_ = std::move(r);
    return true;
}

} // namespace orrery::aero
