#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "linear_solve.h"
#include "orrery/aero/panel_method.h"

namespace orrery::aero {

using geom::Vector3;

namespace {

constexpr double four_pi = 4.0 * std::numbers::pi;
// Beyond this many panel diameters a panel acts as a point source / doublet.
// APPROXIMATION: far-field monopole and dipole; the neglected terms are of
// relative order (diameter / distance)^2 = 1e-2 of an influence that is
// itself 1e-2 of the near-field ones.
constexpr double far_field_ratio = 10.0;
constexpr Vector3 wake_up{0.0, 0.0, 1.0};

struct Influence {
    double source = 0.0;
    double doublet = 0.0;
};

// Doublet influence of a flat quadrilateral wake panel, positive towards
// cross(c1 - c0, c3 - c0).
double quad_doublet(const Vector3& p, const std::array<Vector3, 4>& c) {
    return triangle_influence(p, c[0], c[1], c[2]).doublet +
           triangle_influence(p, c[0], c[2], c[3]).doublet;
}

void check_conditions(const FlowConditions& c) {
    if (!std::isfinite(c.speed) || !(c.speed > 0.0) || !std::isfinite(c.density) ||
        !(c.density > 0.0) || !std::isfinite(c.angle_of_attack) || !std::isfinite(c.sideslip) ||
        !std::isfinite(c.mach()) || !std::isfinite(c.reynolds(1.0))) {
        throw std::invalid_argument("PanelSolver3D: speed and density must be positive and "
                                    "finite, angles, temperature and viscosity finite");
    }
}

// Panels sharing an edge with each panel, skipping trailing-edge segments
// (the doublet strength jumps across them), then their own such neighbours:
// the stencil of the least-squares surface gradient.
std::vector<std::vector<std::uint32_t>> gradient_stencils(const geom::TriangleMesh& mesh,
                                                          const std::vector<std::uint32_t>& te) {
    using Edge = std::tuple<std::uint32_t, std::uint32_t, std::uint32_t>;
    std::vector<Edge> edges;
    edges.reserve(3 * mesh.triangles.size());
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
        for (std::size_t k = 0; k < 3; ++k) {
            const std::uint32_t a = mesh.triangles[t][k];
            const std::uint32_t b = mesh.triangles[t][(k + 1) % 3];
            edges.emplace_back(std::min(a, b), std::max(a, b), static_cast<std::uint32_t>(t));
        }
    }
    std::sort(edges.begin(), edges.end());
    std::vector<std::pair<std::uint32_t, std::uint32_t>> te_edges;
    te_edges.reserve(te.size());
    for (std::size_t k = 0; k + 1 < te.size(); ++k) {
        te_edges.emplace_back(std::min(te[k], te[k + 1]), std::max(te[k], te[k + 1]));
    }
    std::sort(te_edges.begin(), te_edges.end());

    std::vector<std::vector<std::uint32_t>> ring(mesh.triangles.size());
    for (std::size_t i = 0; i < edges.size();) {
        std::size_t j = i;
        while (j < edges.size() && std::get<0>(edges[j]) == std::get<0>(edges[i]) &&
               std::get<1>(edges[j]) == std::get<1>(edges[i])) {
            ++j;
        }
        const std::pair<std::uint32_t, std::uint32_t> key{std::get<0>(edges[i]),
                                                          std::get<1>(edges[i])};
        if (!std::binary_search(te_edges.begin(), te_edges.end(), key)) {
            for (std::size_t x = i; x < j; ++x) {
                for (std::size_t y = i; y < j; ++y) {
                    if (x != y) {
                        ring[std::get<2>(edges[x])].push_back(std::get<2>(edges[y]));
                    }
                }
            }
        }
        i = j;
    }
    std::vector<std::vector<std::uint32_t>> stencil(ring.size());
    for (std::size_t i = 0; i < ring.size(); ++i) {
        std::vector<std::uint32_t>& s = stencil[i];
        s.reserve(4 * ring[i].size());
        for (const std::uint32_t k : ring[i]) {
            s.push_back(k);
            for (const std::uint32_t k2 : ring[k]) {
                s.push_back(k2);
            }
        }
        std::sort(s.begin(), s.end());
        s.erase(std::unique(s.begin(), s.end()), s.end());
        s.erase(std::remove(s.begin(), s.end(), static_cast<std::uint32_t>(i)), s.end());
    }
    return stencil;
}

// Neighbours whose normal turns by more than this (cosine) lie across a sharp
// edge (wing tip, base) and are used only if the smooth ones do not suffice.
constexpr double smooth_normal_cos = 0.7;

// Surface gradient of mu at a panel as a linear combination of neighbour
// differences: grad mu = sum_k weight_k (mu_k - mu_i).
struct GradientTerm {
    std::uint32_t panel = 0;
    Vector3 weight;
};

// Weighted least-squares surface gradient at panel i, in the tangent plane.
// Neighbour offsets are unfolded into the plane (projected direction, true
// length) so a neighbour around a bend is not brought spuriously close.
std::vector<GradientTerm> gradient_weights(const geom::PanelMesh& pm,
                                           const std::vector<std::uint32_t>& stencil,
                                           std::size_t i) {
    const geom::Panel& p = pm.panels[i];
    const Vector3 n = p.normal;
    const Vector3 helper = std::abs(n.x) < 0.9 ? Vector3{1.0, 0.0, 0.0} : Vector3{0.0, 1.0, 0.0};
    const Vector3 e1 = geom::normalized(geom::cross(n, helper));
    const Vector3 e2 = geom::cross(n, e1);
    struct Sample {
        std::uint32_t panel = 0;
        double x = 0.0;
        double y = 0.0;
        double w = 0.0;
    };
    std::vector<Sample> samples;
    samples.reserve(stencil.size());
    for (const double min_cos : {smooth_normal_cos, -1.0}) {
        samples.clear();
        double a11 = 0.0;
        double a12 = 0.0;
        double a22 = 0.0;
        for (const std::uint32_t k : stencil) {
            if (geom::dot(pm.panels[k].normal, n) < min_cos) {
                continue;
            }
            const Vector3 d = pm.panels[k].collocation - p.collocation;
            const Vector3 in_plane = geom::normalized(d - n * geom::dot(d, n)) * geom::norm(d);
            const double x = geom::dot(in_plane, e1);
            const double y = geom::dot(in_plane, e2);
            const double r2 = x * x + y * y;
            if (!(r2 > 0.0)) {
                continue;
            }
            const double w = 1.0 / r2;
            a11 += w * x * x;
            a12 += w * x * y;
            a22 += w * y * y;
            samples.push_back({k, x, y, w});
        }
        const double trace = a11 + a22;
        const double det = a11 * a22 - a12 * a12;
        if (trace > 0.0 && det > 1e-6 * trace * trace) {
            std::vector<GradientTerm> terms;
            terms.reserve(samples.size());
            for (const Sample& s : samples) {
                const double c1 = (a22 * s.x - a12 * s.y) * s.w / det;
                const double c2 = (a11 * s.y - a12 * s.x) * s.w / det;
                terms.push_back({s.panel, e1 * c1 + e2 * c2});
            }
            return terms;
        }
    }
    // No two independent directions even across sharp edges.
    return {};
}

Vector3 apply_gradient(const std::vector<GradientTerm>& terms, const std::vector<double>& mu,
                       std::size_t i) {
    Vector3 g;
    for (const GradientTerm& t : terms) {
        g += t.weight * (mu[t.panel] - mu[i]);
    }
    return g;
}

// Doublet strength of panel i extrapolated linearly to point `at`, as weights
// on the unknowns.
std::vector<std::pair<std::uint32_t, double>> extrapolation(const geom::PanelMesh& pm,
                                                            const std::vector<GradientTerm>& terms,
                                                            std::uint32_t i, const Vector3& at) {
    const Vector3 r = at - pm.panels[i].collocation;
    std::vector<std::pair<std::uint32_t, double>> out;
    out.reserve(terms.size() + 1);
    double own = 1.0;
    for (const GradientTerm& t : terms) {
        const double c = geom::dot(t.weight, r);
        out.emplace_back(t.panel, c);
        own -= c;
    }
    out.emplace_back(i, own);
    return out;
}

// A panel touching a trailing-edge vertex belongs to the upper (lower)
// surface if its normal has at least this component along +up (-up); side
// faces such as a flat wing tip belong to neither.
constexpr double surface_side_threshold = 0.3;

// Per trailing-edge segment, weights w_k such that sum_k w_k mu_k is the
// upper-minus-lower jump of the doublet strength at the trailing edge.
using KuttaJumps = std::vector<std::vector<std::pair<std::uint32_t, double>>>;

// At each trailing-edge vertex the upper value is the mean of the upper
// panels touching it, each extrapolated linearly to the vertex, and likewise
// below; a segment takes the mean of its two ends. Comparing just the two
// panels on a segment would depend on how the quads at the edge were split
// into triangles: the upper and lower triangles sit at different spanwise
// positions and bias the jump in opposite directions, a spurious circulation
// that grows as the panels at the edge get thinner.
KuttaJumps kutta_jumps(const geom::PanelMesh& pm,
                       const std::vector<std::vector<GradientTerm>>& gradient,
                       const geom::TriangleMesh& mesh, const std::vector<std::uint32_t>& te) {
    const std::size_t segments = te.empty() ? 0 : te.size() - 1;
    KuttaJumps out(segments);
    if (segments == 0) {
        return out;
    }
    std::vector<std::pair<std::uint32_t, std::uint32_t>> position; // (vertex, index in te)
    position.reserve(te.size());
    for (std::size_t k = 0; k < te.size(); ++k) {
        position.emplace_back(te[k], static_cast<std::uint32_t>(k));
    }
    std::sort(position.begin(), position.end());
    std::vector<std::vector<std::uint32_t>> upper(te.size());
    std::vector<std::vector<std::uint32_t>> lower(te.size());
    for (std::size_t t = 0; t < pm.panels.size(); ++t) {
        const double side = geom::dot(pm.panels[t].normal, wake_up);
        for (const std::uint32_t v : pm.panels[t].vertices) {
            auto it = std::lower_bound(position.begin(), position.end(),
                                       std::pair<std::uint32_t, std::uint32_t>{v, 0U});
            for (; it != position.end() && it->first == v; ++it) {
                if (side > surface_side_threshold) {
                    upper[it->second].push_back(static_cast<std::uint32_t>(t));
                } else if (side < -surface_side_threshold) {
                    lower[it->second].push_back(static_cast<std::uint32_t>(t));
                }
            }
        }
    }
    // Fallback for a vertex without a clear upper or lower panel: the panels
    // geom::add_wake assigned to an adjacent segment.
    for (const geom::WakePanel& w : pm.wake) {
        if (w.row != 0 || w.segment >= segments) {
            continue;
        }
        for (const std::size_t k : {std::size_t{w.segment}, std::size_t{w.segment} + 1}) {
            if (upper[k].empty()) {
                upper[k].push_back(w.upper_panel);
            }
            if (lower[k].empty()) {
                lower[k].push_back(w.lower_panel);
            }
        }
    }
    for (std::size_t s = 0; s < segments; ++s) {
        for (const std::size_t k : {s, s + 1}) {
            const Vector3& at = mesh.vertices[te[k]];
            for (const auto& [panels, sign] :
                 {std::pair{&upper[k], 1.0}, std::pair{&lower[k], -1.0}}) {
                const double share = sign * 0.5 / static_cast<double>(panels->size());
                for (const std::uint32_t p : *panels) {
                    for (const auto& [panel, weight] : extrapolation(pm, gradient[p], p, at)) {
                        out[s].emplace_back(panel, share * weight);
                    }
                }
            }
        }
    }
    return out;
}

} // namespace

void PanelSolver3D::setup(const AeroGeometry& geometry, const FlowConditions& conditions) {
    if (geometry.mesh.triangles.empty()) {
        throw std::invalid_argument("PanelSolver3D: the mesh has no triangles");
    }
    for (const Vector3& v : geometry.mesh.vertices) {
        if (!geom::isfinite(v)) {
            throw std::invalid_argument("PanelSolver3D: mesh vertex is not finite");
        }
    }
    for (const geom::Triangle& t : geometry.mesh.triangles) {
        for (const std::uint32_t idx : t) {
            if (idx >= geometry.mesh.vertices.size()) {
                throw std::invalid_argument("PanelSolver3D: triangle index out of range");
            }
        }
    }
    check_conditions(conditions);
    const ReferenceValues& ref = geometry.reference;
    if (!std::isfinite(ref.area) || !(ref.area > 0.0) || !std::isfinite(ref.length) ||
        !(ref.length > 0.0) || !geom::isfinite(ref.moment_centre)) {
        throw std::invalid_argument(
            "PanelSolver3D: reference area and length must be positive and finite");
    }
    if (!(std::isfinite(options_.wake_length) && options_.wake_length > 0.0) ||
        options_.wake_rows < 1) {
        throw std::invalid_argument("PanelSolver3D: wake length and rows must be positive");
    }
    geom::PanelMesh pm = geom::build_panels(geometry.mesh);
    if (!geometry.trailing_edge.empty()) {
        geom::WakeOptions wake;
        // APPROXIMATION (A5): fixed, flat wake along the free stream.
        wake.direction = conditions.drag_direction();
        wake.length = options_.wake_length * ref.length;
        wake.rows = options_.wake_rows;
        wake.up = wake_up;
        geom::add_wake(pm, geometry.mesh, geometry.trailing_edge, wake);
    }
    mesh_ = geometry.mesh;
    trailing_edge_ = geometry.trailing_edge;
    panels_ = std::move(pm);
    reference_ = ref;
    conditions_ = conditions;
    result_ = AeroResult{};
    doublets_.clear();
    trefftz_cl_ = 0.0;
    used_gmres_ = false;
    ready_ = true;
}

bool PanelSolver3D::step() {
    if (!ready_) {
        throw std::logic_error("PanelSolver3D::step called before setup");
    }
    const std::vector<geom::Panel>& panels = panels_.panels;
    const std::size_t count = panels.size();
    const Vector3 v_hat = conditions_.drag_direction(); // unit free stream

    std::vector<double> diameter;
    diameter.reserve(count);
    for (const geom::Panel& p : panels) {
        const Vector3& a = mesh_.vertices[p.vertices[0]];
        const Vector3& b = mesh_.vertices[p.vertices[1]];
        const Vector3& c = mesh_.vertices[p.vertices[2]];
        diameter.push_back(std::max({geom::norm(b - a), geom::norm(c - b), geom::norm(a - c)}));
    }
    // Wake panels: sign that turns the doublet along cross(c1 - c0, c3 - c0)
    // into the upper-minus-lower potential jump.
    std::vector<double> wake_sign;
    wake_sign.reserve(panels_.wake.size());
    for (const geom::WakePanel& w : panels_.wake) {
        const Vector3 nq = geom::cross(w.corners[1] - w.corners[0], w.corners[3] - w.corners[0]);
        wake_sign.push_back(geom::dot(nq, wake_up) >= 0.0 ? 1.0 : -1.0);
    }
    const std::vector<std::vector<std::uint32_t>> stencil =
        gradient_stencils(mesh_, trailing_edge_);
    std::vector<std::vector<GradientTerm>> gradient;
    gradient.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        gradient.push_back(gradient_weights(panels_, stencil[i], i));
    }
    // Kutta condition: the wake strength of a trailing-edge segment is the
    // upper-minus-lower jump of the doublet strength at the trailing edge.
    const KuttaJumps kutta = kutta_jumps(panels_, gradient, mesh_, trailing_edge_);
    const auto jump = [&](std::uint32_t segment, const std::vector<double>& mu) {
        double value = 0.0;
        for (const auto& [k, c] : kutta[segment]) {
            value += c * mu[k];
        }
        return value;
    };

    // Dirichlet condition at each collocation point, approached from inside:
    // sum_j mu_j C_ij + sum_w (mu_u - mu_l) C_iw = -sum_j sigma_j B_ij,
    // sigma_j = -n_j . V (Katz and Plotkin, eq. 12.12, per unit speed).
    detail::DenseSystem system(count);
    for (std::size_t i = 0; i < count; ++i) {
        const Vector3& p = panels[i].collocation;
        double b_sum = 0.0;
        for (std::size_t j = 0; j < count; ++j) {
            const geom::Panel& pj = panels[j];
            Influence inf;
            const Vector3 d = p - pj.collocation;
            const double dist = geom::norm(d);
            if (i != j && dist > far_field_ratio * diameter[j]) {
                inf.source = -pj.area / (four_pi * dist);
                inf.doublet = pj.area * geom::dot(d, pj.normal) / (four_pi * dist * dist * dist);
            } else {
                const PanelInfluence t = triangle_influence(p, mesh_.vertices[pj.vertices[0]],
                                                            mesh_.vertices[pj.vertices[1]],
                                                            mesh_.vertices[pj.vertices[2]]);
                inf.source = t.source;
                // Own panel, limit from inside.
                inf.doublet = i == j ? -0.5 : t.doublet;
            }
            system.at(i, j) = inf.doublet;
            b_sum += inf.source * -geom::dot(pj.normal, v_hat);
        }
        system.b[i] = -b_sum;
        for (std::size_t w = 0; w < panels_.wake.size(); ++w) {
            const geom::WakePanel& wp = panels_.wake[w];
            const double c = wake_sign[w] * quad_doublet(p, wp.corners);
            for (const auto& [k, coef] : kutta[wp.segment]) {
                system.at(i, k) += c * coef;
            }
        }
    }

    AeroResult r;
    used_gmres_ = count > options_.dense_limit;
    detail::LinearSolution solution;
    if (used_gmres_) {
        solution = detail::solve_gmres(system, {options_.gmres_restart, options_.gmres_tolerance,
                                                options_.gmres_max_iterations});
    } else {
        solution = detail::solve_lu(system);
    }
    const double residual = solution.residual;
    r.iterations = solution.iterations;
    r.residuals.push_back(residual);
    r.converged =
        std::isfinite(residual) && residual < std::max(1e-8, 10.0 * options_.gmres_tolerance);

    doublets_ = std::move(solution.x);

    // Surface velocity and pressure.
    const double q = conditions_.dynamic_pressure();
    r.surface.points.reserve(count);
    r.surface.normals.reserve(count);
    r.surface.areas.reserve(count);
    r.surface.cp.reserve(count);
    double wetted = 0.0;
    for (std::size_t i = 0; i < count; ++i) {
        const geom::Panel& p = panels[i];
        const Vector3 tangential = v_hat - p.normal * geom::dot(v_hat, p.normal);
        const Vector3 v = tangential + apply_gradient(gradient[i], doublets_, i);
        const double cp = 1.0 - geom::squared_norm(v);
        const Vector3 f = p.normal * (-cp * q * p.area);
        r.force += f;
        r.moment += geom::cross(p.collocation - reference_.moment_centre, f);
        r.surface.points.push_back(p.collocation);
        r.surface.normals.push_back(p.normal);
        r.surface.areas.push_back(p.area);
        r.surface.cp.push_back(cp);
        wetted += p.area;
    }
    set_coefficients(r, conditions_, reference_);
    r.cd_pressure = r.cd;

    // Trefftz plane: the wake trace seen along the free stream, with the
    // doublet sheet replaced by point vortices at the segment ends.
    const Vector3 up_t = geom::normalized(wake_up - v_hat * geom::dot(wake_up, v_hat));
    const Vector3 span_t = geom::cross(up_t, v_hat);
    struct Segment {
        double s0 = 0.0;
        double u0 = 0.0;
        double s1 = 0.0;
        double u1 = 0.0;
        double mu = 0.0;
    };
    std::vector<Segment> segments;
    for (const geom::WakePanel& w : panels_.wake) {
        if (w.row != 0) {
            continue;
        }
        Segment s{geom::dot(w.corners[0], span_t), geom::dot(w.corners[0], up_t),
                  geom::dot(w.corners[1], span_t), geom::dot(w.corners[1], up_t),
                  jump(w.segment, doublets_)};
        if (s.s1 < s.s0) {
            std::swap(s.s0, s.s1);
            std::swap(s.u0, s.u1);
        }
        segments.push_back(s);
    }
    double lift_sum = 0.0;
    double drag_sum = 0.0;
    for (const Segment& k : segments) {
        const double ds = std::hypot(k.s1 - k.s0, k.u1 - k.u0);
        const double ns = -(k.u1 - k.u0) / ds;
        const double nu = (k.s1 - k.s0) / ds;
        const double ms = 0.5 * (k.s0 + k.s1);
        const double mid_u = 0.5 * (k.u0 + k.u1);
        double w = 0.0;
        for (const Segment& v : segments) {
            // Vortex -mu at the start, +mu at the end (counter-clockwise in
            // the span-up plane positive).
            for (const auto& [gs, gu, g] :
                 {std::tuple{v.s0, v.u0, -v.mu}, std::tuple{v.s1, v.u1, v.mu}}) {
                const double ds2 = ms - gs;
                const double du2 = mid_u - gu;
                const double r2 = ds2 * ds2 + du2 * du2;
                if (r2 > 0.0) {
                    const double f = g / (2.0 * std::numbers::pi * r2);
                    w += f * (-du2 * ns + ds2 * nu);
                }
            }
        }
        lift_sum += k.mu * ds;
        drag_sum += -k.mu * w * ds;
    }
    trefftz_cl_ = 2.0 * lift_sum / reference_.area;
    r.cd_induced = drag_sum / reference_.area;

    const double re = conditions_.reynolds(reference_.length);
    r.cd_friction = flat_plate_skin_friction(re) * wetted / reference_.area;

    r.warnings = check_validity(SolverKind::panel, conditions_, reference_.length);
    if (!r.converged) {
        r.warnings.push_back(
            {"panel.not_converged", "panel: the linear solve did not converge (relative residual " +
                                        std::to_string(residual) + ")"});
    }
    result_ = std::move(r);
    return true;
}

} // namespace orrery::aero
