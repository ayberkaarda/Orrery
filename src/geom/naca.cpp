#include "orrery/geom/naca.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace orrery::geom {

namespace {

constexpr double pi = std::numbers::pi;

// Gauss-Legendre rule on [-1, 1] with `order` points, nodes found by Newton
// iteration on P_n from the Chebyshev guess.
template <std::size_t Order> struct GaussLegendre {
    std::array<double, Order> nodes{};
    std::array<double, Order> weights{};

    GaussLegendre() {
        constexpr auto n = static_cast<double>(Order);
        for (std::size_t i = 0; i < Order; ++i) {
            double x = std::cos(pi * (static_cast<double>(i) + 0.75) / (n + 0.5));
            double derivative = 1.0;
            for (int iteration = 0; iteration < 100; ++iteration) {
                double p0 = 1.0;
                double p1 = x;
                for (std::size_t k = 2; k <= Order; ++k) {
                    const auto kd = static_cast<double>(k);
                    const double p2 = ((2.0 * kd - 1.0) * x * p1 - (kd - 1.0) * p0) / kd;
                    p0 = p1;
                    p1 = p2;
                }
                derivative = n * (x * p1 - p0) / (x * x - 1.0);
                const double step = p1 / derivative;
                x -= step;
                if (std::abs(step) < 1e-16) {
                    break;
                }
            }
            nodes[i] = x;
            weights[i] = 2.0 / ((1.0 - x * x) * derivative * derivative);
        }
    }
};

const GaussLegendre<24>& gauss_rule() {
    static const GaussLegendre<24> rule;
    return rule;
}

// Composite Gauss-Legendre integral of f over [a, b] in `pieces` panels.
template <typename F> double integrate(F f, double a, double b, int pieces) {
    const GaussLegendre<24>& rule = gauss_rule();
    double sum = 0.0;
    const double h = (b - a) / static_cast<double>(pieces);
    for (int i = 0; i < pieces; ++i) {
        const double lo = a + h * static_cast<double>(i);
        double panel = 0.0;
        for (std::size_t k = 0; k < rule.nodes.size(); ++k) {
            panel += rule.weights[k] * f(lo + 0.5 * h * (rule.nodes[k] + 1.0));
        }
        sum += 0.5 * h * panel;
    }
    return sum;
}

std::uint32_t to_index(std::size_t i) {
    if (i > std::numeric_limits<std::uint32_t>::max()) {
        throw std::length_error("NACA wing: index exceeds 32 bits");
    }
    return static_cast<std::uint32_t>(i);
}

bool cambered(const Naca4& a) {
    return a.max_camber != 0.0 && a.camber_position > 0.0 && a.camber_position < 1.0;
}

} // namespace

std::optional<Naca4> Naca4::from_designation(std::string_view digits) {
    if (digits.size() != 4) {
        return std::nullopt;
    }
    for (char c : digits) {
        if (c < '0' || c > '9') {
            return std::nullopt;
        }
    }
    const int m = digits[0] - '0';
    const int p = digits[1] - '0';
    const int t = (digits[2] - '0') * 10 + (digits[3] - '0');
    if (t == 0 || (m == 0) != (p == 0)) {
        return std::nullopt;
    }
    Naca4 a;
    a.max_camber = m / 100.0;
    a.camber_position = p / 10.0;
    a.thickness = t / 100.0;
    return a;
}

double Naca4::half_thickness(double x) const noexcept {
    return 5.0 * thickness *
           (0.2969 * std::sqrt(x) + x * (-0.1260 + x * (-0.3516 + x * (0.2843 - 0.1036 * x))));
}

double Naca4::camber(double x) const noexcept {
    if (!cambered(*this)) {
        return 0.0;
    }
    const double m = max_camber;
    const double p = camber_position;
    if (x < p) {
        return m / (p * p) * (2.0 * p * x - x * x);
    }
    return m / ((1.0 - p) * (1.0 - p)) * (1.0 - 2.0 * p + 2.0 * p * x - x * x);
}

double Naca4::camber_slope(double x) const noexcept {
    if (!cambered(*this)) {
        return 0.0;
    }
    const double m = max_camber;
    const double p = camber_position;
    const double d = x < p ? p * p : (1.0 - p) * (1.0 - p);
    return 2.0 * m / d * (p - x);
}

double Naca4::section_area() const {
    // Integrand 2 y_t(x) sqrt(1 + y_c'(x)^2); with x = u^2, dx = 2 u du and
    // y_t(u^2) 2 u is a polynomial in u.
    auto in_u = [this](double u) {
        const double x = u * u;
        const double slope = camber_slope(x);
        return 2.0 * half_thickness(x) * std::sqrt(1.0 + slope * slope) * 2.0 * u;
    };
    auto in_x = [this](double x) {
        const double slope = camber_slope(x);
        return 2.0 * half_thickness(x) * std::sqrt(1.0 + slope * slope);
    };
    constexpr int pieces = 8;
    if (!cambered(*this)) {
        return integrate(in_u, 0.0, 1.0, pieces);
    }
    // The camber slope has a kink at x = p: integrate each side separately.
    return integrate(in_u, 0.0, std::sqrt(camber_position), pieces) +
           integrate(in_x, camber_position, 1.0, pieces);
}

std::vector<SectionPoint> Naca4::section(int n) const {
    if (n < 3 || n > 1000000) {
        throw std::invalid_argument("Naca4::section: n must be in [3, 1000000]");
    }
    const auto count = static_cast<std::size_t>(n);
    std::vector<SectionPoint> points(2 * count);
    points[0] = {1.0, 0.0};
    points[count] = {0.0, 0.0};
    for (std::size_t s = 1; s < count; ++s) {
        const double x = 0.5 * (1.0 - std::cos(pi * static_cast<double>(s) / n));
        const double yt = half_thickness(x);
        const double theta = std::atan(camber_slope(x));
        const double yc = camber(x);
        points[count - s] = {x - yt * std::sin(theta), yc + yt * std::cos(theta)};
        points[count + s] = {x + yt * std::sin(theta), yc - yt * std::cos(theta)};
    }
    return points;
}

double polygon_area(const std::vector<SectionPoint>& polygon) noexcept {
    double twice = 0.0;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const SectionPoint& a = polygon[i];
        const SectionPoint& b = polygon[(i + 1) % polygon.size()];
        twice += a.x * b.z - b.x * a.z;
    }
    return 0.5 * twice;
}

NacaWing naca_wing(const Naca4& airfoil, double chord, double span, int n, int span_divisions) {
    if (!std::isfinite(chord) || !(chord > 0.0) || !std::isfinite(span) || !(span > 0.0)) {
        throw std::invalid_argument("naca_wing: chord and span must be positive and finite");
    }
    if (span_divisions < 1 || span_divisions > 100000) {
        throw std::invalid_argument("naca_wing: span_divisions must be in [1, 100000]");
    }
    const std::vector<SectionPoint> section = airfoil.section(n);
    const std::size_t ring = section.size();
    const auto strips = static_cast<std::size_t>(span_divisions);

    NacaWing wing;
    TriangleMesh& m = wing.mesh;
    m.vertices.reserve(ring * (strips + 1));
    for (std::size_t j = 0; j <= strips; ++j) {
        const double y = span * (static_cast<double>(j) / static_cast<double>(strips) - 0.5);
        for (const SectionPoint& p : section) {
            m.vertices.push_back({chord * p.x, y, chord * p.z});
        }
        wing.trailing_edge.push_back(to_index(j * ring));
    }
    // k <= ring: index ring wraps to the trailing edge (0).
    auto at = [ring](std::size_t j, std::size_t k) {
        return to_index(j * ring + (k == ring ? 0 : k));
    };
    for (std::size_t j = 0; j < strips; ++j) {
        for (std::size_t k = 0; k < ring; ++k) {
            const std::uint32_t a = at(j, k);
            const std::uint32_t b = at(j, k + 1);
            const std::uint32_t c = at(j + 1, k + 1);
            const std::uint32_t d = at(j + 1, k);
            m.triangles.push_back({a, c, b});
            m.triangles.push_back({a, d, c});
        }
    }
    // Tip caps: strips between the upper and lower point of each station.
    const auto stations = static_cast<std::size_t>(n);
    auto upper = [stations](std::size_t s) { return stations - s; };
    // s <= n, so stations + s <= ring; the trailing edge (s = n) is index 0.
    auto lower = [stations, ring](std::size_t s) {
        return stations + s == ring ? 0 : stations + s;
    };
    for (std::size_t s = 0; s < stations; ++s) {
        std::vector<std::array<std::size_t, 3>> cap;
        if (s > 0) {
            cap.push_back({upper(s + 1), upper(s), lower(s)});
        }
        if (s + 1 < stations) {
            cap.push_back({upper(s + 1), lower(s), lower(s + 1)});
        }
        for (const auto& t : cap) {
            // The polygon is counter-clockwise seen from -y: the y = -span/2
            // cap keeps its order, the y = +span/2 cap is reversed.
            m.triangles.push_back({at(0, t[0]), at(0, t[1]), at(0, t[2])});
            m.triangles.push_back({at(strips, t[0]), at(strips, t[2]), at(strips, t[1])});
        }
    }

    const double area = polygon_area(section) * chord * chord;
    double perimeter = 0.0;
    for (std::size_t k = 0; k < ring; ++k) {
        const SectionPoint& a = section[k];
        const SectionPoint& b = section[(k + 1) % ring];
        perimeter += std::hypot(b.x - a.x, b.z - a.z);
    }
    wing.polyhedral_volume = area * span;
    wing.polyhedral_area = 2.0 * area + perimeter * chord * span;
    return wing;
}

} // namespace orrery::geom
