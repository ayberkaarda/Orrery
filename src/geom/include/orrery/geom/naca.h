#pragma once

#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

#include "orrery/geom/triangle_mesh.h"

namespace orrery::geom {

// Point of a 2D airfoil section: x along the chord, z normal to it.
struct SectionPoint {
    double x = 0.0;
    double z = 0.0;
};

// NACA 4-digit section of unit chord: maximum camber m (fraction of chord),
// its position p (fraction of chord) and maximum thickness t (fraction of
// chord). "2412" is m = 0.02, p = 0.4, t = 0.12.
//
// Thickness y_t = 5 t (0.2969 sqrt(x) - 0.1260 x - 0.3516 x^2 + 0.2843 x^3
// - 0.1036 x^4), applied normal to the camber line.
// APPROXIMATION (G2): the closed-trailing-edge coefficient -0.1036 replaces the
// original -0.1015 (open trailing edge 0.021 t thick) - the closed edge gives
// a watertight wing and a single wake separation line; the half-thickness
// differs by 0.0105 t x^4, at most 0.13 percent of the chord for t = 0.12.
struct Naca4 {
    double max_camber = 0.0;
    double camber_position = 0.0;
    double thickness = 0.12;

    // Parses "0012", "2412", ... Empty for anything else.
    [[nodiscard]] static std::optional<Naca4> from_designation(std::string_view digits);

    [[nodiscard]] double half_thickness(double x) const noexcept;
    [[nodiscard]] double camber(double x) const noexcept;
    [[nodiscard]] double camber_slope(double x) const noexcept;

    // Exact section area for unit chord: integral of 2 y_t sqrt(1 + y_c'^2)
    // dx (thickness applied normal to the camber line), evaluated by
    // Gauss-Legendre quadrature after x = u^2 removes the sqrt(x) singularity;
    // converged to double precision. For a symmetric section it equals the
    // closed form 10 t (0.2969 * 2/3 - 0.1260/2 - 0.3516/3 + 0.2843/4 -
    // 0.1036/5).
    [[nodiscard]] double section_area() const;

    // Counter-clockwise section polygon (seen with x right and z up) with
    // 2 n points: index 0 is the trailing edge, 1 .. n-1 the upper surface
    // from the trailing edge forward, n the leading edge and n+1 .. 2n-1 the
    // lower surface back to the trailing edge. Stations use cosine spacing
    // x_s = (1 - cos(pi s / n)) / 2. Requires n >= 3.
    [[nodiscard]] std::vector<SectionPoint> section(int n) const;
};

// Shoelace area of a closed polygon (positive when counter-clockwise).
[[nodiscard]] double polygon_area(const std::vector<SectionPoint>& polygon) noexcept;

// Straight, untwisted rectangular wing: the section (scaled to `chord`) lies
// in the x-z plane and is extruded along y from -span/2 to +span/2 in
// `span_divisions` equal strips. Closed by flat tip caps.
struct NacaWing {
    TriangleMesh mesh;
    // Trailing-edge vertices from y = -span/2 to +span/2 (span_divisions + 1).
    std::vector<std::uint32_t> trailing_edge;
    // Exact volume and area of this tessellation (prism of the polygon).
    double polyhedral_volume = 0.0;
    double polyhedral_area = 0.0;
};

// Throws std::invalid_argument for a non-positive chord or span, n outside
// [3, 1e6], span_divisions outside [1, 1e5], or a mesh above the default
// MeshLimits (checked before anything is allocated).
[[nodiscard]] NacaWing naca_wing(const Naca4& airfoil, double chord, double span, int n,
                                 int span_divisions);

} // namespace orrery::geom
