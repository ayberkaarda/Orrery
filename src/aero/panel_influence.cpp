#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>

#include "orrery/aero/panel_method.h"

namespace orrery::aero {

using geom::Vector3;

PanelInfluence triangle_influence(const Vector3& p, const Vector3& a, const Vector3& b,
                                  const Vector3& c) {
    const std::array<Vector3, 3> v{a, b, c};
    const Vector3 n = geom::normalized(geom::cross(b - a, c - a));
    const std::array<Vector3, 3> r{a - p, b - p, c - p};
    const std::array<double, 3> len{geom::norm(r[0]), geom::norm(r[1]), geom::norm(r[2])};

    // Solid angle (van Oosterom and Strackee 1983). The triple product is
    // negative when p lies on the side n points to, so -omega / (4 pi) is the
    // doublet potential that tends to +1/2 there.
    const double triple = geom::dot(r[0], geom::cross(r[1], r[2]));
    const double denom = len[0] * len[1] * len[2] + geom::dot(r[0], r[1]) * len[2] +
                         geom::dot(r[0], r[2]) * len[1] + geom::dot(r[1], r[2]) * len[0];
    const double omega = 2.0 * std::atan2(triple, denom);

    // Source: integral dS / r = sum_k d_k ln((r_k + r_k+1 + l_k) / (r_k + r_k+1 - l_k))
    //                           - |h| |omega|,
    // d_k the in-plane distance from the projection of p to edge k (positive
    // on the inner side), h the height of p above the plane.
    const double h = geom::dot(p - a, n);
    double edge_sum = 0.0;
    for (std::size_t k = 0; k < 3; ++k) {
        const std::size_t k1 = (k + 1) % 3;
        const Vector3 e = v[k1] - v[k];
        const double l = geom::norm(e);
        const double d = geom::dot(p - v[k], geom::cross(n, e)) / l;
        const double s = len[k] + len[k1];
        const double den = s - l;
        if (den > 0.0 && d != 0.0) {
            edge_sum += d * std::log((s + l) / den);
        }
    }
    const double integral = edge_sum - std::abs(h) * std::abs(omega);

    PanelInfluence out;
    out.source = -integral / (4.0 * std::numbers::pi);
    out.doublet = -omega / (4.0 * std::numbers::pi);
    return out;
}

} // namespace orrery::aero
