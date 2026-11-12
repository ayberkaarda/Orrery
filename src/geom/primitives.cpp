#include "orrery/geom/primitives.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>

namespace orrery::geom {

namespace {

constexpr double pi = std::numbers::pi;

// Relative tolerance for profile continuity and axis contact.
constexpr double profile_tolerance = 1e-12;

ProfilePoint arc_point(ProfilePoint centre, double radius, double phi) {
    return {centre.r + radius * std::cos(phi), centre.z + radius * std::sin(phi)};
}

double distance(ProfilePoint a, ProfilePoint b) {
    return std::hypot(a.r - b.r, a.z - b.z);
}

// Antiderivatives along an arc r = rc + R cos(phi), z = zc + R sin(phi):
//   pi r^2 dz  = pi R (rc^2 cos + 2 rc R cos^2 + R^2 cos^3) dphi
//   2 pi r ds  = 2 pi R (rc + R cos) |dphi|
double arc_volume_primitive(double rc, double radius, double phi) {
    const double s = std::sin(phi);
    const double c = std::cos(phi);
    return pi * radius *
           (rc * rc * s + rc * radius * (phi + s * c) + radius * radius * (s - s * s * s / 3.0));
}

double arc_area_primitive(double rc, double radius, double phi) {
    return 2.0 * pi * radius * (rc * phi + radius * std::sin(phi));
}

// Smallest r reached by an arc between phi0 and phi1 (both directions).
double arc_min_r(const ProfileSegment& s) {
    double lo = std::min(s.phi0, s.phi1);
    const double hi = std::max(s.phi0, s.phi1);
    double r = std::min(s.start.r, s.end.r);
    // cos reaches -1 at odd multiples of pi.
    const double k = std::ceil((lo - pi) / (2.0 * pi));
    lo = pi + 2.0 * pi * k;
    if (lo <= hi) {
        r = std::min(r, s.centre.r - s.radius);
    }
    return r;
}

int arc_pieces(const ProfileSegment& s, double arc_step) {
    const double pieces = std::ceil(std::abs(s.phi1 - s.phi0) / arc_step);
    return std::max(1, static_cast<int>(std::min(pieces, 1e6)));
}

void check_resolution(const RevolutionResolution& res) {
    if (res.around < 3 || res.around > 100000) {
        throw std::invalid_argument("RevolutionResolution: around must be in [3, 100000]");
    }
    if (!(res.arc_step > 1e-6) || !(res.arc_step <= pi)) {
        throw std::invalid_argument("RevolutionResolution: arc_step must be in (1e-6, pi]");
    }
}

void check_positive(double value, const char* what) {
    if (!std::isfinite(value) || !(value > 0.0)) {
        throw std::invalid_argument(std::string(what) + " must be positive and finite");
    }
}

std::uint32_t to_index(std::size_t i) {
    if (i > std::numeric_limits<std::uint32_t>::max()) {
        throw std::length_error("mesh index exceeds 32 bits");
    }
    return static_cast<std::uint32_t>(i);
}

} // namespace

ProfileSegment ProfileSegment::line(ProfilePoint from, ProfilePoint to) noexcept {
    ProfileSegment s;
    s.kind = Kind::line;
    s.start = from;
    s.end = to;
    return s;
}

ProfileSegment ProfileSegment::arc(ProfilePoint centre, double radius, double phi0,
                                   double phi1) noexcept {
    ProfileSegment s;
    s.kind = Kind::arc;
    s.centre = centre;
    s.radius = radius;
    s.phi0 = phi0;
    s.phi1 = phi1;
    s.start = arc_point(centre, radius, phi0);
    s.end = arc_point(centre, radius, phi1);
    return s;
}

RevolutionProfile::RevolutionProfile(std::vector<ProfileSegment> segments)
    : segments_(std::move(segments)) {}

std::optional<std::string> RevolutionProfile::validate() const {
    if (segments_.empty()) {
        return "profile has no segments";
    }
    double scale = 0.0;
    for (const ProfileSegment& s : segments_) {
        for (const ProfilePoint& p : {s.start, s.end, s.centre}) {
            if (!std::isfinite(p.r) || !std::isfinite(p.z)) {
                return "profile point is not finite";
            }
            scale = std::max({scale, std::abs(p.r), std::abs(p.z)});
        }
        if (s.kind == ProfileSegment::Kind::arc) {
            if (!std::isfinite(s.radius) || !(s.radius > 0.0) || !std::isfinite(s.phi0) ||
                !std::isfinite(s.phi1) || s.phi0 == s.phi1) {
                return "arc needs a positive radius and distinct finite angles";
            }
            scale = std::max(scale, s.radius);
        } else if (distance(s.start, s.end) == 0.0) {
            return "line segment has zero length";
        }
    }
    if (!(scale > 0.0)) {
        return "profile has zero size";
    }
    const double tol = profile_tolerance * scale;
    if (std::abs(segments_.front().start.r) > tol || std::abs(segments_.back().end.r) > tol) {
        return "profile must start and end on the axis";
    }
    for (std::size_t i = 0; i + 1 < segments_.size(); ++i) {
        if (distance(segments_[i].end, segments_[i + 1].start) > tol) {
            return "profile segments are not continuous";
        }
    }
    for (const ProfileSegment& s : segments_) {
        const double min_r =
            s.kind == ProfileSegment::Kind::arc ? arc_min_r(s) : std::min(s.start.r, s.end.r);
        if (min_r < -tol) {
            return "profile crosses the axis";
        }
    }
    return std::nullopt;
}

double RevolutionProfile::volume() const noexcept {
    double v = 0.0;
    for (const ProfileSegment& s : segments_) {
        if (s.kind == ProfileSegment::Kind::line) {
            const double r0 = s.start.r;
            const double r1 = s.end.r;
            v += pi * (s.end.z - s.start.z) * (r0 * r0 + r0 * r1 + r1 * r1) / 3.0;
        } else {
            v += arc_volume_primitive(s.centre.r, s.radius, s.phi1) -
                 arc_volume_primitive(s.centre.r, s.radius, s.phi0);
        }
    }
    return v;
}

double RevolutionProfile::surface_area() const noexcept {
    double a = 0.0;
    for (const ProfileSegment& s : segments_) {
        if (s.kind == ProfileSegment::Kind::line) {
            a += pi * (s.start.r + s.end.r) * distance(s.start, s.end);
        } else {
            a += std::abs(arc_area_primitive(s.centre.r, s.radius, s.phi1) -
                          arc_area_primitive(s.centre.r, s.radius, s.phi0));
        }
    }
    return a;
}

std::vector<ProfilePoint> RevolutionProfile::polyline(const RevolutionResolution& res) const {
    check_resolution(res);
    if (const auto error = validate()) {
        throw std::invalid_argument("RevolutionProfile: " + *error);
    }
    std::vector<ProfilePoint> points;
    points.push_back(segments_.front().start);
    for (const ProfileSegment& s : segments_) {
        if (s.kind == ProfileSegment::Kind::line) {
            points.push_back(s.end);
            continue;
        }
        const int pieces = arc_pieces(s, res.arc_step);
        for (int i = 1; i < pieces; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(pieces);
            points.push_back(arc_point(s.centre, s.radius, s.phi0 + (s.phi1 - s.phi0) * t));
        }
        points.push_back(s.end);
    }
    // The poles lie exactly on the axis (cos(pi / 2) is not exactly zero).
    points.front().r = 0.0;
    points.back().r = 0.0;
    return points;
}

double RevolutionProfile::polyhedral_volume(const RevolutionResolution& res) const {
    const std::vector<ProfilePoint> p = polyline(res);
    const double n = static_cast<double>(res.around);
    // Area of a regular N-gon of circumradius r is k r^2.
    const double k = 0.5 * n * std::sin(2.0 * pi / n);
    double v = 0.0;
    for (std::size_t i = 0; i + 1 < p.size(); ++i) {
        const double r0 = p[i].r;
        const double r1 = p[i + 1].r;
        v += (p[i + 1].z - p[i].z) * k * (r0 * r0 + r0 * r1 + r1 * r1) / 3.0;
    }
    return v;
}

double RevolutionProfile::polyhedral_area(const RevolutionResolution& res) const {
    const std::vector<ProfilePoint> p = polyline(res);
    const double n = static_cast<double>(res.around);
    const double half = pi / n;
    double a = 0.0;
    for (std::size_t i = 0; i + 1 < p.size(); ++i) {
        // N trapezoids with parallel sides 2 r sin(pi/N) and the distance
        // between their apothems (r cos(pi/N)) as height.
        const double dz = p[i + 1].z - p[i].z;
        const double dr = (p[i + 1].r - p[i].r) * std::cos(half);
        a += n * std::sin(half) * (p[i].r + p[i + 1].r) * std::hypot(dz, dr);
    }
    return a;
}

TriangleMesh RevolutionProfile::mesh(const RevolutionResolution& res) const {
    const std::vector<ProfilePoint> p = polyline(res);
    const std::size_t rings = p.size() - 2;
    if (rings == 0) {
        throw std::invalid_argument("RevolutionProfile: profile has no point off the axis");
    }
    for (std::size_t i = 1; i + 1 < p.size(); ++i) {
        if (!(p[i].r > 0.0)) {
            throw std::invalid_argument("RevolutionProfile: interior point on the axis");
        }
    }
    const auto n = static_cast<std::size_t>(res.around);
    TriangleMesh m;
    m.vertices.reserve(rings * n + 2);
    m.vertices.push_back({0.0, 0.0, p.front().z});
    for (std::size_t i = 1; i <= rings; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            const double theta = 2.0 * pi * static_cast<double>(j) / static_cast<double>(n);
            m.vertices.push_back({p[i].r * std::cos(theta), p[i].r * std::sin(theta), p[i].z});
        }
    }
    m.vertices.push_back({0.0, 0.0, p.back().z});
    const std::uint32_t bottom = 0;
    const std::uint32_t top = to_index(m.vertices.size() - 1);
    auto ring = [n](std::size_t i, std::size_t j) { return to_index(1 + (i - 1) * n + (j % n)); };
    m.triangles.reserve(2 * n * rings);
    for (std::size_t j = 0; j < n; ++j) {
        m.triangles.push_back({bottom, ring(1, j + 1), ring(1, j)});
    }
    for (std::size_t i = 1; i < rings; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            const std::uint32_t a = ring(i, j);
            const std::uint32_t b = ring(i, j + 1);
            const std::uint32_t c = ring(i + 1, j + 1);
            const std::uint32_t d = ring(i + 1, j);
            m.triangles.push_back({a, b, c});
            m.triangles.push_back({a, c, d});
        }
    }
    for (std::size_t j = 0; j < n; ++j) {
        m.triangles.push_back({ring(rings, j), ring(rings, j + 1), top});
    }
    return m;
}

RevolutionProfile sphere_profile(double radius) {
    check_positive(radius, "sphere radius");
    return RevolutionProfile({ProfileSegment::arc({0.0, 0.0}, radius, -pi / 2.0, pi / 2.0)});
}

RevolutionProfile cylinder_profile(double radius, double height) {
    check_positive(radius, "cylinder radius");
    check_positive(height, "cylinder height");
    return RevolutionProfile({ProfileSegment::line({0.0, 0.0}, {radius, 0.0}),
                              ProfileSegment::line({radius, 0.0}, {radius, height}),
                              ProfileSegment::line({radius, height}, {0.0, height})});
}

RevolutionProfile cone_profile(double radius, double height) {
    check_positive(radius, "cone radius");
    check_positive(height, "cone height");
    return RevolutionProfile({ProfileSegment::line({0.0, 0.0}, {radius, 0.0}),
                              ProfileSegment::line({radius, 0.0}, {0.0, height})});
}

CapsuleShape CapsuleShape::apollo() noexcept {
    return CapsuleShape{};
}

CapsuleShape CapsuleShape::orion() noexcept {
    CapsuleShape s;
    s.max_diameter = 5.03;
    s.nose_radius = 6.04;
    s.shoulder_radius = 0.25;
    s.afterbody_half_angle = 32.5 * pi / 180.0;
    s.top_diameter = 1.8;
    return s;
}

namespace {

// Derived capsule geometry: shoulder torus centre and the angles at which
// the shoulder meets the heat shield (phi_t, negative) and the cone.
struct CapsuleLayout {
    double radius = 0.0;
    ProfilePoint shoulder_centre;
    double tangent_phi = 0.0;
    ProfilePoint cone_start;
    double top_z = 0.0;
};

CapsuleLayout capsule_layout(const CapsuleShape& s) {
    CapsuleLayout l;
    l.radius = 0.5 * s.max_diameter;
    const double cr = l.radius - s.shoulder_radius;
    const double d = s.nose_radius - s.shoulder_radius;
    // The shoulder circle touches the heat-shield sphere from inside:
    // |shoulder centre - sphere centre| = nose_radius - shoulder_radius.
    const double cz = s.nose_radius - std::sqrt(d * d - cr * cr);
    l.shoulder_centre = {cr, cz};
    l.tangent_phi = std::atan2(cz - s.nose_radius, cr);
    l.cone_start = arc_point(l.shoulder_centre, s.shoulder_radius, s.afterbody_half_angle);
    const double top_r = 0.5 * s.top_diameter;
    l.top_z = l.cone_start.z + (l.cone_start.r - top_r) / std::tan(s.afterbody_half_angle);
    return l;
}

} // namespace

std::optional<std::string> CapsuleShape::validate() const {
    for (double v : {max_diameter, nose_radius, shoulder_radius, afterbody_half_angle}) {
        if (!std::isfinite(v) || !(v > 0.0)) {
            return "capsule parameters must be positive and finite";
        }
    }
    if (!std::isfinite(top_diameter) || top_diameter < 0.0) {
        return "capsule top diameter must be non-negative and finite";
    }
    if (!(afterbody_half_angle < pi / 2.0)) {
        return "afterbody half angle must be below 90 degrees";
    }
    const double radius = 0.5 * max_diameter;
    if (!(shoulder_radius < radius)) {
        return "shoulder radius must be below the maximum radius";
    }
    if (!(nose_radius > radius)) {
        return "nose radius must exceed the maximum radius (heat shield shallower than a "
               "hemisphere)";
    }
    const CapsuleLayout l = capsule_layout(*this);
    if (!(0.5 * top_diameter < l.cone_start.r)) {
        return "top diameter must be below the afterbody base diameter";
    }
    return std::nullopt;
}

RevolutionProfile capsule_profile(const CapsuleShape& shape) {
    if (const auto error = shape.validate()) {
        throw std::invalid_argument("CapsuleShape: " + *error);
    }
    const CapsuleLayout l = capsule_layout(shape);
    const double top_r = 0.5 * shape.top_diameter;
    std::vector<ProfileSegment> segments;
    segments.push_back(
        ProfileSegment::arc({0.0, shape.nose_radius}, shape.nose_radius, -pi / 2.0, l.tangent_phi));
    ProfileSegment shoulder = ProfileSegment::arc(l.shoulder_centre, shape.shoulder_radius,
                                                  l.tangent_phi, shape.afterbody_half_angle);
    // The tangent point is shared exactly by the two arcs.
    shoulder.start = segments.back().end;
    segments.push_back(shoulder);
    segments.push_back(ProfileSegment::line(l.cone_start, {top_r, l.top_z}));
    if (top_r > 0.0) {
        segments.push_back(ProfileSegment::line({top_r, l.top_z}, {0.0, l.top_z}));
    }
    return RevolutionProfile(std::move(segments));
}

double Ellipsoid::volume() const noexcept {
    return 4.0 / 3.0 * pi * a * b * c;
}

double Ellipsoid::surface_area() const {
    check_positive(a, "ellipsoid semi-axis a");
    check_positive(b, "ellipsoid semi-axis b");
    check_positive(c, "ellipsoid semi-axis c");
    return 4.0 * pi * a * b * c * carlson_rg(1.0 / (a * a), 1.0 / (b * b), 1.0 / (c * c));
}

TriangleMesh Ellipsoid::mesh(const RevolutionResolution& res) const {
    check_positive(a, "ellipsoid semi-axis a");
    check_positive(b, "ellipsoid semi-axis b");
    check_positive(c, "ellipsoid semi-axis c");
    TriangleMesh m = sphere_profile(1.0).mesh(res);
    for (Vector3& v : m.vertices) {
        v = {a * v.x, b * v.y, c * v.z};
    }
    return m;
}

double Ellipsoid::polyhedral_volume(const RevolutionResolution& res) const {
    return a * b * c * sphere_profile(1.0).polyhedral_volume(res);
}

namespace {

void check_carlson_args(double x, double y, double z, bool z_positive) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || x < 0.0 || y < 0.0 ||
        z < 0.0 || (z_positive && z == 0.0)) {
        throw std::domain_error("Carlson integral: invalid argument");
    }
}

// Duplication theorem stopping tolerance: the truncated series then has a
// relative error below 1e-17 (Carlson 1995; Numerical Recipes 3rd ed. 6.12).
constexpr double carlson_tolerance = 0.0008;
constexpr int carlson_max_iterations = 200;

} // namespace

double carlson_rf(double x, double y, double z) {
    check_carlson_args(x, y, z, false);
    if ((x == 0.0 && y == 0.0) || (x == 0.0 && z == 0.0) || (y == 0.0 && z == 0.0)) {
        throw std::domain_error("carlson_rf: at most one argument may be zero");
    }
    double xt = x;
    double yt = y;
    double zt = z;
    double ave = 0.0;
    double dx = 0.0;
    double dy = 0.0;
    double dz = 0.0;
    for (int i = 0; i < carlson_max_iterations; ++i) {
        const double sx = std::sqrt(xt);
        const double sy = std::sqrt(yt);
        const double sz = std::sqrt(zt);
        const double lambda = sx * (sy + sz) + sy * sz;
        xt = 0.25 * (xt + lambda);
        yt = 0.25 * (yt + lambda);
        zt = 0.25 * (zt + lambda);
        ave = (xt + yt + zt) / 3.0;
        dx = (ave - xt) / ave;
        dy = (ave - yt) / ave;
        dz = (ave - zt) / ave;
        if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) <= carlson_tolerance) {
            break;
        }
    }
    const double e2 = dx * dy - dz * dz;
    const double e3 = dx * dy * dz;
    return (1.0 + (e2 / 24.0 - 0.1 - 3.0 / 44.0 * e3) * e2 + e3 / 14.0) / std::sqrt(ave);
}

double carlson_rd(double x, double y, double z) {
    check_carlson_args(x, y, z, true);
    if (x == 0.0 && y == 0.0) {
        throw std::domain_error("carlson_rd: x and y may not both be zero");
    }
    constexpr double c1 = 3.0 / 14.0;
    constexpr double c2 = 1.0 / 6.0;
    constexpr double c3 = 9.0 / 22.0;
    constexpr double c4 = 3.0 / 26.0;
    constexpr double c5 = 0.25 * c3;
    constexpr double c6 = 1.5 * c4;
    double xt = x;
    double yt = y;
    double zt = z;
    double sum = 0.0;
    double fac = 1.0;
    double ave = 0.0;
    double dx = 0.0;
    double dy = 0.0;
    double dz = 0.0;
    for (int i = 0; i < carlson_max_iterations; ++i) {
        const double sx = std::sqrt(xt);
        const double sy = std::sqrt(yt);
        const double sz = std::sqrt(zt);
        const double lambda = sx * (sy + sz) + sy * sz;
        sum += fac / (sz * (zt + lambda));
        fac *= 0.25;
        xt = 0.25 * (xt + lambda);
        yt = 0.25 * (yt + lambda);
        zt = 0.25 * (zt + lambda);
        ave = 0.2 * (xt + yt + 3.0 * zt);
        dx = (ave - xt) / ave;
        dy = (ave - yt) / ave;
        dz = (ave - zt) / ave;
        if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) <= carlson_tolerance) {
            break;
        }
    }
    const double ea = dx * dy;
    const double eb = dz * dz;
    const double ec = ea - eb;
    const double ed = ea - 6.0 * eb;
    const double ee = ed + ec + ec;
    return 3.0 * sum + fac *
                           (1.0 + ed * (-c1 + c5 * ed - c6 * dz * ee) +
                            dz * (c2 * ee + dz * (-c3 * ec + dz * c4 * ea))) /
                           (ave * std::sqrt(ave));
}

double carlson_rg(double x, double y, double z) {
    check_carlson_args(x, y, z, false);
    // Put the middle value last: (x - z)(y - z) <= 0 makes every term of
    // 2 R_G = z R_F - (x - z)(y - z) R_D / 3 + sqrt(x y / z) non-negative.
    std::array<double, 3> v{x, y, z};
    std::sort(v.begin(), v.end());
    const double lo = v[0];
    const double mid = v[1];
    const double hi = v[2];
    if (mid == 0.0) {
        // R_G(0, 0, z) = sqrt(z) / 2.
        return 0.5 * std::sqrt(hi);
    }
    return 0.5 *
           (mid * carlson_rf(lo, hi, mid) -
            (lo - mid) * (hi - mid) * carlson_rd(lo, hi, mid) / 3.0 + std::sqrt(lo * hi / mid));
}

TriangleMesh box_mesh(double lx, double ly, double lz) {
    check_positive(lx, "box size x");
    check_positive(ly, "box size y");
    check_positive(lz, "box size z");
    TriangleMesh m;
    // Vertex i has x, y, z bits (i & 1), (i & 2), (i & 4).
    for (int i = 0; i < 8; ++i) {
        m.vertices.push_back({(i & 1) != 0 ? 0.5 * lx : -0.5 * lx,
                              (i & 2) != 0 ? 0.5 * ly : -0.5 * ly,
                              (i & 4) != 0 ? 0.5 * lz : -0.5 * lz});
    }
    m.triangles = {{0, 4, 6}, {0, 6, 2}, {1, 3, 7}, {1, 7, 5}, {0, 1, 5}, {0, 5, 4},
                   {2, 6, 7}, {2, 7, 3}, {0, 2, 3}, {0, 3, 1}, {4, 5, 7}, {4, 7, 6}};
    return m;
}

TriangleMesh cube_mesh(double side) {
    return box_mesh(side, side, side);
}

TriangleMesh flat_plate_mesh(double chord, double span, double thickness) {
    return box_mesh(chord, span, thickness);
}

double Wedge::volume() const noexcept {
    return length * length * std::tan(half_angle) * span;
}

double Wedge::surface_area() const noexcept {
    const double h = length * std::tan(half_angle);
    return 2.0 * length * h + 2.0 * span * length / std::cos(half_angle) + 2.0 * span * h;
}

TriangleMesh Wedge::mesh() const {
    check_positive(length, "wedge length");
    check_positive(span, "wedge span");
    if (!std::isfinite(half_angle) || !(half_angle > 0.0) || !(half_angle < pi / 2.0)) {
        throw std::invalid_argument("wedge half angle must be in (0, 90) degrees");
    }
    const double h = length * std::tan(half_angle);
    const double s = 0.5 * span;
    TriangleMesh m;
    m.vertices = {{0.0, -s, 0.0}, {0.0, s, 0.0},    {length, -s, h},
                  {length, s, h}, {length, -s, -h}, {length, s, -h}};
    m.triangles = {{0, 2, 3}, {0, 3, 1}, {0, 1, 5}, {0, 5, 4},
                   {4, 5, 3}, {4, 3, 2}, {0, 4, 2}, {1, 3, 5}};
    return m;
}

} // namespace orrery::geom
