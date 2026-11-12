#pragma once

#include <optional>
#include <string>
#include <vector>

#include "orrery/geom/triangle_mesh.h"

namespace orrery::geom {

// Parametric primitives (metres). Each primitive has two exact references:
//   - the analytic volume and surface area of the true shape (closed form);
//   - a tessellation whose own polyhedral volume and area are known in closed
//     form as well (exact for flat-faced shapes; for curved ones the regular
//     N-gon frustum formulas below), so the generated mesh can be checked
//     against an exact value at any resolution. See docs/geometry-import.md.

// Point of a meridian profile: distance r from the z axis and height z.
struct ProfilePoint {
    double r = 0.0;
    double z = 0.0;
};

// One piece of a meridian profile: a straight line or a circular arc in the
// (r, z) half plane. Arc points are centre + radius (cos phi, sin phi), phi
// running from phi0 to phi1 (either direction).
struct ProfileSegment {
    enum class Kind { line, arc };
    Kind kind = Kind::line;
    ProfilePoint start;
    ProfilePoint end;
    ProfilePoint centre;
    double radius = 0.0;
    double phi0 = 0.0;
    double phi1 = 0.0;

    [[nodiscard]] static ProfileSegment line(ProfilePoint from, ProfilePoint to) noexcept;
    [[nodiscard]] static ProfileSegment arc(ProfilePoint centre, double radius, double phi0,
                                            double phi1) noexcept;
};

// Tessellation of a solid of revolution: `around` vertices per ring, and each
// arc split into pieces of at most `arc_step` radians. Lines are not split.
struct RevolutionResolution {
    int around = 64;
    double arc_step = 0.05;
};

// Closed meridian profile of a solid of revolution about the z axis: a chain
// of segments from a point on the axis to another point on the axis, with the
// body on the side of the axis (outline traversed from the bottom pole to the
// top pole). Interior points must lie off the axis (r > 0).
class RevolutionProfile {
public:
    RevolutionProfile() = default;
    explicit RevolutionProfile(std::vector<ProfileSegment> segments);

    [[nodiscard]] const std::vector<ProfileSegment>& segments() const noexcept { return segments_; }

    // Empty if the profile is valid, otherwise the reason.
    [[nodiscard]] std::optional<std::string> validate() const;

    // Exact volume (integral of pi r^2 dz) and area (integral of 2 pi r ds)
    // of the true solid of revolution.
    [[nodiscard]] double volume() const noexcept;
    [[nodiscard]] double surface_area() const noexcept;

    // Profile polyline used by the tessellation: first and last point on the
    // axis.
    [[nodiscard]] std::vector<ProfilePoint> polyline(const RevolutionResolution& res) const;

    // Exact volume and area of the tessellation (regular N-gon frustums).
    [[nodiscard]] double polyhedral_volume(const RevolutionResolution& res) const;
    [[nodiscard]] double polyhedral_area(const RevolutionResolution& res) const;

    // Closed, outward-oriented mesh: one pole vertex at each end and one ring
    // of `around` vertices per interior polyline point. Throws
    // std::invalid_argument if the profile or the resolution is invalid, or if
    // the mesh would exceed the default MeshLimits (checked before anything
    // is allocated).
    [[nodiscard]] TriangleMesh mesh(const RevolutionResolution& res) const;

private:
    std::vector<ProfileSegment> segments_;
};

// Sphere of radius r centred at the origin.
[[nodiscard]] RevolutionProfile sphere_profile(double radius);
// Cylinder of radius r, base centred at the origin, axis +z.
[[nodiscard]] RevolutionProfile cylinder_profile(double radius, double height);
// Cone of base radius r, base centred at the origin, apex at (0, 0, height).
[[nodiscard]] RevolutionProfile cone_profile(double radius, double height);

// Blunt entry capsule (Apollo / Orion type): a spherical heat shield of radius
// `nose_radius` with its nose at the origin, a toroidal shoulder of radius
// `shoulder_radius` reaching the maximum diameter, a conical afterbody
// narrowing at `afterbody_half_angle` (radians, from the axis) and a flat top
// of diameter `top_diameter`. Axis +z, heat shield facing -z.
struct CapsuleShape {
    double max_diameter = 3.912;
    double nose_radius = 4.694;
    double shoulder_radius = 0.196;
    double afterbody_half_angle = 0.5759586531581288; // 33 degrees
    double top_diameter = 0.7;

    // APPROXIMATION (G1): Apollo- and Orion-like proportions from published outer
    // mould lines, with the rounded apex replaced by a flat top - geometric
    // stand-ins for drag and heating studies, not flight-accurate shapes.
    [[nodiscard]] static CapsuleShape apollo() noexcept;
    [[nodiscard]] static CapsuleShape orion() noexcept;

    // Empty if the parameters describe a tangent-continuous capsule.
    [[nodiscard]] std::optional<std::string> validate() const;
};

// Throws std::invalid_argument when shape.validate() fails.
[[nodiscard]] RevolutionProfile capsule_profile(const CapsuleShape& shape);

// Ellipsoid with semi-axes a, b, c along x, y, z, centred at the origin.
struct Ellipsoid {
    double a = 1.0;
    double b = 1.0;
    double c = 1.0;

    [[nodiscard]] double volume() const noexcept;
    // Exact area: 4 pi a b c R_G(a^-2, b^-2, c^-2) with Carlson's symmetric
    // integrals evaluated to double precision.
    [[nodiscard]] double surface_area() const;
    // A unit-sphere tessellation scaled by (a, b, c): its volume is exactly
    // a b c times the unit sphere tessellation's volume.
    [[nodiscard]] TriangleMesh mesh(const RevolutionResolution& res) const;
    [[nodiscard]] double polyhedral_volume(const RevolutionResolution& res) const;
};

// Carlson's symmetric elliptic integrals (exposed for tests).
[[nodiscard]] double carlson_rf(double x, double y, double z);
[[nodiscard]] double carlson_rd(double x, double y, double z);
[[nodiscard]] double carlson_rg(double x, double y, double z);

// Rectangular box of size lx, ly, lz centred at the origin (12 triangles).
[[nodiscard]] TriangleMesh box_mesh(double lx, double ly, double lz);
[[nodiscard]] TriangleMesh cube_mesh(double side);
// Flat plate: chord along x, span along y, thickness along z, centred.
[[nodiscard]] TriangleMesh flat_plate_mesh(double chord, double span, double thickness);

// Symmetric wedge (triangular prism): leading edge along y at x = 0, base at
// x = length with half-height length tan(half_angle); span along y, centred.
struct Wedge {
    double length = 1.0;
    double half_angle = 0.17453292519943295; // 10 degrees
    double span = 1.0;

    [[nodiscard]] double volume() const noexcept;
    [[nodiscard]] double surface_area() const noexcept;
    [[nodiscard]] TriangleMesh mesh() const;
};

} // namespace orrery::geom
