#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <glm/ext/vector_double3.hpp>

#include "orrery/aero/aero_result.h"
#include "orrery/aero/lbm/lbm2d.h"
#include "orrery/aero/lbm3d/lbm3d.h"

// Flow fields for the aero visualisation (docs/aero-visualisation.md): plain
// data on regular grids, the adapters from the solvers of orrery_aero, the
// derived quantities (speed, vorticity, Q-criterion) and the CPU sampling the
// section planes use. Everything here is double precision and free of
// OpenGL; the GPU paths (aero_streamlines.h, aero_isosurface.h, aero_view.h)
// are checked against it.
namespace orrery::render {

// Largest grid the visualisation accepts (nodes): 256^3.
inline constexpr std::size_t kMaxFieldNodes = std::size_t{1} << 24U;

// Regular grid of nodes in world (body) axes, metres: node (i, j, k) sits at
// origin + spacing * (i, j, k) and has the linear index i + nx (j + ny k),
// the layout of aero::VelocityField and of the LBM solvers. A 2D grid has
// dims[2] == 1 and lies in the plane z = origin.z.
struct GridGeometry {
    std::array<std::size_t, 3> dims{};
    glm::dvec3 origin{0.0};
    double spacing = 1.0;

    // dims[0] * dims[1] * dims[2], 0 when that overflows.
    [[nodiscard]] std::size_t node_count() const noexcept;
    [[nodiscard]] bool is_2d() const noexcept { return dims[2] == 1; }
    [[nodiscard]] std::size_t index(std::size_t i, std::size_t j, std::size_t k) const noexcept {
        return i + dims[0] * (j + dims[1] * k);
    }
    [[nodiscard]] glm::dvec3 node_position(std::size_t i, std::size_t j,
                                           std::size_t k) const noexcept;
    // Position of the last node, origin + spacing * (dims - 1).
    [[nodiscard]] glm::dvec3 upper_corner() const noexcept;
};

// Empty when the grid is usable: at least 2 nodes along x and y (and along z
// unless 2D), at most kMaxFieldNodes nodes, a positive finite spacing and a
// finite origin. Otherwise the reason.
[[nodiscard]] std::optional<std::string> validate(const GridGeometry& grid);

struct ScalarGrid {
    GridGeometry grid;
    std::vector<double> values; // one per node
};

struct VectorGrid {
    GridGeometry grid;
    std::vector<glm::dvec3> values; // one per node
};

// What the view draws of a flow. velocity is required (m/s); pressure is
// optional and shown with `pressure_label` (the LBM adapters give the
// pressure coefficient Cp). solid is empty or holds one flag per node (1 =
// inside a body): solid nodes are excluded from sampling, streamlines and
// particles stop there, and section planes draw them in the body colour.
struct FlowField {
    VectorGrid velocity;
    std::optional<ScalarGrid> pressure;
    std::string pressure_label = "p";
    std::vector<std::uint8_t> solid;
};

// Empty when the field is usable: a valid grid, value counts matching it
// (pressure on the same grid), finite values. Otherwise the reason.
[[nodiscard]] std::optional<std::string> validate(const FlowField& field);

// Lattice units -> world units for the LBM adapters. The lattice node (x, y,
// z) maps to origin + spacing * (x, y, z) (a 2D lattice to the plane
// z = origin.z, lattice y along world +Y), a lattice velocity u to
// velocity_scale * u m/s. With reference_speed > 0 (lattice units, normally
// the inlet speed) pressure is reported as Cp = c_s^2 (rho - 1) /
// (rho_ref reference_speed^2 / 2) with rho_ref = 1; otherwise as the lattice
// gauge pressure c_s^2 (rho - 1).
struct LatticeMapping {
    glm::dvec3 origin{0.0};
    double spacing = 1.0;        // [m per cell]
    double velocity_scale = 1.0; // [m/s per lattice speed]
    double reference_speed = 0.0;
};

[[nodiscard]] FlowField flow_field_from(const aero::VelocityField& velocity);
[[nodiscard]] FlowField flow_field_from(const aero::lbm::MacroFields2D& fields,
                                        const LatticeMapping& mapping = {});
[[nodiscard]] FlowField flow_field_from(const aero::lbm::MacroFields3D& fields,
                                        const LatticeMapping& mapping = {});

// Scalar quantities the view derives from a FlowField.
//   speed                |u| [m/s]
//   vorticity            component of curl u along the section normal (z
//                        for a 2D field) [1/s], signed
//   vorticity_magnitude  |curl u| [1/s]
//   pressure             FlowField::pressure (empty grid when absent)
//   q_criterion          Q = (|Omega|^2 - |S|^2) / 2 [1/s^2], Omega and S the
//                        antisymmetric and symmetric parts of grad u;
//                        Q > 0 where rotation dominates strain (vortex cores,
//                        Hunt, Wray and Moin 1988)
enum class FieldQuantity : std::uint8_t {
    speed,
    vorticity,
    vorticity_magnitude,
    pressure,
    q_criterion
};

// Short label with unit for legends ("|u| [m/s]", "omega [1/s]", ...);
// pressure uses `pressure_label`.
[[nodiscard]] std::string quantity_label(FieldQuantity quantity,
                                         std::string_view pressure_label = "p");
// True for quantities with a meaningful sign (drawn with a diverging map).
[[nodiscard]] bool is_signed(FieldQuantity quantity) noexcept;

enum class Axis : std::uint8_t { x, y, z };

// Velocity gradient by central differences in the interior and one-sided
// differences at the grid edges and next to solid nodes (derivatives along
// z are zero on a 2D grid). Solid nodes get zero. An invalid grid, a value
// count other than the node count or a nonempty `solid` of another size gives
// all zero.
[[nodiscard]] VectorGrid vorticity(const VectorGrid& velocity, std::span<const std::uint8_t> solid);
[[nodiscard]] ScalarGrid q_criterion(const VectorGrid& velocity,
                                     std::span<const std::uint8_t> solid);
[[nodiscard]] ScalarGrid speed(const VectorGrid& velocity);
// `quantity` on the nodes of the field's grid; `normal` selects the
// vorticity component (ignored for the other quantities and for 2D fields).
[[nodiscard]] ScalarGrid derive(const FlowField& field, FieldQuantity quantity,
                                Axis normal = Axis::z);

// Trilinear interpolation (bilinear in the plane of a 2D grid, whose z
// coordinate is ignored); empty outside the grid.
[[nodiscard]] std::optional<double> sample(const ScalarGrid& field, const glm::dvec3& point);
[[nodiscard]] std::optional<glm::dvec3> sample(const VectorGrid& field, const glm::dvec3& point);

// An axis-aligned cut through a scalar grid, sampled at the grid nodes of
// the two in-plane axes and interpolated linearly along the normal axis.
// The in-plane axes are (y, z) for a cut normal to x, (x, z) normal to y and
// (x, y) normal to z. Texel (u, v) sits at corner + u_step * u + v_step * v.
struct SectionImage {
    Axis axis = Axis::z;
    double position = 0.0; // world coordinate along the axis, clamped into the grid
    std::size_t width = 0;
    std::size_t height = 0;
    glm::dvec3 corner{0.0};
    glm::dvec3 u_step{0.0};
    glm::dvec3 v_step{0.0};
    std::vector<double> values;      // width * height, row u fastest
    std::vector<std::uint8_t> solid; // width * height: 1 where the nearer node is solid
};

// The section of `field` normal to `axis` at world coordinate `position`
// (clamped into the grid). A 2D grid has only the z section (any other axis
// gives an empty image). `solid` is empty or one flag per node.
[[nodiscard]] SectionImage sample_section(const ScalarGrid& field,
                                          std::span<const std::uint8_t> solid, Axis axis,
                                          double position);

} // namespace orrery::render
