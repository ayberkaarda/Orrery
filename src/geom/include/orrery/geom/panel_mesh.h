#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "orrery/geom/triangle_mesh.h"

namespace orrery::geom {

// Surface panel for a panel method: one per triangle, collocation point at
// the centroid, outward unit normal and area.
struct Panel {
    Triangle vertices{};
    Vector3 collocation;
    Vector3 normal;
    double area = 0.0;
};

// Flat wake panel shed from one trailing-edge segment: corners in order
// (edge start, edge end, end + step, start + step), with the upper and lower
// surface panels that meet at that segment (for the Kutta condition).
struct WakePanel {
    std::array<Vector3, 4> corners{};
    Vector3 collocation;
    Vector3 normal;
    double area = 0.0;
    std::uint32_t upper_panel = 0;
    std::uint32_t lower_panel = 0;
    // Spanwise segment index along the trailing edge and streamwise row.
    std::uint32_t segment = 0;
    std::uint32_t row = 0;
};

struct PanelMesh {
    std::vector<Panel> panels;
    std::vector<WakePanel> wake;
};

// One panel per triangle. Throws std::invalid_argument if a triangle has no
// area (run repair_mesh first).
[[nodiscard]] PanelMesh build_panels(const TriangleMesh& mesh);

struct TrailingEdgeOptions {
    // Free-stream direction: both faces at a trailing edge face downstream.
    Vector3 flow_direction{1.0, 0.0, 0.0};
    // Spanwise axis used to order the edge (from low to high projection).
    Vector3 span_direction{0.0, 1.0, 0.0};
    // Smallest angle between the two face normals of a sharp edge (radians).
    double min_normal_angle = 2.0;
};

// Sharp edges whose two faces both face downstream, chained into one
// polyline ordered along the span. Empty if there are none or they do not
// form one simple chain.
[[nodiscard]] std::vector<std::uint32_t>
detect_trailing_edge(const TriangleMesh& mesh, const TrailingEdgeOptions& options = {});

struct WakeOptions {
    // Wake panels follow the free stream from the trailing edge.
    // APPROXIMATION: fixed, flat wake (A5 in docs/physics.md) - no roll-up.
    Vector3 direction{1.0, 0.0, 0.0};
    double length = 10.0;
    int rows = 1;
    // Of the two faces at a trailing-edge segment, the one whose normal has
    // the larger component along `up` is the upper surface.
    Vector3 up{0.0, 0.0, 1.0};
};

// Appends wake strips behind the trailing-edge polyline (vertex indices of
// `mesh`, consecutive vertices joined by mesh edges). Throws
// std::invalid_argument if a segment is not an edge with exactly two faces.
void add_wake(PanelMesh& panels, const TriangleMesh& mesh,
              const std::vector<std::uint32_t>& trailing_edge, const WakeOptions& options);

} // namespace orrery::geom
