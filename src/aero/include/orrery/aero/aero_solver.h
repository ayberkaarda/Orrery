#pragma once

#include <cstdint>
#include <vector>

#include "orrery/aero/aero_result.h"
#include "orrery/aero/flow_conditions.h"
#include "orrery/aero/regime.h"
#include "orrery/geom/naca.h"
#include "orrery/geom/triangle_mesh.h"

namespace orrery::aero {

// Body handed to a solver, in body axes (see FlowConditions), metres.
// A 3D solver uses `mesh`; a 2D solver uses `section`. Each solver documents
// which members it reads and throws std::invalid_argument from setup() if
// they are missing.
struct AeroGeometry {
    // Closed, outward-oriented surface (geom::analyze_mesh / repair_mesh).
    geom::TriangleMesh mesh;
    // Trailing-edge polyline of a lifting body (vertex indices of `mesh`,
    // ordered along the span; geom::detect_trailing_edge). Empty for a
    // non-lifting body.
    std::vector<std::uint32_t> trailing_edge;
    // 2D section in the x-z plane (x downstream, z up), counter-clockwise
    // seen with x right and z up, starting and ending at the trailing edge
    // (geom::Naca4::section ordering).
    std::vector<geom::SectionPoint> section;
    ReferenceValues reference;
};

// Common interface of the aerodynamic solvers (ADR-009).
//
// Usage: setup(), then step() until it returns true (a direct solver finishes
// in its first step, an iterative or time-marching one may take many), then
// read result(). result() may also be read between steps for a live view of
// an iterative solver. Calling setup() again starts over.
class IAeroSolver {
public:
    virtual ~IAeroSolver() = default;

    [[nodiscard]] virtual SolverKind kind() const noexcept = 0;

    // Validates and stores the problem. Throws std::invalid_argument for a
    // geometry the solver cannot use or non-finite / non-positive conditions.
    // Conditions outside the validity envelope are accepted: they produce
    // warnings in the result (never an exception).
    virtual void setup(const AeroGeometry& geometry, const FlowConditions& conditions) = 0;

    // Advances the solution; returns true once the result is final. Throws
    // std::logic_error if setup() has not been called.
    virtual bool step() = 0;

    // Latest result (default-constructed before the first step).
    [[nodiscard]] virtual const AeroResult& result() const noexcept = 0;
};

} // namespace orrery::aero
