#pragma once

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <numbers>
#include <string>
#include <vector>

#include "orrery/aero/aero_solver.h"
#include "orrery/aero/compute/compute_backend.h"
#include "orrery/aero/lbm3d/lbm_aero_solver3d.h"
#include "orrery/geom/primitives.h"

// Drag of a body (sphere, flat plate) in a channel through LbmAeroSolver3D:
// the body mesh from orrery_geom is voxelized by the adapter, the free stream
// enters through the velocity inlet, the sides are free slip. Lengths are in
// reference lengths L (sphere diameter, plate side); the conditions are a
// water-like fluid with Re = rho U L / mu.
namespace orrery::aero::lbm::validation {

struct BodyCase {
    double reynolds = 100.0;
    double cells = 16.0;     // lattice cells per reference length
    double velocity = 0.05;  // lattice inlet speed (Ma 0.087)
    double upstream = 4.0;   // inlet to body, reference lengths
    double downstream = 8.0; // body to outlet
    double width = 6.0;      // side of the square cross-section
    double duration = 30.0;  // convective times L / U
    double average = 10.0;   // convective times averaged
    LesMode les = LesMode::off;
    // Absorbing layers, reference lengths (LbmSolver3DOptions).
    double sponge_strength = 0.1;
    double sponge_inlet = 0.0;
    double sponge_outlet = 2.0;
    double sponge_sides = 0.0;
    double ramp = 3.0; // convective times of the gradual start
    SolidWall wall = SolidWall::interpolated;
};

struct BodyResult {
    double cd = 0.0;
    double cl = 0.0;
    double cy = 0.0;
    bool converged = false;
    double blockage = 0.0;
    double tau = 0.0;
    std::size_t nx = 0;
    std::size_t ny = 0;
    std::size_t nz = 0;
    std::size_t iterations = 0;
    double seconds = 0.0;
    AeroResult result;
    std::vector<double> history_drag;
};

// Standard drag curve of a sphere (Schiller and Naumann 1933, VDI Z. 77:318;
// as quoted by Clift, Grace and Weber, Bubbles, Drops, and Particles, 1978,
// table 5.2), Cd = 24 / Re (1 + 0.15 Re^0.687), within a few percent of the
// measured curve for Re below about 800 (Clift et al. 1978).
inline double schiller_naumann(double reynolds) {
    return 24.0 / reynolds * (1.0 + 0.15 * std::pow(reynolds, 0.687));
}

inline AeroGeometry sphere_geometry() {
    AeroGeometry g;
    g.mesh = geom::sphere_profile(0.5).mesh({64, 0.05});
    g.reference.length = 1.0;
    g.reference.area = 0.25 * std::numbers::pi;
    return g;
}

// Square plate of side 1 and thickness t, normal to the free stream along x.
inline AeroGeometry square_plate_geometry(double thickness) {
    AeroGeometry g;
    g.mesh = geom::box_mesh(thickness, 1.0, 1.0);
    g.reference.length = 1.0;
    g.reference.area = 1.0;
    return g;
}

inline FlowConditions water_at(double reynolds) {
    FlowConditions c;
    c.speed = 0.1;
    c.density = 1000.0;
    c.viscosity = c.density * c.speed * 1.0 / reynolds;
    return c;
}

inline BodyResult run_body(const AeroGeometry& geometry, const BodyCase& c,
                           compute::IComputeBackend* backend, const std::string& label) {
    LbmSolver3DOptions options;
    options.resolution = LatticeResolution{c.cells, c.velocity};
    options.upstream = c.upstream;
    options.downstream = c.downstream;
    options.width = c.width;
    options.duration = c.duration;
    options.average_window = c.average;
    options.les = c.les;
    options.sponge_strength = c.sponge_strength;
    options.sponge_inlet = c.sponge_inlet;
    options.sponge_outlet = c.sponge_outlet;
    options.sponge_sides = c.sponge_sides;
    options.start_ramp = c.ramp;
    options.wall = c.wall;
    options.steps_per_call = 2000;
    options.sample_every = 10;
    options.backend = backend;
    LbmAeroSolver3D solver(options);
    const auto start = std::chrono::steady_clock::now();
    solver.setup(geometry, water_at(c.reynolds));
    while (!solver.step()) {
    }
    BodyResult out;
    out.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    out.result = solver.result();
    out.cd = out.result.cd;
    out.cl = out.result.cl;
    out.cy = out.result.cy;
    out.converged = out.result.converged;
    out.blockage = solver.blockage();
    out.tau = solver.lattice()->config().tau;
    out.nx = solver.lattice()->config().nx;
    out.ny = solver.lattice()->config().ny;
    out.nz = solver.lattice()->config().nz;
    out.iterations = out.result.iterations;
    out.history_drag.assign(solver.history().drag().begin(), solver.history().drag().end());
    std::cout << "measured: " << label << " Re " << c.reynolds << " cells/L " << c.cells << " grid "
              << out.nx << "x" << out.ny << "x" << out.nz << " tau " << out.tau << " blockage "
              << out.blockage << " steps " << out.iterations << ": Cd " << out.cd << " Cl "
              << out.cl << " Cy " << out.cy << ", converged " << out.converged << ", "
              << out.seconds << " s";
    for (const ValidityWarning& w : out.result.warnings) {
        std::cout << " [" << w.code << "]";
    }
    std::cout << "\n";
    return out;
}

} // namespace orrery::aero::lbm::validation
