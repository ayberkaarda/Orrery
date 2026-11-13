#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <string>

#include "../../aero/lbm/lbm_test_support.h"
#include "orrery/aero/lbm/force_history.h"
#include "orrery/aero/lbm/lattice_units.h"
#include "orrery/aero/lbm/lbm2d.h"

// Flow past a circular cylinder (2D, Re = U D / nu) in a channel with a
// Zou-He inlet, an extrapolation outlet and free-slip sides. The run starts
// from the free stream with a small transverse velocity bump behind the
// cylinder so the wake leaves its unstable symmetric state early; the drag
// and lift coefficients are sampled every few steps and analysed over the
// last part of the run (periodic shedding).
namespace orrery::aero::lbm::validation {

struct CylinderCase {
    double reynolds = 100.0;
    double diameter = 10.0;   // lattice cells
    double velocity = 0.05;   // lattice inlet speed (Ma 0.087)
    double upstream = 8.0;    // centre to inlet, in diameters
    double downstream = 22.0; // centre to outlet, in diameters
    double height = 16.0;     // channel height, in diameters
    std::size_t steps = 30000;
    std::size_t sample_every = 10;
    double analyse_after = 0.5; // fraction of the run discarded as start-up
    bool smagorinsky = false;
};

struct CylinderResult {
    double strouhal = 0.0;
    double mean_drag = 0.0;
    double lift_amplitude = 0.0;
    double drag_moving_average = 0.0; // over the last shedding period
    std::size_t cycles = 0;
    double period_spread = 0.0;
    bool settled = false; // settled_oscillation: periodic and stationary
    double max_mach = 0.0;
    double seconds = 0.0;
    std::size_t nx = 0;
    std::size_t ny = 0;
};

inline test::CylinderSetup make_cylinder(const CylinderCase& c) {
    const auto nx = static_cast<std::size_t>(std::lround((c.upstream + c.downstream) * c.diameter));
    const auto ny = static_cast<std::size_t>(std::lround(c.height * c.diameter));
    // The centre sits half a node off the channel axis, so the discrete
    // body is not symmetric about it either.
    const double cx = c.upstream * c.diameter;
    const double cy = 0.5 * static_cast<double>(ny - 1) + 0.5;
    test::CylinderSetup setup =
        test::cylinder_setup(nx, ny, c.diameter, cx, cy, c.velocity, c.reynolds);
    setup.config.smagorinsky = Smagorinsky{c.smagorinsky, 0.1};
    return setup;
}

// Free stream plus a transverse bump of 10 percent of U one diameter behind
// the cylinder.
inline InitialState perturbed_start(const CylinderCase& c, const test::CylinderSetup& setup) {
    const double cx = c.upstream * c.diameter + c.diameter;
    const double cy = 0.5 * static_cast<double>(setup.config.ny - 1);
    const double u = c.velocity;
    const double r2 = c.diameter * c.diameter;
    return [cx, cy, u, r2](std::size_t xi, std::size_t yi) {
        const double dx = static_cast<double>(xi) - cx;
        const double dy = static_cast<double>(yi) - cy;
        return d2q9::Moments{1.0, u, 0.1 * u * std::exp(-(dx * dx + dy * dy) / r2)};
    };
}

inline CylinderResult run_cylinder(Lbm2DSolver& solver, const CylinderCase& c,
                                   const test::CylinderSetup& setup, const std::string& label) {
    test::require_ok(solver.initialize(perturbed_start(c, setup)));
    const double period_guess = c.diameter / (0.17 * c.velocity);
    const auto window =
        static_cast<std::size_t>(std::lround(period_guess / static_cast<double>(c.sample_every)));
    CoefficientHistory history(window);
    CylinderResult result;
    result.nx = setup.config.nx;
    result.ny = setup.config.ny;
    const auto start = std::chrono::steady_clock::now();
    while (solver.steps() < c.steps) {
        test::require_ok(solver.step(c.sample_every));
        const Force2D f = test::take(solver.force());
        history.push(static_cast<double>(solver.steps()),
                     force_coefficients(f.x, f.y, c.velocity, c.diameter));
        if (solver.steps() % 5000 == 0) {
            result.max_mach = std::max(result.max_mach,
                                       lattice_mach(max_speed(test::take(solver.macroscopic()))));
        }
    }
    result.seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    const double from = c.analyse_after * static_cast<double>(c.steps);
    result.mean_drag = mean_since(history.time(), history.drag(), from);
    result.drag_moving_average = history.moving_average().drag;
    if (const auto shedding = dominant_oscillation(history.time(), history.lift(), from)) {
        result.strouhal = shedding->frequency * c.diameter / c.velocity;
        result.lift_amplitude = shedding->amplitude;
        result.cycles = shedding->cycles;
        result.period_spread = shedding->period_spread;
    }
    result.settled =
        settled_oscillation(history.time(), history.drag(), history.lift(), from).has_value();
    std::cout << "measured: cylinder " << label << " Re " << c.reynolds << " D " << c.diameter
              << " grid " << result.nx << "x" << result.ny << " steps " << c.steps << ": St "
              << result.strouhal << ", mean Cd " << result.mean_drag << ", Cd moving average "
              << result.drag_moving_average << ", Cl amplitude " << result.lift_amplitude
              << ", cycles " << result.cycles << ", period spread " << result.period_spread
              << ", settled " << result.settled << ", max lattice Mach " << result.max_mach << ", "
              << result.seconds << " s\n";
    return result;
}

} // namespace orrery::aero::lbm::validation
