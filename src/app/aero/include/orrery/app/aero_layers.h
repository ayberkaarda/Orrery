#pragma once

#include "orrery/render/aero_field.h"

// Layer choices of the Aero view that the workspace offers (docs/aero-workspace.md,
// "View"). A header of its own so that the workspace's public interface does
// not pull in the colour-map types of render::AeroView.
namespace orrery::app {

struct AeroLayers {
    bool cp_surface = true;
    bool section = true;
    render::FieldQuantity section_quantity = render::FieldQuantity::speed;
    // Section position as a fraction of the grid along its axis (0.5: centre).
    double section_fraction = 0.5;
    bool streamlines = true;
    bool particles = false;
    bool q_iso = false;
    double q_level = 0.05;
    bool forces = true;
};

} // namespace orrery::app
