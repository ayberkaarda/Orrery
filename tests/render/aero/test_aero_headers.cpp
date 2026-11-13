// Every public render header must be usable together in one translation
// unit: two headers that define the same name differently would make the
// program ill-formed (one-definition rule) even when no single file
// includes both.
#include <type_traits>

#include <catch2/catch_test_macros.hpp>

#include "orrery/render/aero_colormap.h"
#include "orrery/render/aero_compute.h"
#include "orrery/render/aero_field.h"
#include "orrery/render/aero_isosurface.h"
#include "orrery/render/aero_streamlines.h"
#include "orrery/render/aero_surface.h"
#include "orrery/render/aero_view.h"
#include "orrery/render/body_visuals.h"
#include "orrery/render/render_settings.h"
#include "orrery/render/space_renderer.h"

using namespace orrery::render;

TEST_CASE("aero and space render headers compile in one translation unit", "[aero]") {
    STATIC_REQUIRE(!std::is_same_v<RangeMode, ScaleMode>);
    const ColorScale colour{RangeMode::symmetric};
    RenderSettings settings;
    settings.scale_mode = ScaleMode::true_scale;
    CHECK(colour.mode == RangeMode::symmetric);
    CHECK(settings.scale_mode == ScaleMode::true_scale);
}
