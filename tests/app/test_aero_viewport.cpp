#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "../render/gl_test_support.h"
#include "orrery/aero/lbm3d/lbm3d.h"
#include "orrery/app/aero_viewport.h"
#include "orrery/render/camera.h"
#include "orrery/sim/aero_import.h"
#include "orrery/sim/aero_view_data.h"

// The Aero workspace's scene adapter: data the view rejects is reported
// (AeroViewport::problem) and retried on the next draw instead of being
// marked as shown.

using namespace orrery;

namespace {

std::shared_ptr<const sim::PreparedGeometry> naca_section() {
    scene::GeometryRef ref;
    ref.shape = scene::AeroShape::naca_section;
    auto prepared = sim::load_experiment_geometry(ref, {});
    REQUIRE(prepared.has_value());
    return std::make_shared<const sim::PreparedGeometry>(std::move(prepared).value());
}

// A 4 x 4 x 4 lattice; `filled` false leaves its arrays empty, which the
// view rejects.
std::shared_ptr<const sim::FlowSnapshot> lattice(bool filled) {
    aero::lbm::MacroFields3D fields;
    fields.nx = 4;
    fields.ny = 4;
    fields.nz = 4;
    if (filled) {
        const std::size_t n = fields.nx * fields.ny * fields.nz;
        fields.density.assign(n, 1.0);
        fields.velocity_x.assign(n, 0.05);
        fields.velocity_y.assign(n, 0.0);
        fields.velocity_z.assign(n, 0.0);
        fields.solid.assign(n, 0);
    }
    sim::FlowSnapshot snapshot;
    snapshot.fields = std::move(fields);
    snapshot.origin = {-1.0, -1.0, -1.0};
    snapshot.spacing = 0.5;
    snapshot.reference_speed = 0.05;
    return std::make_shared<const sim::FlowSnapshot>(std::move(snapshot));
}

} // namespace

TEST_CASE("The aero viewport reports rejected layers and retries them",
          "[app][aero][viewport][gl]") {
    render::OffscreenRenderer renderer = render::test::renderer_or_skip(320, 180);
    auto created = app::AeroViewport::create();
    INFO((created ? std::string() : created.error()));
    REQUIRE(created.has_value());
    app::AeroViewport viewport = std::move(created).value();
    render::Camera camera;
    const auto draw = [&](const app::AeroViewState& state) {
        std::optional<std::size_t> triangles;
        auto rendered = renderer.render([&](const render::FrameInfo& frame) {
            auto drawn = viewport.draw(state, camera, frame, 1.0 / 60.0);
            INFO((drawn ? std::string() : drawn.error()));
            REQUIRE(drawn.has_value());
            triangles = drawn.value();
        });
        REQUIRE(rendered.has_value());
        return triangles.value_or(0);
    };

    // A 2D body without section points cannot be drawn: reported, not
    // marked as shown, so the same revision is drawn once the data is good.
    const std::shared_ptr<const sim::PreparedGeometry> good = naca_section();
    sim::PreparedGeometry empty = *good;
    empty.section.clear();
    app::AeroViewState state;
    state.geometry = std::make_shared<const sim::PreparedGeometry>(std::move(empty));
    state.geometry_revision = 1;
    CHECK(draw(state) == 0);
    CHECK(viewport.problem().starts_with("body: "));
    state.geometry = good;
    CHECK(draw(state) > 0);
    CHECK(viewport.problem().empty());

    // A flow field the view rejects: reported and retried the same way.
    state.flow = lattice(false);
    state.view_revision = 1;
    (void)draw(state);
    CHECK(viewport.problem().starts_with("flow field: "));
    CHECK_FALSE(viewport.has_flow());
    state.flow = lattice(true);
    (void)draw(state);
    CHECK(viewport.problem().empty());
    CHECK(viewport.has_flow());
}
