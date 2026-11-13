#include "orrery/app/aero_viewport.h"

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "orrery/app/aero_scene.h"
#include "orrery/render/aero_view.h"

namespace orrery::app {

namespace {

bool same_conditions(const aero::FlowConditions& a, const aero::FlowConditions& b) noexcept {
    return a.angle_of_attack == b.angle_of_attack && a.sideslip == b.sideslip &&
           a.speed == b.speed && a.density == b.density;
}

} // namespace

struct AeroViewport::Impl {
    explicit Impl(render::AeroView v) noexcept : view(std::move(v)) {}

    render::AeroView view;
    render::AeroViewOptions options;
    std::optional<std::uint64_t> geometry_revision;
    std::optional<std::uint64_t> view_revision;
    bool cp_layer = true;
    aero::FlowConditions conditions;
    std::optional<render::GridGeometry> grid;
    geom::Bounds body_bounds;
    bool two_dimensional = false;

    void sync(const AeroViewState& state) {
        const bool new_geometry = geometry_revision != state.geometry_revision;
        const bool new_view = view_revision != state.view_revision;
        const bool new_frame = !same_conditions(conditions, state.conditions);
        const bool new_layer = cp_layer != state.layers.cp_surface;
        if (!state.geometry) {
            if (new_geometry) {
                (void)view.set_surface(render::AeroSurface{});
                view.clear_flow();
                view.set_forces(std::nullopt);
                grid.reset();
                geometry_revision = state.geometry_revision;
            }
            return;
        }
        const sim::PreparedGeometry& geometry = *state.geometry;
        const sim::ViewFrame frame = sim::view_frame(state.conditions, geometry.two_dimensional);
        if (new_geometry || new_view || new_frame || new_layer) {
            const std::vector<double> none;
            const std::vector<double>& cp =
                state.layers.cp_surface && state.surface_cp ? *state.surface_cp : none;
            auto surface = view_surface(geometry, cp, frame);
            if (surface) {
                body_bounds = view_bounds(surface.value().mesh);
                (void)view.set_surface(surface.value());
            }
            two_dimensional = geometry.two_dimensional;
        }
        if (new_geometry || new_view) {
            if (state.flow) {
                render::FlowField field = view_flow(*state.flow);
                grid = field.velocity.grid;
                if (!view.set_flow(std::move(field))) {
                    view.clear_flow();
                    grid.reset();
                }
            } else {
                view.clear_flow();
                grid.reset();
            }
            if (state.result) {
                view.set_forces(view_forces(*state.result, state.conditions, geometry, frame));
            } else {
                view.set_forces(std::nullopt);
            }
        }
        geometry_revision = state.geometry_revision;
        view_revision = state.view_revision;
        conditions = state.conditions;
        cp_layer = state.layers.cp_surface;
        options = view_options(state.layers, two_dimensional, body_bounds, grid,
                               state.layers.cp_surface && state.surface_cp ? *state.surface_cp
                                                                           : std::vector<double>{});
    }
};

AeroViewport::AeroViewport(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}
AeroViewport::AeroViewport(AeroViewport&& other) noexcept = default;
AeroViewport& AeroViewport::operator=(AeroViewport&& other) noexcept = default;
AeroViewport::~AeroViewport() = default;

core::Result<AeroViewport, std::string> AeroViewport::create() {
    auto view = render::AeroView::create();
    if (!view) {
        return core::failure(std::string(view.error()));
    }
    return AeroViewport(std::make_unique<Impl>(std::move(view).value()));
}

core::Result<std::size_t, std::string> AeroViewport::draw(const AeroViewState& state,
                                                          const render::Camera& camera,
                                                          const render::FrameInfo& frame,
                                                          double dt) {
    Impl& s = *impl_;
    s.sync(state);
    if (state.running && s.options.particles.enabled) {
        (void)s.view.advance(dt, s.options);
    }
    auto drawn = s.view.draw(camera, frame, s.options);
    if (!drawn) {
        return core::failure(std::string(drawn.error()));
    }
    return drawn.value().surface_triangles;
}

void AeroViewport::draw_overlay(const render::FrameInfo& frame) {
    impl_->view.draw_overlay(frame, impl_->options);
}

} // namespace orrery::app
