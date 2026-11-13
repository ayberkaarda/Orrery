#include "orrery/app/aero_viewport.h"

#include <cstdint>
#include <optional>
#include <string>
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

    // Why a layer is missing or stale after the latest frame (empty when
    // everything is up to date): the sync of new data, then the particles.
    std::string sync_problem;
    std::string particle_problem;

    // Uploads what changed. Revisions are committed only when every upload
    // succeeded, so a failed one is retried on the next frame.
    void sync(const AeroViewState& state) {
        const bool new_geometry = geometry_revision != state.geometry_revision;
        const bool new_view = view_revision != state.view_revision;
        const bool new_frame = !same_conditions(conditions, state.conditions);
        const bool new_layer = cp_layer != state.layers.cp_surface;
        if (!state.geometry) {
            if (new_geometry) {
                clear_all();
                geometry_revision = state.geometry_revision;
            }
            sync_problem.clear();
            return;
        }
        const sim::PreparedGeometry& geometry = *state.geometry;
        const sim::ViewFrame frame = sim::view_frame(state.conditions, geometry.two_dimensional);
        std::string problem;
        if (new_geometry || new_view || new_frame || new_layer) {
            problem = sync_surface(state, geometry, frame);
        }
        if (new_geometry || new_view) {
            const std::string flow_problem = sync_flow(state, geometry, frame);
            problem = problem.empty() ? flow_problem : problem;
        }
        sync_problem = problem;
        if (!problem.empty()) {
            return;
        }
        geometry_revision = state.geometry_revision;
        view_revision = state.view_revision;
        conditions = state.conditions;
        cp_layer = state.layers.cp_surface;
        options = view_options(state.layers, two_dimensional, body_bounds, grid,
                               state.layers.cp_surface && state.surface_cp ? *state.surface_cp
                                                                           : std::vector<double>{});
    }

    void clear_all() {
        (void)view.set_surface(render::AeroSurface{}); // an empty surface is always accepted
        view.clear_flow();
        view.set_forces(std::nullopt);
        grid.reset();
    }

    // The body with its Cp; the reason when it cannot be shown.
    std::string sync_surface(const AeroViewState& state, const sim::PreparedGeometry& geometry,
                             const sim::ViewFrame& frame) {
        const std::vector<double> none;
        const std::vector<double>& cp =
            state.layers.cp_surface && state.surface_cp ? *state.surface_cp : none;
        auto surface = view_surface(geometry, cp, frame);
        if (!surface) {
            return "body: " + surface.error();
        }
        body_bounds = view_bounds(surface.value().mesh);
        two_dimensional = geometry.two_dimensional;
        if (auto set = view.set_surface(surface.value()); !set) {
            return "body: " + set.error();
        }
        return {};
    }

    // The flow field and the force arrows; the reason when the field is
    // rejected (it is then cleared, never left from an older run).
    std::string sync_flow(const AeroViewState& state, const sim::PreparedGeometry& geometry,
                          const sim::ViewFrame& frame) {
        std::string problem;
        if (state.flow) {
            render::FlowField field = view_flow(*state.flow);
            grid = field.velocity.grid;
            if (auto set = view.set_flow(std::move(field)); !set) {
                problem = "flow field: " + set.error();
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
        return problem;
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
    s.particle_problem.clear();
    if (state.running && s.options.particles.enabled) {
        if (auto advanced = s.view.advance(dt, s.options); !advanced) {
            s.particle_problem = "particles: " + advanced.error();
        }
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

std::string AeroViewport::problem() const {
    const Impl& s = *impl_;
    return s.sync_problem.empty() ? s.particle_problem : s.sync_problem;
}

bool AeroViewport::has_flow() const noexcept {
    return impl_->view.has_flow();
}

} // namespace orrery::app
