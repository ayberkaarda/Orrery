#include "orrery/app/entry_viewport.h"

#include <cstdint>
#include <optional>
#include <utility>

namespace orrery::app {

struct EntryViewport::Impl {
    explicit Impl(render::EntryView v) noexcept : view(std::move(v)) {}

    render::EntryView view;
    std::optional<std::uint64_t> revision;
    std::string problem;

    // Uploads the track when it changed; the revision is committed only
    // after a successful upload, so a failed one is retried.
    void sync(const EntryViewState& state) {
        if (revision == state.revision) {
            return;
        }
        auto set = view.set_track(state.track, state.globe);
        if (!set) {
            problem = "trajectory: " + set.error();
            return;
        }
        problem.clear();
        revision = state.revision;
    }
};

EntryViewport::EntryViewport(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}
EntryViewport::EntryViewport(EntryViewport&& other) noexcept = default;
EntryViewport& EntryViewport::operator=(EntryViewport&& other) noexcept = default;
EntryViewport::~EntryViewport() = default;

core::Result<EntryViewport, std::string> EntryViewport::create() {
    auto view = render::EntryView::create();
    if (!view) {
        return core::failure(std::string(view.error()));
    }
    return EntryViewport(std::make_unique<Impl>(std::move(view).value()));
}

core::Result<render::EntryDrawStats, std::string>
EntryViewport::draw(const EntryViewState& state, const render::Camera& camera,
                    const render::FrameInfo& frame) {
    Impl& s = *impl_;
    s.sync(state);
    return s.view.draw(camera, frame, state.options);
}

std::string EntryViewport::problem() const {
    return impl_->problem;
}

} // namespace orrery::app
