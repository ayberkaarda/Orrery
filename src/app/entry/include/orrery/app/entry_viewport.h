#pragma once

#include <memory>
#include <string>

#include "orrery/app/entry_workspace.h"
#include "orrery/core/result.h"
#include "orrery/render/camera.h"
#include "orrery/render/entry_view.h"
#include "orrery/render/hdr_pipeline.h"

// The Entry workspace's scene: render::EntryView fed from
// EntryWorkspace::view_state(). The track is uploaded when the state's
// revision changes; a failed upload is reported by problem() and retried on
// the next frame. Requires the window's OpenGL context to be current.
namespace orrery::app {

class EntryViewport {
public:
    [[nodiscard]] static core::Result<EntryViewport, std::string> create();

    EntryViewport(EntryViewport&& other) noexcept;
    EntryViewport& operator=(EntryViewport&& other) noexcept;
    EntryViewport(const EntryViewport&) = delete;
    EntryViewport& operator=(const EntryViewport&) = delete;
    ~EntryViewport();

    // Draws into the bound HDR scene framebuffer.
    [[nodiscard]] core::Result<render::EntryDrawStats, std::string>
    draw(const EntryViewState& state, const render::Camera& camera, const render::FrameInfo& frame);

    // Why the track is missing or stale after the latest frame (empty when
    // the view shows the current state).
    [[nodiscard]] std::string problem() const;

private:
    struct Impl;
    explicit EntryViewport(std::unique_ptr<Impl> impl) noexcept;
    std::unique_ptr<Impl> impl_;
};

} // namespace orrery::app
