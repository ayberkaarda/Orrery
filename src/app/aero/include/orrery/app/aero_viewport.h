#pragma once

#include <memory>
#include <string>

#include "orrery/app/aero_workspace.h"
#include "orrery/core/result.h"
#include "orrery/render/camera.h"
#include "orrery/render/hdr_pipeline.h"

// The scene of the Aero workspace in the window: render::AeroView fed from
// the workspace's AeroViewState. Data is uploaded only when its revision
// changes (a new body, new view data from the solver thread, other layer
// choices), so an idle scene costs one draw. Requires the OpenGL context it
// was created in to be current for every call and for its destruction.
// render::AeroView stays out of this header (pimpl), so the window's
// translation unit does not mix its colour-map types with the space
// renderer's.
namespace orrery::app {

class AeroViewport {
public:
    [[nodiscard]] static core::Result<AeroViewport, std::string> create();

    AeroViewport(AeroViewport&& other) noexcept;
    AeroViewport& operator=(AeroViewport&& other) noexcept;
    AeroViewport(const AeroViewport&) = delete;
    AeroViewport& operator=(const AeroViewport&) = delete;
    ~AeroViewport();

    // Draws into the bound HDR scene target; advances the particles by dt
    // while the solver runs. Returns the number of triangles drawn.
    [[nodiscard]] core::Result<std::size_t, std::string> draw(const AeroViewState& state,
                                                              const render::Camera& camera,
                                                              const render::FrameInfo& frame,
                                                              double dt);
    // Legends into the bound sRGB overlay target.
    void draw_overlay(const render::FrameInfo& frame);

private:
    struct Impl;
    explicit AeroViewport(std::unique_ptr<Impl> impl) noexcept;
    std::unique_ptr<Impl> impl_;
};

} // namespace orrery::app
