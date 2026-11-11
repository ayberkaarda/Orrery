#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <glm/ext/quaternion_double.hpp>
#include <glm/ext/vector_double3.hpp>
#include <glm/ext/vector_float3.hpp>

#include "orrery/core/result.h"
#include "orrery/render/camera.h"
#include "orrery/render/gl_resources.h"
#include "orrery/render/hdr_pipeline.h"

// Pipeline test scene: a lit torus in front of a dark gradient backdrop.
// It exercises every stage of the Phase 4a pipeline at once: floating origin
// (the torus sits 1e4 AU from the world origin), log depth, MSAA, scene-linear
// lighting with highlights well above 1, ACES tone mapping and the sRGB encode.
// The golden snapshot test renders it; it is not an asset for the application.
namespace orrery::render {

// Interleaved position (3 floats, metres) and normal (3 floats) per vertex,
// counter-clockwise triangles seen from outside.
struct TestMesh {
    std::vector<float> vertices;
    std::vector<std::uint32_t> indices;
};

// Torus around the model Z axis; segments >= 3.
[[nodiscard]] TestMesh make_torus(double major_radius, double minor_radius, int major_segments,
                                  int minor_segments);

struct TestSceneLighting {
    glm::vec3 albedo{0.80F, 0.78F, 0.74F};
    glm::dvec3 key_direction{0.08, -0.69, 0.74}; // towards the light (normalised on use)
    glm::vec3 key_irradiance{3.4F, 3.15F, 2.8F};
    glm::dvec3 rim_direction{-0.12, 0.89, 0.11};
    glm::vec3 rim_irradiance{1.2F, 2.0F, 3.4F};
    glm::vec3 sky_ambient{0.045F, 0.060F, 0.095F};
    glm::vec3 ground_ambient{0.020F, 0.016F, 0.012F};
    float shininess = 96.0F;
    glm::vec3 backdrop_horizon{0.016F, 0.020F, 0.034F};
    glm::vec3 backdrop_zenith{0.002F, 0.003F, 0.008F};
};

class TestShapeScene {
public:
    // Requires a current OpenGL 4.6 context.
    [[nodiscard]] static core::Result<TestShapeScene, std::string> create();

    // Draws the backdrop and the torus into the bound HDR scene framebuffer.
    void draw(const Camera& camera, const FrameInfo& frame) const;

    glm::dvec3 world_position{0.0};             // torus centre [m]
    glm::dquat orientation{1.0, 0.0, 0.0, 0.0}; // model -> world
    TestSceneLighting lighting;

private:
    TestShapeScene(Program shape, Program backdrop, Buffer vertices, Buffer indices,
                   VertexArray mesh, VertexArray empty, int index_count) noexcept;
    Program shape_;
    Program backdrop_;
    Buffer vertices_;
    Buffer indices_;
    VertexArray mesh_;
    VertexArray empty_;
    int index_count_ = 0;
};

// The reference view used by the golden snapshot: torus centre at
// (1e4 AU, -3 AU, 0.5 AU), tilted, seen by an Orbit camera at 5.4 m.
[[nodiscard]] glm::dvec3 test_scene_reference_position() noexcept;
[[nodiscard]] glm::dquat test_scene_reference_orientation() noexcept;
[[nodiscard]] Camera test_scene_reference_camera();

} // namespace orrery::render
