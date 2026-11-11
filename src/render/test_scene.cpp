#include "orrery/render/test_scene.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <span>
#include <utility>

#include <glm/ext/matrix_double4x4.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>

#include "gl_loader.h"
#include "orrery/core/units.h"
#include "orrery/render/embedded_shaders.h"

namespace orrery::render {

namespace {

constexpr double kTwoPi = 6.283185307179586;

} // namespace

TestMesh make_torus(double major_radius, double minor_radius, int major_segments,
                    int minor_segments) {
    TestMesh mesh;
    const int nu = std::max(major_segments, 3);
    const int nv = std::max(minor_segments, 3);
    mesh.vertices.reserve(static_cast<std::size_t>(nu + 1) * static_cast<std::size_t>(nv + 1) * 6U);
    for (int i = 0; i <= nu; ++i) {
        const double u = kTwoPi * i / nu;
        for (int j = 0; j <= nv; ++j) {
            const double v = kTwoPi * j / nv;
            const double ring = major_radius + minor_radius * std::cos(v);
            const std::array<double, 6> vertex{
                ring * std::cos(u),        ring * std::sin(u),        minor_radius * std::sin(v),
                std::cos(v) * std::cos(u), std::cos(v) * std::sin(u), std::sin(v)};
            for (const double value : vertex) {
                mesh.vertices.push_back(static_cast<float>(value));
            }
        }
    }
    const auto index = [nv](int i, int j) { return static_cast<std::uint32_t>(i * (nv + 1) + j); };
    mesh.indices.reserve(static_cast<std::size_t>(nu) * static_cast<std::size_t>(nv) * 6U);
    for (int i = 0; i < nu; ++i) {
        for (int j = 0; j < nv; ++j) {
            // d(position)/du x d(position)/dv points along the outward normal,
            // so (i, j) -> (i + 1, j) -> (i + 1, j + 1) is counter-clockwise
            // seen from outside.
            mesh.indices.insert(mesh.indices.end(),
                                {index(i, j), index(i + 1, j), index(i + 1, j + 1), index(i, j),
                                 index(i + 1, j + 1), index(i, j + 1)});
        }
    }
    return mesh;
}

TestShapeScene::TestShapeScene(Program shape, Program backdrop, Buffer vertices, Buffer indices,
                               VertexArray mesh, VertexArray empty, int index_count) noexcept
    : shape_(std::move(shape)), backdrop_(std::move(backdrop)), vertices_(std::move(vertices)),
      indices_(std::move(indices)), mesh_(std::move(mesh)), empty_(std::move(empty)),
      index_count_(index_count) {}

core::Result<TestShapeScene, std::string> TestShapeScene::create() {
    const std::array<StageSource, 2> shape_stages{
        StageSource{ShaderStage::vertex,
                    {shaders::test_shape_vert.name, shaders::test_shape_vert.source},
                    {}},
        StageSource{ShaderStage::fragment,
                    {shaders::test_shape_frag.name, shaders::test_shape_frag.source},
                    {{shaders::log_depth.name, shaders::log_depth.source}}},
    };
    auto shape = Program::create(shape_stages);
    if (!shape) {
        return core::failure(shape.error());
    }
    const std::array<StageSource, 2> backdrop_stages{
        StageSource{ShaderStage::vertex,
                    {shaders::fullscreen_vert.name, shaders::fullscreen_vert.source},
                    {}},
        StageSource{ShaderStage::fragment,
                    {shaders::background_frag.name, shaders::background_frag.source},
                    {}},
    };
    auto backdrop = Program::create(backdrop_stages);
    if (!backdrop) {
        return core::failure(backdrop.error());
    }

    const TestMesh torus = make_torus(1.0, 0.38, 128, 64);
    auto vertices = Buffer::from(std::span<const float>(torus.vertices));
    if (!vertices) {
        return core::failure("test scene vertices: " + vertices.error());
    }
    auto indices = Buffer::from(std::span<const std::uint32_t>(torus.indices));
    if (!indices) {
        return core::failure("test scene indices: " + indices.error());
    }
    VertexArray mesh = VertexArray::create();
    constexpr std::size_t kStride = 6 * sizeof(float);
    mesh.set_vertex_buffer(0, vertices.value(), 0, kStride);
    mesh.set_float_attribute(0, 0, 3, 0);
    mesh.set_float_attribute(1, 0, 3, 3 * sizeof(float));
    mesh.set_index_buffer(indices.value());

    TestShapeScene scene(std::move(shape).value(), std::move(backdrop).value(),
                         std::move(vertices).value(), std::move(indices).value(), std::move(mesh),
                         VertexArray::create(), static_cast<int>(torus.indices.size()));
    scene.world_position = test_scene_reference_position();
    scene.orientation = test_scene_reference_orientation();
    return scene;
}

void TestShapeScene::draw(const Camera& camera, const FrameInfo& frame) const {
    // Backdrop: fills the frame behind everything, no depth.
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    backdrop_.set("u_horizon", lighting.backdrop_horizon);
    backdrop_.set("u_zenith", lighting.backdrop_zenith);
    backdrop_.use();
    empty_.bind();
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);

    // Model -> render space: the translation is the floating-origin position
    // of the torus centre, formed in double and cast to float once.
    const glm::dvec3 centre = camera.to_render_double(world_position);
    const glm::dmat4 model =
        glm::translate(glm::dmat4(1.0), centre) * glm::mat4_cast(glm::normalize(orientation));
    shape_.set("u_model", glm::mat4(model));
    shape_.set("u_view_projection", camera.view_projection_matrix(frame.aspect()));
    shape_.set("u_log_depth_scale",
               static_cast<float>(log_depth_scale(camera.lens().log_depth_far)));
    shape_.set("u_albedo", lighting.albedo);
    shape_.set("u_key_direction", glm::vec3(glm::normalize(lighting.key_direction)));
    shape_.set("u_key_irradiance", lighting.key_irradiance);
    shape_.set("u_rim_direction", glm::vec3(glm::normalize(lighting.rim_direction)));
    shape_.set("u_rim_irradiance", lighting.rim_irradiance);
    shape_.set("u_sky_ambient", lighting.sky_ambient);
    shape_.set("u_ground_ambient", lighting.ground_ambient);
    shape_.set("u_shininess", lighting.shininess);
    shape_.use();
    mesh_.bind();
    glDrawElements(GL_TRIANGLES, index_count_, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

glm::dvec3 test_scene_reference_position() noexcept {
    const double au = core::constants::astronomical_unit.value();
    return {1.0e4 * au, -3.0 * au, 0.5 * au};
}

glm::dquat test_scene_reference_orientation() noexcept {
    return glm::normalize(glm::angleAxis(0.45, glm::dvec3(0.0, 0.0, 1.0)) *
                          glm::angleAxis(1.05, glm::dvec3(1.0, 0.0, 0.0)));
}

Camera test_scene_reference_camera() {
    Camera camera;
    CameraLens lens;
    lens.vertical_fov = 0.6981317007977318; // 40 degrees
    lens.near_plane = 0.05;
    // A fixed lens inside the accepted ranges: set_lens cannot reject it.
    (void)camera.set_lens(lens);
    OrbitMode orbit;
    orbit.focus = test_scene_reference_position();
    orbit.distance = 5.4;
    orbit.yaw = -0.55;
    orbit.pitch = 0.32;
    camera.set_mode(orbit);
    camera.update(0.0, CameraInput{}, BodyFrameLookup{});
    return camera;
}

} // namespace orrery::render
