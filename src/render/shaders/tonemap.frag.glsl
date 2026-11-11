#version 460 core
// HDR resolve: exposure -> ACES filmic tone curve -> linear [0, 1] output.
// The output attachment is GL_SRGB8_ALPHA8 with GL_FRAMEBUFFER_SRGB enabled,
// so the hardware applies the sRGB transfer function on store.
// The CPU reference is orrery::render::tone_map_aces (tone_map.cpp).

layout(binding = 0) uniform sampler2D u_hdr;
uniform float u_exposure;

layout(location = 0) out vec4 o_color;

// APPROXIMATION: ACES RRT + ODT (sRGB, 100 nit) - Stephen Hill's fit
// (BakingLab, MIT) of the reference transforms instead of the full CTL
// pipeline - a few percent from the reference over [0, ~16] scene-linear.
//
// GLSL matrices are column-major; each constructor below lists the rows of
// the published (row-major) matrices, and `v * M` multiplies by the published
// matrix from the left.
const mat3 kAcesInput = mat3(
    0.59719, 0.35458, 0.04823,
    0.07600, 0.90834, 0.01566,
    0.02840, 0.13383, 0.83777);
const mat3 kAcesOutput = mat3(
    1.60475, -0.53108, -0.07367,
    -0.10208, 1.10813, -0.00605,
    -0.00327, -0.07276, 1.07602);

vec3 rrt_and_odt_fit(vec3 v) {
    vec3 a = v * (v + 0.0245786) - 0.000090537;
    vec3 b = v * (0.983729 * v + 0.4329510) + 0.238081;
    return a / b;
}

vec3 aces_fitted(vec3 linear_srgb) {
    vec3 v = linear_srgb * kAcesInput;
    v = rrt_and_odt_fit(v);
    v = v * kAcesOutput;
    return clamp(v, 0.0, 1.0);
}

void main() {
    vec3 hdr = texelFetch(u_hdr, ivec2(gl_FragCoord.xy), 0).rgb;
    vec3 exposed = max(hdr, vec3(0.0)) * u_exposure;
    o_color = vec4(aces_fitted(exposed), 1.0);
}
