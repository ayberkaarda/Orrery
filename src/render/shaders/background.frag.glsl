#version 460 core
// Test scene backdrop: a vertical scene-linear gradient written straight into
// the HDR target (no depth). Used with fullscreen.vert.glsl.

in vec2 v_uv;

uniform vec3 u_horizon; // scene-linear radiance at the bottom edge
uniform vec3 u_zenith;  // scene-linear radiance at the top edge

layout(location = 0) out vec4 o_color;

void main() {
    float t = smoothstep(0.0, 1.0, v_uv.y);
    o_color = vec4(mix(u_horizon, u_zenith, t), 1.0);
}
