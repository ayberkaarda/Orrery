#version 460 core
// Starfield sprite: a Gaussian point spread function added to the HDR scene
// (blend ONE, ONE).

in vec3 v_radiance;
in float v_sigma;

layout(location = 0) out vec4 o_color;

void main() {
    vec2 offset = (gl_PointCoord - 0.5) * (max(ceil(v_sigma * 6.0) + 1.0, 2.0));
    float falloff = exp(-0.5 * dot(offset, offset) / (v_sigma * v_sigma));
    o_color = vec4(v_radiance * falloff, 1.0);
}
