#version 460 core
// Curtain colour: a uniform scene-linear tint with the interpolated opacity
// (log_depth.glsl is composed in before this file). Blending: SRC_ALPHA,
// ONE_MINUS_SRC_ALPHA, no depth writes.

in float v_alpha;
in float v_view_w;

uniform vec3 u_color;
uniform float u_opacity;

layout(location = 0) out vec4 o_color;

void main() {
    float alpha = clamp(v_alpha, 0.0, 1.0) * u_opacity;
    if (alpha <= 0.001) {
        discard;
    }
    o_color = vec4(u_color, alpha);
    gl_FragDepth = orrery_log_depth(v_view_w);
}
