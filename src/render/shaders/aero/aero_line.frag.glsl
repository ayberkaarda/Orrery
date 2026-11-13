#version 460 core
// Line coverage: a one-pixel linear ramp at the edges (anti-aliasing
// independent of MSAA) times the interpolated opacity. log_depth.glsl is
// composed in before this file. Blending: SRC_ALPHA, ONE_MINUS_SRC_ALPHA.

noperspective in float v_offset_px;
in float v_alpha;
in float v_view_w;
in vec3 v_color;

uniform float u_half_width; // [px]

layout(location = 0) out vec4 o_color;

void main() {
    float coverage = clamp(u_half_width + 0.5 - abs(v_offset_px), 0.0, 1.0);
    float alpha = coverage * clamp(v_alpha, 0.0, 1.0);
    if (alpha <= 0.0) {
        discard;
    }
    o_color = vec4(v_color, alpha);
    gl_FragDepth = orrery_log_depth(v_view_w);
}
