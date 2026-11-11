#version 460 core
// Orbit trail coverage: a one-pixel linear ramp at the line edges gives
// anti-aliasing independent of MSAA; the opacity fades with sample age.
// log_depth.glsl is composed in before this file. Blending: SRC_ALPHA,
// ONE_MINUS_SRC_ALPHA into the HDR scene.

noperspective in float v_offset_px;
in float v_fade;
in float v_view_w;
flat in vec4 v_color;

uniform float u_half_width; // [px]

layout(location = 0) out vec4 o_color;

void main() {
    float coverage = clamp(u_half_width + 0.5 - abs(v_offset_px), 0.0, 1.0);
    float fade = clamp(v_fade, 0.0, 1.0);
    float alpha = coverage * fade * fade * v_color.a;
    if (alpha <= 0.0) {
        discard;
    }
    o_color = vec4(v_color.rgb, alpha);
    gl_FragDepth = orrery_log_depth(v_view_w);
}
