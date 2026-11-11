#version 460 core
// Glow sprites, added to the HDR scene (blend ONE, ONE). log_depth.glsl is
// composed in before this file.

in vec2 v_offset_px;
in float v_view_w;
flat in vec4 v_color_disc;
flat in float v_kind;
flat in float v_extent_px;

layout(location = 0) out vec4 o_color;

void main() {
    float r = length(v_offset_px);
    float disc = max(v_color_disc.w, 0.5);
    float window = 1.0 - smoothstep(0.55, 1.0, r / v_extent_px);
    vec3 color;
    if (v_kind < 0.5) {
        // APPROXIMATION: star glare - an analytic halo (Moffat-like core plus a
        // wide faint skirt) standing in for the optical glare of a bright
        // source until the bloom pass exists - display only.
        float x = r / (1.6 * disc);
        float core = 1.0 / pow(1.0 + x * x, 2.0);
        float skirt = 0.06 / (1.0 + 0.04 * x * x);
        color = v_color_disc.rgb * (0.55 * core + skirt) * window;
    } else {
        // Faint ring just outside the black disc (placeholder for the photon
        // ring of the lensed image).
        float d = (r - 1.12 * disc) / (0.08 * disc + 0.6);
        color = v_color_disc.rgb * exp(-0.5 * d * d) * window;
    }
    o_color = vec4(color, 1.0);
    gl_FragDepth = orrery_log_depth(v_view_w);
}
