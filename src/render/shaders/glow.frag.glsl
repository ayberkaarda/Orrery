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
    vec3 color;
    if (v_kind < 0.5) {
        // APPROXIMATION: star glare - an analytic point spread function (a
        // Moffat core plus power-law wings falling as r^-3, like the scatter
        // of a lens or the eye) standing in for the optical glare of a bright
        // source until the bloom pass exists - display only. The window only
        // removes the last, already faint, part of the wings at the sprite
        // edge, so the glare has no visible rim.
        float window = 1.0 - smoothstep(0.3, 1.0, r / v_extent_px);
        float x = r / (1.6 * disc);
        float core = 1.0 / ((1.0 + x * x) * (1.0 + x * x));
        float y = r / (3.0 * disc);
        float wings = 1.0 / pow(1.0 + y * y, 1.5);
        float z = r / (9.0 * disc);
        float veil = 1.0 / pow(1.0 + z * z, 1.5);
        color = v_color_disc.rgb * (0.55 * core + 0.09 * wings + 0.012 * veil) * window;
    } else {
        // Faint ring just outside the black disc (placeholder for the photon
        // ring of the lensed image).
        float window = 1.0 - smoothstep(0.55, 1.0, r / v_extent_px);
        float d = (r - 1.12 * disc) / (0.08 * disc + 0.6);
        color = v_color_disc.rgb * exp(-0.5 * d * d) * window;
    }
    o_color = vec4(color, 1.0);
    gl_FragDepth = orrery_log_depth(v_view_w);
}
