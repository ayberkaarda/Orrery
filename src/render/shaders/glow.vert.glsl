#version 460 core
// Screen-space glow sprites: a halo around stars and a faint ring around the
// black-hole placeholder. One instanced quad per sprite, sized in pixels
// around the projected centre; the centre's log depth lets bodies in front
// hide the sprite. Six vertices per instance, no vertex buffer for corners.

layout(location = 0) in vec4 i_centre_kind;  // render-space centre [m], kind (0 star, 1 black hole)
layout(location = 1) in vec4 i_color_disc;   // scene-linear colour, disc radius [px]

uniform mat4 u_view_projection;
uniform vec2 u_viewport; // [px]
uniform float u_near;    // smallest clip w that is drawn [m]

out vec2 v_offset_px;
out float v_view_w;
flat out vec4 v_color_disc;
flat out float v_kind;
flat out float v_extent_px;

const vec2 kCorners[6] = vec2[6](vec2(-1.0, -1.0), vec2(1.0, -1.0), vec2(1.0, 1.0),
                                 vec2(-1.0, -1.0), vec2(1.0, 1.0), vec2(-1.0, 1.0));

void main() {
    vec4 clip = u_view_projection * vec4(i_centre_kind.xyz, 1.0);
    v_color_disc = i_color_disc;
    v_kind = i_centre_kind.w;
    float disc = i_color_disc.w;
    float extent = v_kind < 0.5 ? clamp(disc * 10.0, 28.0, 320.0) : disc * 1.6 + 4.0;
    v_extent_px = extent;
    if (clip.w < u_near) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0); // behind the eye: clipped away
        v_offset_px = vec2(0.0);
        v_view_w = 1.0;
        return;
    }
    vec2 corner = kCorners[gl_VertexID % 6];
    v_offset_px = corner * extent;
    vec2 ndc = clip.xy / clip.w + corner * extent * 2.0 / u_viewport;
    gl_Position = vec4(ndc * clip.w, 0.0, clip.w);
    v_view_w = clip.w;
}
