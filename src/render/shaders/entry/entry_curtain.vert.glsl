#version 460 core
// Curtain between the ground track and the trajectory: a triangle strip of
// ground and track vertices (positions relative to the track anchor [m],
// opacity in w: 0 on the ground, u_opacity at the trajectory).

layout(location = 0) in vec4 a_position; // relative to the anchor, opacity

uniform mat4 u_view_projection;
uniform vec3 u_offset; // anchor in camera-relative render space

out float v_alpha;
out float v_view_w;

void main() {
    gl_Position = u_view_projection * vec4(a_position.xyz + u_offset, 1.0);
    v_alpha = a_position.w;
    v_view_w = gl_Position.w;
}
