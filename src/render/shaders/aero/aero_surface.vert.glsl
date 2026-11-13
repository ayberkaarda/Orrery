#version 460 core
// Aero surfaces (body, iso-surface, force arrows): world positions [m] in
// body axes, small enough for float; u_model adds the floating-origin offset
// (world origin in camera-relative render space, formed in double on the
// CPU) and, for arrows, their placement.

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec2 a_value; // value, weight (0: neutral colour)

uniform mat4 u_model;           // world -> render space
uniform mat4 u_view_projection; // render space -> clip space

out vec3 v_render_position;
out vec3 v_normal;
out float v_value;
out float v_weight;
out float v_view_w;

void main() {
    vec4 render_position = u_model * vec4(a_position, 1.0);
    v_render_position = render_position.xyz;
    v_normal = mat3(u_model) * a_normal;
    v_value = a_value.x;
    v_weight = a_value.y;
    gl_Position = u_view_projection * render_position;
    v_view_w = gl_Position.w;
}
