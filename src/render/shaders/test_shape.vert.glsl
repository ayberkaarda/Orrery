#version 460 core
// Test shape vertex stage. Positions arrive in model space [m]; u_model maps
// them into camera-relative render space (floating origin already applied on
// the CPU in double precision), so every value here is small and float-safe.

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;

uniform mat4 u_model;           // model -> render space (rotation, uniform scale, translation)
uniform mat4 u_view_projection; // render space -> clip space

out vec3 v_render_position;
out vec3 v_normal;
out float v_view_w;

void main() {
    vec4 render_position = u_model * vec4(a_position, 1.0);
    v_render_position = render_position.xyz;
    v_normal = mat3(u_model) * a_normal;
    gl_Position = u_view_projection * render_position;
    v_view_w = gl_Position.w;
}
