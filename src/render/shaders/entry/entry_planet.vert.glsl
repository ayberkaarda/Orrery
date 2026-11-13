#version 460 core
// Planet of the entry view: a unit icosphere in the planet-fixed frame.
// u_model scales it to the planet radius, turns it into the view frame and
// moves it to camera-relative render space (formed in double on the CPU).

layout(location = 0) in vec3 a_position; // unit sphere, planet-fixed

uniform mat4 u_model;           // planet-fixed unit sphere -> render space
uniform mat3 u_rotation;        // planet-fixed -> render axes
uniform mat4 u_view_projection; // render space -> clip space

out vec3 v_fixed;  // planet-fixed direction (graticule)
out vec3 v_normal; // render axes
out vec3 v_render_position;
out float v_view_w;

void main() {
    vec4 render_position = u_model * vec4(a_position, 1.0);
    v_fixed = a_position;
    v_normal = u_rotation * a_position;
    v_render_position = render_position.xyz;
    gl_Position = u_view_projection * render_position;
    v_view_w = gl_Position.w;
}
