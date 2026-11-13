#version 460 core
// A flat quad in render space, corner + s u_edge_u + t u_edge_v for s, t in
// [0, 1] (two triangles from gl_VertexID, no vertex buffer): section planes
// and the floor grid. v_uv maps the corner to u_uv_min and the far corner to
// u_uv_max (texel centres of the first and last grid node for sections).

uniform vec3 u_corner; // render space [m]
uniform vec3 u_edge_u;
uniform vec3 u_edge_v;
uniform vec2 u_uv_min;
uniform vec2 u_uv_max;
uniform mat4 u_view_projection;

out vec2 v_uv;
out vec2 v_local; // [0, 1]^2 across the quad
out vec3 v_render_position;
out float v_view_w;

void main() {
    const vec2 corners[6] = vec2[6](vec2(0.0, 0.0), vec2(1.0, 0.0), vec2(1.0, 1.0),
                                    vec2(0.0, 0.0), vec2(1.0, 1.0), vec2(0.0, 1.0));
    vec2 st = corners[gl_VertexID % 6];
    vec3 p = u_corner + st.x * u_edge_u + st.y * u_edge_v;
    v_uv = mix(u_uv_min, u_uv_max, st);
    v_local = st;
    v_render_position = p;
    gl_Position = u_view_projection * vec4(p, 1.0);
    v_view_w = gl_Position.w;
}
