#version 460 core
// Overlay quad in pixels (origin top-left) over the sRGB target, two
// triangles from gl_VertexID; the texture rows run bottom to top.

uniform vec4 u_rect;    // x, y, width, height [px]
uniform vec2 u_viewport; // [px]

out vec2 v_uv;

void main() {
    const vec2 corners[6] = vec2[6](vec2(0.0, 0.0), vec2(1.0, 0.0), vec2(1.0, 1.0),
                                    vec2(0.0, 0.0), vec2(1.0, 1.0), vec2(0.0, 1.0));
    vec2 st = corners[gl_VertexID % 6];
    vec2 pixel = u_rect.xy + st * u_rect.zw;
    v_uv = vec2(st.x, 1.0 - st.y);
    gl_Position = vec4(pixel.x / u_viewport.x * 2.0 - 1.0, 1.0 - pixel.y / u_viewport.y * 2.0,
                       0.0, 1.0);
}
