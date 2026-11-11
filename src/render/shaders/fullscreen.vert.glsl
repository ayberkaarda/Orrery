#version 460 core
// Fullscreen triangle without vertex buffers: gl_VertexID 0, 1 and 2 map to
// clip-space (-1, -1), (3, -1) and (-1, 3), which cover the whole viewport.

out vec2 v_uv;

void main() {
    vec2 corner = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));
    v_uv = corner;
    gl_Position = vec4(corner * 2.0 - 1.0, 0.0, 1.0);
}
