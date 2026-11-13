#version 460 core
// Markers as screen-space discs: one instance per marker, six vertices per
// instance. Centres are relative to the track anchor [m]; the radius is in
// pixels. Markers behind the near plane are hidden.

layout(location = 0) in vec4 i_center; // relative to the anchor, radius [px]
layout(location = 1) in vec3 i_color;  // scene-linear

uniform mat4 u_view_projection;
uniform vec3 u_offset;
uniform vec2 u_viewport; // [px]
uniform float u_near;

out vec2 v_local_px; // offset from the centre [px]
out float v_radius;
out vec3 v_color;
out float v_view_w;

void main() {
    vec4 c = u_view_projection * vec4(i_center.xyz + u_offset, 1.0);
    if (c.w < u_near) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        v_local_px = vec2(0.0);
        v_radius = 0.0;
        v_color = vec3(0.0);
        v_view_w = 1.0;
        return;
    }
    const vec2 corners[6] = vec2[6](vec2(-1.0, -1.0), vec2(1.0, -1.0), vec2(1.0, 1.0),
                                    vec2(-1.0, -1.0), vec2(1.0, 1.0), vec2(-1.0, 1.0));
    float extent = i_center.w + 1.5;
    vec2 local = corners[gl_VertexID % 6] * extent;
    vec2 half_viewport = 0.5 * u_viewport;
    vec2 s = c.xy / c.w * half_viewport + local;
    gl_Position = vec4(s / half_viewport * c.w, c.z, c.w);
    v_local_px = local;
    v_radius = i_center.w;
    v_color = i_color;
    v_view_w = c.w;
}
