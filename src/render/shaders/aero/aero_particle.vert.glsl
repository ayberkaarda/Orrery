#version 460 core
// Particle traces drawn straight from the particle kernel's trace ring
// (vertex pulling, no copy to the host): instance p * (K - 1) + s is the
// segment from the s-th newest trace point of particle p to the next older
// one. Positions are grid index coordinates; w is the age at that point
// (< 0: not part of the current life, the segment is dropped). Opacity
// falls from the head to the tail and fades in and out over the life.
// Widened to screen-space quads like aero_line.vert.glsl.

layout(std430, binding = 0) readonly buffer Traces {
    vec4 traces[];
};

uniform mat4 u_view_projection;
uniform vec3 u_grid_origin; // render space [m]
uniform float u_spacing;    // [m]
uniform uint u_trace_points;
uniform uint u_head;
uniform vec2 u_viewport;
uniform float u_half_width;
uniform float u_near;
uniform float u_lifetime;
uniform vec3 u_color; // scene-linear
uniform float u_opacity;

noperspective out float v_offset_px;
out float v_alpha;
out float v_view_w;
out vec3 v_color;

void hide() {
    gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
    v_offset_px = 0.0;
    v_alpha = 0.0;
    v_view_w = 1.0;
    v_color = vec3(0.0);
}

float life_fade(float age) {
    return smoothstep(0.0, 0.08 * u_lifetime, age) *
           (1.0 - smoothstep(0.8 * u_lifetime, u_lifetime, age));
}

void main() {
    uint k = max(u_trace_points, 2u);
    uint segments = k - 1u;
    uint particle = uint(gl_InstanceID) / segments;
    uint s = uint(gl_InstanceID) % segments;
    uint newer = (u_head + k - s) % k;
    uint older = (u_head + k - s - 1u) % k;
    vec4 a = traces[particle * k + newer];
    vec4 b = traces[particle * k + older];
    if (a.w < 0.0 || b.w < 0.0) {
        hide();
        return;
    }
    vec4 c0 = u_view_projection * vec4(u_grid_origin + u_spacing * a.xyz, 1.0);
    vec4 c1 = u_view_projection * vec4(u_grid_origin + u_spacing * b.xyz, 1.0);
    if (c0.w < u_near || c1.w < u_near) {
        hide();
        return;
    }
    float tail0 = 1.0 - float(s) / float(segments);
    float tail1 = 1.0 - float(s + 1u) / float(segments);
    float a0 = u_opacity * tail0 * life_fade(a.w);
    float a1 = u_opacity * tail1 * life_fade(b.w);
    int corner = gl_VertexID % 6;
    float end = (corner == 1 || corner == 2 || corner == 4) ? 1.0 : 0.0;
    float side = (corner == 2 || corner == 4 || corner == 5) ? 1.0 : -1.0;
    vec2 half_viewport = 0.5 * u_viewport;
    vec2 s0 = c0.xy / c0.w * half_viewport;
    vec2 s1 = c1.xy / c1.w * half_viewport;
    vec2 d = s1 - s0;
    float len = length(d);
    if (!(len >= 1.0e-2)) {
        hide();
        return;
    }
    vec2 dir = d / len;
    vec2 normal = vec2(-dir.y, dir.x);
    float extent = u_half_width + 1.0;
    vec4 c = end > 0.5 ? c1 : c0;
    vec2 p = (end > 0.5 ? s1 : s0) + normal * side * extent + dir * (end * 2.0 - 1.0) * 0.5;
    gl_Position = vec4(p / half_viewport * c.w, c.z, c.w);
    v_offset_px = side * extent;
    v_alpha = end > 0.5 ? a1 : a0;
    v_view_w = c.w;
    v_color = u_color;
}
