#version 460 core
// Streamline segments as anti-aliased screen-space quads (the technique of
// trail.vert.glsl): one instance per segment, six vertices per instance.
// Endpoints are world positions [m] (body axes) with the mapped value in w;
// u_offset is the world origin in camera-relative render space. Each end
// carries its own opacity. The segment is clipped against the near plane
// before the perspective divide and widened across its screen direction.

layout(location = 0) in vec4 i_p0; // world position, value
layout(location = 1) in vec4 i_p1;
layout(location = 2) in vec2 i_alpha; // opacity at the start and the end

layout(binding = 0) uniform sampler2D u_colormap;

uniform mat4 u_view_projection;
uniform vec3 u_offset;
uniform vec2 u_viewport;    // [px]
uniform float u_half_width; // [px]
uniform float u_near;
uniform vec2 u_range;
uniform int u_use_colormap;
uniform vec3 u_color; // scene-linear, when the colour map is off

noperspective out float v_offset_px;
out float v_alpha;
out float v_view_w;
out vec3 v_color;

vec3 mapped(float value) {
    if (u_use_colormap == 0) {
        return u_color;
    }
    float span = u_range.y - u_range.x;
    float t = span > 0.0 ? clamp((value - u_range.x) / span, 0.0, 1.0) : 0.5;
    return texture(u_colormap, vec2((t * 255.0 + 0.5) / 256.0, 0.5)).rgb;
}

void hide() {
    gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
    v_offset_px = 0.0;
    v_alpha = 0.0;
    v_view_w = 1.0;
    v_color = vec3(0.0);
}

void main() {
    vec4 c0 = u_view_projection * vec4(i_p0.xyz + u_offset, 1.0);
    vec4 c1 = u_view_projection * vec4(i_p1.xyz + u_offset, 1.0);
    float a0 = i_alpha.x;
    float a1 = i_alpha.y;
    float value0 = i_p0.w;
    float value1 = i_p1.w;
    if (c0.w < u_near && c1.w < u_near) {
        hide();
        return;
    }
    if (c0.w < u_near) {
        float t = (u_near - c0.w) / (c1.w - c0.w);
        c0 = mix(c0, c1, t);
        a0 = mix(a0, a1, t);
        value0 = mix(value0, value1, t);
    } else if (c1.w < u_near) {
        float t = (u_near - c1.w) / (c0.w - c1.w);
        c1 = mix(c1, c0, t);
        a1 = mix(a1, a0, t);
        value1 = mix(value1, value0, t);
    }
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
    vec2 s = (end > 0.5 ? s1 : s0) + normal * side * extent + dir * (end * 2.0 - 1.0) * 0.5;
    gl_Position = vec4(s / half_viewport * c.w, c.z, c.w);
    v_offset_px = side * extent;
    v_alpha = end > 0.5 ? a1 : a0;
    v_view_w = c.w;
    v_color = mapped(end > 0.5 ? value1 : value0);
}
