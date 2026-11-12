#version 460 core
// Orbit trail segments as anti-aliased screen-space quads. One instance per
// segment (endpoints already camera-relative in float, see
// orrery::render::append_trail_segments), six vertices per instance. The
// segment is clipped against the near plane in clip space before the
// perspective divide, then widened perpendicular to its screen direction.
// Segments without a screen direction (shorter than kMinScreenLength) are
// dropped; append_trail_segments already omits repeated world points.

layout(location = 0) in vec4 i_p0_fade0; // render-space start [m], fade at the start
layout(location = 1) in vec4 i_p1_fade1; // render-space end [m], fade at the end
layout(location = 2) in vec4 i_color;    // scene-linear colour, opacity

uniform mat4 u_view_projection;
uniform vec2 u_viewport;     // [px]
uniform float u_half_width;  // half line width [px]
uniform float u_near;        // smallest clip w that is drawn [m]

// Segments shorter than this on screen are dropped [px].
const float kMinScreenLength = 1.0e-2;

noperspective out float v_offset_px; // signed distance from the centre line [px]
out float v_fade;
out float v_view_w;
flat out vec4 v_color;

void main() {
    v_color = i_color;
    vec4 c0 = u_view_projection * vec4(i_p0_fade0.xyz, 1.0);
    vec4 c1 = u_view_projection * vec4(i_p1_fade1.xyz, 1.0);
    float f0 = i_p0_fade0.w;
    float f1 = i_p1_fade1.w;
    if (c0.w < u_near && c1.w < u_near) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0); // entirely behind the eye
        v_offset_px = 0.0;
        v_fade = 0.0;
        v_view_w = 1.0;
        return;
    }
    if (c0.w < u_near) {
        float t = (u_near - c0.w) / (c1.w - c0.w);
        c0 = mix(c0, c1, t);
        f0 = mix(f0, f1, t);
    } else if (c1.w < u_near) {
        float t = (u_near - c1.w) / (c0.w - c1.w);
        c1 = mix(c1, c0, t);
        f1 = mix(f1, f0, t);
    }

    int corner = gl_VertexID % 6;
    // (end, side) per corner: two triangles covering the quad.
    float end = (corner == 1 || corner == 2 || corner == 4) ? 1.0 : 0.0;
    float side = (corner == 2 || corner == 4 || corner == 5) ? 1.0 : -1.0;

    vec2 half_viewport = 0.5 * u_viewport;
    vec2 s0 = c0.xy / c0.w * half_viewport;
    vec2 s1 = c1.xy / c1.w * half_viewport;
    vec2 d = s1 - s0;
    float len = length(d);
    if (!(len >= kMinScreenLength)) {
        // Screen-degenerate (along the view axis, or equal endpoints after
        // clipping): no direction to widen along, and a joint needs no cap
        // because the neighbouring segments overlap there.
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        v_offset_px = 0.0;
        v_fade = 0.0;
        v_view_w = 1.0;
        return;
    }
    vec2 dir = d / len;
    vec2 normal = vec2(-dir.y, dir.x);
    float extent = u_half_width + 1.0; // one pixel of feather for the coverage ramp

    vec4 c = end > 0.5 ? c1 : c0;
    vec2 s = (end > 0.5 ? s1 : s0) + normal * side * extent + dir * (end * 2.0 - 1.0) * 0.5;
    gl_Position = vec4(s / half_viewport * c.w, c.z, c.w);
    v_offset_px = side * extent;
    v_fade = end > 0.5 ? f1 : f0;
    v_view_w = c.w;
}
