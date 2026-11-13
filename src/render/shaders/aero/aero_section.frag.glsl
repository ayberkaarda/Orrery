#version 460 core
// Section plane: the sampled field (u_values: r = value, g = solid flag,
// bilinear between grid nodes) through the colour map, which is already
// pre-compensated for the display transform, so the plane shows the legend
// colours exactly. Solid nodes take the body colour with an anti-aliased
// edge at the half-way level of the filtered flag. Unlit; log depth.

in vec2 v_uv;
in vec2 v_local;
in vec3 v_render_position;
in float v_view_w;

layout(binding = 0) uniform sampler2D u_colormap;
layout(binding = 1) uniform sampler2D u_values;

uniform vec2 u_range;
uniform vec3 u_solid_color; // scene-linear
uniform float u_opacity;
uniform vec2 u_contours; // origin and step (step 0: none)
uniform float u_contour_strength;

layout(location = 0) out vec4 o_color;

void main() {
    vec2 s = texture(u_values, v_uv).rg;
    float span = u_range.y - u_range.x;
    float t = span > 0.0 ? clamp((s.r - u_range.x) / span, 0.0, 1.0) : 0.5;
    vec3 color = texture(u_colormap, vec2((t * 255.0 + 0.5) / 256.0, 0.5)).rgb;
    if (u_contours.y > 0.0) {
        float f = (s.r - u_contours.x) / u_contours.y;
        float distance_px = abs(fract(f + 0.5) - 0.5) / max(fwidth(f), 1.0e-6);
        float line = 1.0 - clamp(distance_px - 0.35, 0.0, 1.0);
        color *= 1.0 - u_contour_strength * line;
    }
    float edge = max(fwidth(s.g), 1.0e-4);
    float solid = smoothstep(0.5 - edge, 0.5 + edge, s.g);
    color = mix(color, u_solid_color, solid);
    o_color = vec4(color, u_opacity);
    gl_FragDepth = orrery_log_depth(v_view_w);
}
