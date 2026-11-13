#version 460 core
// Marker disc: the marker colour inside a dark ring of 1.5 px, anti-aliased
// over one pixel (log_depth.glsl is composed in before this file). The depth
// is pulled towards the camera like the lines', so a marker on the ribbon
// stays in front of it while the planet still hides markers behind it.

in vec2 v_local_px;
in float v_radius;
in vec3 v_color;
in float v_view_w;

uniform vec3 u_ring;       // scene-linear ring colour
uniform float u_depth_bias;

layout(location = 0) out vec4 o_color;

void main() {
    float r = length(v_local_px);
    float coverage = clamp(v_radius + 0.5 - r, 0.0, 1.0);
    if (coverage <= 0.0) {
        discard;
    }
    float inner = clamp(v_radius - 1.5 + 0.5 - r, 0.0, 1.0);
    o_color = vec4(mix(u_ring, v_color, inner), coverage);
    gl_FragDepth = orrery_log_depth(v_view_w) - u_depth_bias;
}
