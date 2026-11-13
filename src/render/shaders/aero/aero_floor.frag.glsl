#version 460 core
// Floor grid under a 3D aero scene: thin anti-aliased lines every u_step
// metres (stronger every fifth), fading out towards the edge of the quad and
// with grazing view angles. Blended over the backdrop; log depth.

in vec2 v_uv; // world coordinates in the floor plane [m]
in vec2 v_local;
in vec3 v_render_position;
in float v_view_w;

uniform float u_step;
uniform vec3 u_color; // scene-linear line colour
uniform float u_opacity;

layout(location = 0) out vec4 o_color;

float grid_line(vec2 p) {
    vec2 w = max(fwidth(p), vec2(1.0e-6));
    vec2 d = abs(fract(p + 0.5) - 0.5) / w;
    return 1.0 - clamp(min(d.x, d.y) - 0.25, 0.0, 1.0);
}

void main() {
    vec2 p = v_uv / u_step;
    float minor = grid_line(p);
    float major = grid_line(p / 5.0);
    float r = length(v_local - 0.5) * 2.0;
    float fade = 1.0 - smoothstep(0.15, 0.95, r);
    vec3 view = normalize(v_render_position);
    float grazing = smoothstep(0.02, 0.25, abs(view.z));
    float alpha = max(0.35 * minor, major) * fade * fade * grazing * u_opacity;
    if (alpha <= 0.002) {
        discard;
    }
    o_color = vec4(u_color, alpha);
    gl_FragDepth = orrery_log_depth(v_view_w);
}
