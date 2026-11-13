#version 460 core
// Aero surface shading in scene-linear radiance with logarithmic depth
// (log_depth.glsl is composed in before this file).
//
// The base colour comes from the colour map (u_colormap: 256 entries already
// pre-compensated for the display transform, see
// orrery::render::scene_linear_for_display) or is the neutral colour; the
// lighting factor is 1 where the surface faces the key light, so there the
// displayed colour equals the legend colour.
//
// APPROXIMATION: studio shading - a camera-attached key light with a wrapped
// Lambert term (floor u_ambient), a weak Blinn-Phong highlight with Schlick
// Fresnel (F0 = 0.04) and a faint cool rim - reads the shape without hiding
// the colour map; no shadows or global illumination.

in vec3 v_render_position;
in vec3 v_normal;
in float v_value;
in float v_weight;
in float v_view_w;

layout(binding = 0) uniform sampler2D u_colormap;

uniform vec2 u_range;          // value at t = 0 and t = 1
uniform vec3 u_neutral;        // scene-linear colour without a value
uniform vec3 u_key_direction;  // towards the key light, render axes
uniform float u_ambient;       // lighting factor facing away from the key
uniform float u_specular;      // highlight strength
uniform float u_rim;           // rim strength
uniform float u_opacity;
uniform vec2 u_contours;       // contour origin and step (step 0: none)
uniform float u_contour_strength;

layout(location = 0) out vec4 o_color;

void main() {
    vec3 n = normalize(v_normal);
    if (!gl_FrontFacing) {
        n = -n;
    }
    vec3 v = normalize(-v_render_position);
    vec3 base = u_neutral;
    bool mapped = v_weight > 0.5;
    if (mapped) {
        float span = u_range.y - u_range.x;
        float t = span > 0.0 ? clamp((v_value - u_range.x) / span, 0.0, 1.0) : 0.5;
        base = texture(u_colormap, vec2((t * 255.0 + 0.5) / 256.0, 0.5)).rgb;
    }
    float wrap = clamp((dot(n, u_key_direction) + 0.3) / 1.3, 0.0, 1.0);
    float light = mix(u_ambient, 1.0, wrap * wrap * (3.0 - 2.0 * wrap));
    vec3 h = normalize(u_key_direction + v);
    float n_dot_h = max(dot(n, h), 0.0);
    float fresnel = 0.04 + 0.96 * pow(1.0 - max(dot(v, h), 0.0), 5.0);
    float highlight = u_specular * fresnel * pow(n_dot_h, 72.0) * 9.0;
    float rim = u_rim * pow(1.0 - max(dot(n, v), 0.0), 3.0);
    vec3 color = base * light + vec3(highlight) + rim * vec3(0.55, 0.65, 0.85);

    if (mapped && u_contours.y > 0.0) {
        // Iso-lines of the mapped value at every legend step, one pixel wide.
        float f = (v_value - u_contours.x) / u_contours.y;
        float distance_px = abs(fract(f + 0.5) - 0.5) / max(fwidth(f), 1.0e-6);
        float line = 1.0 - clamp(distance_px - 0.35, 0.0, 1.0);
        color *= 1.0 - u_contour_strength * line;
    }

    o_color = vec4(color, u_opacity);
    gl_FragDepth = orrery_log_depth(v_view_w);
}
