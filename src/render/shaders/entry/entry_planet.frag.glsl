#version 460 core
// Planet shading of the entry view in scene-linear radiance with logarithmic
// depth (log_depth.glsl is composed in before this file).
//
// APPROXIMATION: a plain albedo with a soft terminator (wrapped Lambert from
// the sun direction), a weak limb darkening and a coastline-free graticule
// every u_grid_step degrees; no terrain, clouds or night lights. Grid lines
// are one pixel wide (fwidth) and fade where they crowd (near the poles and
// at grazing angles).

in vec3 v_fixed;
in vec3 v_normal;
in vec3 v_render_position;
in float v_view_w;

uniform vec3 u_sun;        // towards the sun, render axes (normalised)
uniform vec3 u_day;        // albedo x sun at full illumination
uniform vec3 u_night;      // radiance on the night side
uniform vec3 u_grid_color; // graticule on the day side
uniform float u_grid_step; // [deg]; 0 hides the graticule
uniform float u_grid_strength;

layout(location = 0) out vec4 o_color;

float grid_line(vec2 g) {
    vec2 w = max(fwidth(g), vec2(1.0e-5));
    vec2 d = abs(fract(g + 0.5) - 0.5) / w;
    float line = 1.0 - clamp(min(d.x, d.y) - 0.35, 0.0, 1.0);
    // Fade where neighbouring lines come closer than a few pixels.
    float crowd = clamp(1.0 - 6.0 * max(w.x, w.y), 0.0, 1.0);
    return line * crowd;
}

void main() {
    vec3 n = normalize(v_normal);
    float ndl = dot(n, u_sun);
    float day = smoothstep(-0.12, 0.30, ndl);
    vec3 lit = u_day * (0.30 + 0.70 * clamp(ndl, 0.0, 1.0));
    vec3 color = mix(u_night, lit, day);

    if (u_grid_step > 0.0) {
        vec3 f = normalize(v_fixed);
        float latitude = degrees(asin(clamp(f.z, -1.0, 1.0)));
        float longitude = degrees(atan(f.y, f.x));
        float line = grid_line(vec2(longitude, latitude) / u_grid_step);
        vec3 grid = u_grid_color * mix(0.28, 1.0, day);
        color = mix(color, grid, line * u_grid_strength);
    }

    vec3 v = normalize(-v_render_position);
    color *= mix(0.72, 1.0, clamp(dot(n, v) * 2.5, 0.0, 1.0));
    o_color = vec4(color, 1.0);
    gl_FragDepth = orrery_log_depth(v_view_w);
}
