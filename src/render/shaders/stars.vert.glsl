#version 460 core
// Starfield: one point sprite per star, drawn at infinity. Directions are
// rotated by the view only (the camera position never matters), and the
// depth is fixed because the starfield is drawn first without depth testing.

layout(location = 0) in vec4 a_direction_sigma; // unit direction (world axes), sigma [px]
layout(location = 1) in vec4 a_color_peak;      // linear colour (unit luminance), peak radiance

uniform mat4 u_view_projection;
uniform float u_brightness; // user brightness factor
uniform float u_pixel_scale; // sprite size factor for the viewport height

out vec3 v_radiance;
out float v_sigma;

void main() {
    vec4 clip = u_view_projection * vec4(a_direction_sigma.xyz, 0.0);
    gl_Position = vec4(clip.xy, 0.0, clip.w);
    float sigma = a_direction_sigma.w * u_pixel_scale;
    v_sigma = sigma;
    gl_PointSize = max(ceil(sigma * 6.0) + 1.0, 2.0);
    v_radiance = a_color_peak.rgb * a_color_peak.a * u_brightness;
}
