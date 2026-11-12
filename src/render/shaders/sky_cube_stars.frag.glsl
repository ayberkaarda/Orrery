#version 460 core
// Star in one face of the environment cube map (with
// sky_cube_stars.vert.glsl): a Gaussian in the angle between the texel's
// direction and the star, added to the face (blend ONE, ONE). The texel
// direction comes from the fragment position, so the profile is the same
// on every face and across edges and corners.

flat in vec3 v_radiance;
flat in vec3 v_direction;
flat in float v_sigma; // [rad]

uniform mat3 u_face_to_world; // face NDC (x, y, -1) -> world direction
uniform float u_texel_angle;  // 2 / face size

layout(location = 0) out vec4 o_color;

void main() {
    vec2 ndc = gl_FragCoord.xy * u_texel_angle - 1.0;
    vec3 d = normalize(u_face_to_world * vec3(ndc, -1.0));
    float angle = 2.0 * asin(min(0.5 * length(d - v_direction), 1.0));
    o_color = vec4(v_radiance * exp(-0.5 * angle * angle / (v_sigma * v_sigma)), 1.0);
}
