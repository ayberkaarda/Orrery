#version 460 core
// Test shape fragment stage: scene-linear shading into the HDR target with
// logarithmic depth (log_depth.glsl is composed in before this file).
// Render space keeps the world axes (only the camera position is subtracted),
// so +Z is the world "up" used by the hemispheric ambient term.

in vec3 v_render_position;
in vec3 v_normal;
in float v_view_w;

uniform vec3 u_albedo;           // diffuse reflectance, linear
uniform vec3 u_key_direction;    // unit vector towards the key light
uniform vec3 u_key_irradiance;   // key light irradiance (scene-linear)
uniform vec3 u_rim_direction;    // unit vector towards the rim light
uniform vec3 u_rim_irradiance;   // rim light irradiance (scene-linear)
uniform vec3 u_sky_ambient;      // ambient radiance from +Z
uniform vec3 u_ground_ambient;   // ambient radiance from -Z
uniform float u_shininess;       // Blinn-Phong exponent

layout(location = 0) out vec4 o_color;

const float kPi = 3.14159265358979;

// APPROXIMATION: shading - energy-normalised Blinn-Phong with Schlick Fresnel
// (F0 = 0.04) and a two-colour hemispheric ambient instead of a physically
// based BRDF and image-based lighting - test shape only, it exists to exercise
// the HDR -> ACES -> sRGB path, not to model a material.
vec3 shade(vec3 n, vec3 v, vec3 l, vec3 irradiance) {
    float n_dot_l = max(dot(n, l), 0.0);
    vec3 h = normalize(l + v);
    float n_dot_h = max(dot(n, h), 0.0);
    float v_dot_h = max(dot(v, h), 0.0);
    float fresnel = 0.04 + 0.96 * pow(1.0 - v_dot_h, 5.0);
    float specular = (u_shininess + 8.0) / (8.0 * kPi) * pow(n_dot_h, u_shininess);
    vec3 diffuse = u_albedo / kPi * (1.0 - fresnel);
    return (diffuse + vec3(specular * fresnel)) * irradiance * n_dot_l;
}

void main() {
    vec3 n = normalize(v_normal);
    if (!gl_FrontFacing) {
        n = -n;
    }
    vec3 v = normalize(-v_render_position); // the camera sits at the render-space origin

    vec3 color = shade(n, v, u_key_direction, u_key_irradiance);
    color += shade(n, v, u_rim_direction, u_rim_irradiance);
    float up = 0.5 + 0.5 * n.z;
    color += u_albedo * mix(u_ground_ambient, u_sky_ambient, up);

    o_color = vec4(color, 1.0);
    gl_FragDepth = orrery_log_depth(v_view_w);
}
