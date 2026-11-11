#version 460 core
// Body vertex stage: one unit icosphere per LOD, instanced once per body.
// The instance carries the body centre already made camera-relative on the
// CPU in double precision (floating origin), so every value is float-safe.

layout(location = 0) in vec3 a_position; // unit sphere, equal to the outward normal

layout(location = 1) in vec4 i_centre_radius;      // render-space centre [m], visual radius [m]
layout(location = 2) in vec4 i_albedo_style;       // linear albedo, surface style
layout(location = 3) in vec4 i_secondary_specular; // secondary colour, specular weight
layout(location = 4) in vec4 i_light_seed;         // unit vector to the light, noise seed
layout(location = 5) in vec4 i_irradiance;         // light irradiance (scene-linear), marker weight
layout(location = 6) in vec4 i_emission;           // emitted radiance, unused
layout(location = 7) in vec4 i_axis;               // unit spin axis (world axes), unused

uniform mat4 u_view_projection; // render space -> clip space

out vec3 v_normal;
out vec3 v_render_position;
out float v_view_w;
flat out vec4 v_albedo_style;
flat out vec4 v_secondary_specular;
flat out vec4 v_light_seed;
flat out vec4 v_irradiance;
flat out vec3 v_emission;
flat out vec3 v_axis;

void main() {
    vec3 render_position = i_centre_radius.xyz + i_centre_radius.w * a_position;
    v_normal = a_position;
    v_render_position = render_position;
    gl_Position = u_view_projection * vec4(render_position, 1.0);
    v_view_w = gl_Position.w;
    v_albedo_style = i_albedo_style;
    v_secondary_specular = i_secondary_specular;
    v_light_seed = i_light_seed;
    v_irradiance = i_irradiance;
    v_emission = i_emission.rgb;
    v_axis = i_axis.xyz;
}
