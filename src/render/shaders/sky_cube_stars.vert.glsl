#version 460 core
// Starfield drawn into one face of the environment cube map of the
// black-hole pass (used with sky_cube_stars.frag.glsl). Each star keeps the
// angular width and the integrated brightness it has on a reference screen
// (1080 rows over a 40 degree field of view, where one pixel subtends
// u_reference_pixel_angle), so the lensed sky looks like the unlensed one.
// Sprites narrower than 0.6 cube texels are widened, keeping their flux.
//
// A star is a Gaussian in the angle from its direction (the fragment shader
// evaluates it at each texel's own direction), so its profile and flux do
// not depend on where it falls on the cube. Near an edge or a corner its
// footprint spreads over two or three faces: every face draws the star when
// its centre lies within the face or within the guard band around it (the
// viewport reaches past the face, see BlackHolePass), and keeps the texels
// that are its own. Points are clipped by their centre, so without the
// guard band a neighbouring face would drop its share.

// APPROXIMATION: star brightness in the lensed sky - the angular brightness
// of the reference screen instead of the per-pixel brightness of the sprite
// sky - a cube map has no screen pixels - the two skies agree within about
// 10 percent at 1080 rows over 40 degrees (and at 1440 x 810); smaller images
// show the lensed stars dimmer than the unlensed sprites.
layout(location = 0) in vec4 a_direction_sigma; // unit direction (world axes), sigma [reference px]
layout(location = 1) in vec4 a_color_peak;      // linear colour (unit luminance), peak radiance

uniform mat4 u_view_projection;       // face camera: 90 degree field of view
uniform float u_guard_scale;          // N / (N + 2 guard): clip x, y scale of the widened viewport
uniform float u_brightness;           // user brightness factor
uniform float u_reference_pixel_angle; // [rad]
uniform float u_texel_angle;          // 2 / face size: texel angle at the face centre [rad]

flat out vec3 v_radiance;
flat out vec3 v_direction;
flat out float v_sigma; // [rad]

const float kMinSigmaTexels = 0.6;
const float kFootprintSigmas = 3.5; // the sprite reaches 3.5 sigma: 0.2 percent of the flux lies beyond

void main() {
    vec3 direction = normalize(a_direction_sigma.xyz);
    vec4 clip = u_view_projection * vec4(direction, 0.0);
    gl_Position = vec4(clip.xy * u_guard_scale, 0.0, clip.w);
    // Texel angle at the star, the same on every face that holds it:
    // (2 / N) m^(3/2) with m the largest absolute component of the direction.
    vec3 a = abs(direction);
    float m = max(a.x, max(a.y, a.z));
    float texel = u_texel_angle * m * sqrt(m);
    float sigma_angle = a_direction_sigma.w * u_reference_pixel_angle;
    float widened = max(sigma_angle, kMinSigmaTexels * texel);
    float flux_ratio = (sigma_angle / widened) * (sigma_angle / widened);
    v_radiance = a_color_peak.rgb * a_color_peak.a * flux_ratio * u_brightness;
    v_direction = direction;
    v_sigma = widened;
    // Sprite size on this face: the finest texel angle around the star, the
    // one towards the face centre, is (2 / N) / (1 + x^2 + y^2).
    vec2 face = clip.w > 0.0 ? clip.xy / clip.w : vec2(0.0);
    float finest = u_texel_angle / (1.0 + dot(face, face));
    gl_PointSize = 2.0 * ceil(kFootprintSigmas * widened / finest) + 2.0;
}
