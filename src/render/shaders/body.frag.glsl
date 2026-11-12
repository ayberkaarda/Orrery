#version 460 core
// Body fragment stage: procedural surfaces, PBR-lite lighting and emissive
// stars in scene-linear radiance, with logarithmic depth. log_depth.glsl and
// noise.glsl are composed in before this file.

in vec3 v_normal;
in vec3 v_render_position;
in float v_view_w;
flat in vec4 v_albedo_style;
flat in vec4 v_secondary_specular;
flat in vec4 v_light_seed;
flat in vec4 v_irradiance; // irradiance, marker weight (1 = drawn far above its true size)
flat in vec3 v_emission;
flat in vec3 v_axis;

uniform float u_ambient; // ambient irradiance as a fraction of the light

layout(location = 0) out vec4 o_color;

const float kPi = 3.14159265358979;

// Surface styles (orrery::render::SurfaceStyle).
const int kUniform = 0;
const int kRocky = 1;
const int kCloudy = 2;
const int kTerrestrial = 3;
const int kDesert = 4;
const int kBanded = 5;
const int kIceGiant = 6;
const int kStar = 7;
const int kBlackHole = 8;
const int kProbe = 9;

struct Surface {
    vec3 albedo;
    float specular; // specular weight
    float gloss;    // Blinn-Phong exponent
};

// Body-fixed coordinates: z along the spin axis.
vec3 body_frame(vec3 n, vec3 axis) {
    vec3 helper = abs(axis.z) < 0.9 ? vec3(0.0, 0.0, 1.0) : vec3(1.0, 0.0, 0.0);
    vec3 x = normalize(cross(helper, axis));
    vec3 y = cross(axis, x);
    return vec3(dot(n, x), dot(n, y), dot(n, axis));
}

// APPROXIMATION: procedural surfaces - value-noise albedo maps (continents,
// belts, mottling) instead of measured maps or textures - suggest the look of
// each body class at a glance; no real geography.
Surface surface(int style, vec3 local, float seed) {
    vec3 albedo = v_albedo_style.rgb;
    vec3 secondary = v_secondary_specular.rgb;
    float specular = v_secondary_specular.a;
    vec3 p = local + vec3(seed * 37.0, seed * 11.0, seed * 23.0);
    float latitude = asin(clamp(local.z, -1.0, 1.0));
    Surface s = Surface(albedo, specular, 24.0);
    if (style == kRocky) {
        float large = orrery_fbm(p * 2.2, 5);
        float fine = orrery_fbm(p * 11.0, 3);
        s.albedo = mix(secondary, albedo, smoothstep(0.30, 0.70, large)) * (0.82 + 0.36 * fine);
    } else if (style == kCloudy) {
        float swirl = orrery_fbm(vec3(local.xy * 2.0, latitude * 5.0) + p, 5);
        s.albedo = mix(albedo, secondary, 0.45 * smoothstep(0.35, 0.75, swirl));
        s.gloss = 8.0;
    } else if (style == kTerrestrial) {
        vec3 warp = vec3(orrery_fbm(p * 1.3 + 3.7, 3), orrery_fbm(p * 1.3 - 5.1, 3),
                         orrery_fbm(p * 1.3 + 9.4, 3));
        float continents = orrery_fbm(p * 1.6 + 1.2 * warp, 6);
        float land = smoothstep(0.59, 0.605, continents);
        float relief = orrery_fbm(p * 8.0, 4);
        // Vegetation near the equator, desert in the subtropics, tundra poleward.
        float latitude_abs = abs(local.z);
        vec3 green = secondary;
        vec3 arid = vec3(0.30, 0.24, 0.15);
        vec3 tundra = vec3(0.22, 0.21, 0.17);
        float dry = smoothstep(0.15, 0.40, latitude_abs) * (1.0 - smoothstep(0.45, 0.62, latitude_abs));
        vec3 land_color = mix(green, arid, clamp(dry * (0.2 + 0.9 * relief), 0.0, 0.8));
        land_color = mix(land_color, tundra, smoothstep(0.62, 0.80, latitude_abs));
        land_color *= 0.85 + 0.3 * relief;
        vec3 ocean = albedo * (0.85 + 0.35 * smoothstep(0.565, 0.59, continents)); // shallows
        vec3 ground = mix(ocean, land_color, land);
        float ice = smoothstep(0.90, 0.93, latitude_abs + 0.05 * relief);
        ground = mix(ground, vec3(0.72, 0.74, 0.77), ice);
        vec3 q = p * 3.6 + 0.9 * warp + vec3(5.2, 1.3, -7.7);
        float cloud_field = orrery_fbm(q, 6);
        float clouds = smoothstep(0.50, 0.74, cloud_field) * (0.45 + 0.55 * orrery_fbm(q * 3.0, 3));
        // Clearer subtropics, cloudier mid-latitudes.
        clouds *= 0.55 + 0.45 * smoothstep(0.30, 0.65, latitude_abs);
        s.albedo = mix(ground, vec3(0.80, 0.80, 0.82), clouds);
        s.specular = specular * (1.0 - land) * (1.0 - ice) * (1.0 - clouds);
        s.gloss = 90.0;
    } else if (style == kDesert) {
        float features = orrery_fbm(p * 2.4, 6);
        vec3 ground = mix(albedo, secondary, smoothstep(0.48, 0.66, features));
        ground *= 0.88 + 0.24 * orrery_fbm(p * 12.0, 3);
        float cap = smoothstep(0.955, 0.975, abs(local.z));
        s.albedo = mix(ground, vec3(0.78, 0.76, 0.74), cap);
    } else if (style == kBanded) {
        float turbulence = orrery_fbm(vec3(local.xy * 3.0, latitude * 10.0) + p, 5);
        float band = sin(latitude * 15.0 + 2.2 * turbulence);
        float belts = smoothstep(-0.2, 0.7, band);
        vec3 base = mix(albedo, secondary, belts);
        base *= 0.9 + 0.2 * orrery_fbm(vec3(local.xy * 9.0, latitude * 40.0) + p, 3);
        float polar = smoothstep(0.75, 1.0, abs(local.z));
        s.albedo = mix(base, secondary * 0.85, polar * 0.6);
        s.gloss = 10.0;
    } else if (style == kIceGiant) {
        float band = sin(latitude * 9.0 + 1.5 * orrery_fbm(vec3(local.xy, latitude * 6.0) + p, 4));
        s.albedo = mix(albedo, secondary, 0.25 + 0.25 * band);
        s.gloss = 12.0;
    }
    return s;
}

void main() {
    int style = int(v_albedo_style.a + 0.5);
    vec3 n = normalize(v_normal);
    vec3 v = normalize(-v_render_position); // the camera is the render-space origin
    float mu = max(dot(n, v), 0.0);
    vec3 color;

    if (style == kStar) {
        // APPROXIMATION: photosphere - linear limb darkening I(mu) = 1 - 0.6 (1 - mu)
        // (solar value in the visible) and faint value-noise granulation -
        // looks right for Sun-like stars at any distance.
        vec3 local = body_frame(n, normalize(v_axis));
        float granulation = orrery_fbm(local * 24.0 + v_light_seed.w * 31.0, 4);
        color = v_emission * (1.0 - 0.6 * (1.0 - mu)) * (0.94 + 0.12 * granulation);
    } else if (style == kBlackHole) {
        // APPROXIMATION: black hole - an unlensed black disc of the
        // Schwarzschild shadow radius (sqrt(27)/2 r_s) plus a faint ring sprite,
        // placeholder until the lensing renderer exists - position and apparent
        // size of the shadow only; no lensing of the background.
        color = vec3(0.0);
    } else if (style == kProbe) {
        color = v_emission * (0.75 + 0.25 * mu);
    } else {
        vec3 local = body_frame(n, normalize(v_axis));
        Surface s = surface(style, local, v_light_seed.w);
        vec3 l = v_light_seed.xyz;
        float n_dot_l = max(dot(n, l), 0.0);
        // APPROXIMATION: PBR-lite - Lambert diffuse plus an energy-normalised
        // Blinn-Phong lobe weighted by a Schlick Fresnel term (F0 = 0.04) and a
        // per-material specular weight, one directional light per body from its
        // brightest star and a small constant ambient - plausible shading for
        // bodies seen from space; no atmosphere, no shadows, no eclipses.
        vec3 h = normalize(l + v);
        float n_dot_h = max(dot(n, h), 0.0);
        float v_dot_h = max(dot(v, h), 0.0);
        float fresnel = 0.04 + 0.96 * pow(1.0 - v_dot_h, 5.0);
        float lobe = (s.gloss + 8.0) / (8.0 * kPi) * pow(n_dot_h, s.gloss);
        vec3 diffuse = s.albedo / kPi;
        vec3 specular = vec3(s.specular * fresnel * lobe);
        vec3 irradiance = v_irradiance.rgb;
        color = (diffuse + specular) * irradiance * n_dot_l;
        color += diffuse * irradiance * u_ambient;
        // A body enlarged by the minimum screen size is a marker, not a
        // resolved disc: keep its night side readable so that it never shows
        // as a black hole against the sky.
        color = max(color, diffuse * irradiance * 0.3 * v_irradiance.a);
    }

    o_color = vec4(color, 1.0);
    gl_FragDepth = orrery_log_depth(v_view_w);
}
