#version 460 core
// Black-hole composite (docs/rendering.md, "Black hole"; ADR-017): brings
// the half-resolution geodesic trace to full resolution and shades it.
// Used with fullscreen.vert.glsl; log_depth.glsl is composed in before this
// file. Drawn additively into the HDR scene target with depth test ALWAYS,
// in place of the unlensed sky.
//
// Edge-aware upsampling: the four trace samples around the pixel are blended
// with bilinear weights, except across the critical curve b = 3 sqrt(3) M.
// Off the disk that curve is the only discontinuity of the ray map, and its
// position is known analytically for every pixel (camera outside the photon
// sphere: a ray is captured exactly when it moves inwards with b < b_c), so
// samples on the other side of it are left out and the shadow edge is
// anti-aliased at full resolution. The escape directions (not the colours)
// are interpolated and the sky is looked up once per pixel, so stars stay
// sharp; where neighbouring directions jump (close to the critical curve,
// where the sky is squeezed into thin rings) the nearest sample is used.

// APPROXIMATION: edge-aware upsampling and occlusion depth - escape
// directions are interpolated between the half-resolution samples (nearest
// sample where they jump) and the shadow and disk take the depth of the hole
// centre or of their first opaque hit, from the nearest sample - the trace
// runs at half resolution (ADR-017); depth is needed only to hide bodies
// behind the hole - exact at the critical curve (analytic) and wherever the
// ray map is smooth over two pixels; near the critical curve the sky is
// filtered rather than resolved; bodies within the disk's thickness of the
// hole can be ordered wrongly by one sample.

in vec2 v_uv;

layout(binding = 0) uniform sampler2D u_disk;        // trace: emission, sky transmittance
layout(binding = 1) uniform sampler2D u_sky;         // trace: escape direction, depth [M]
layout(binding = 2) uniform samplerCube u_environment;

uniform ivec2 u_trace_size;
uniform vec2 u_viewport;
uniform mat3 u_camera_basis;
uniform vec2 u_tan_half_fov;
uniform vec3 u_camera_position;   // [M]
uniform float u_observer_shift;   // 1 / sqrt(1 - 2 / r_camera)
uniform int u_analytic_shadow;    // 1 when the camera is outside the photon sphere
uniform float u_environment_texel; // 2 / face size: texel angle at a face centre [rad]
uniform float u_environment_max_lod;
uniform float u_environment_scale; // 0 hides the sky (starfield off)
uniform float u_mass_length;       // metres per M

layout(location = 0) out vec4 o_color;

const float kCriticalB = 5.196152422706632;
const float kJumpCos = 0.995; // directions more than about 5.7 degrees apart
// Radiance bound of the trace (black_hole_trace.comp.glsl, "radiance
// bound"), applied again to the written pixel so that disk and sky together
// stay far below the largest half float of the HDR target.
const float kMaxRadiance = 1.0e4;

vec3 ray_direction(vec2 pixel) {
    vec2 ndc = pixel / u_viewport * 2.0 - 1.0;
    return normalize(u_camera_basis * vec3(ndc * u_tan_half_fov, -1.0));
}

// Impact parameter [M] and direction sense of the ray through `pixel`.
vec2 impact(vec2 pixel) {
    vec3 d = ray_direction(pixel);
    float r0 = length(u_camera_position);
    vec3 e1 = u_camera_position / r0;
    float c = dot(d, e1);
    return vec2(r0 * length(d - c * e1) * u_observer_shift, c);
}

bool in_shadow(vec2 bc) {
    return bc.y < 0.0 && bc.x < kCriticalB;
}

void main() {
    vec2 pixel = gl_FragCoord.xy;
    vec2 bc = impact(pixel);
    bool analytic = u_analytic_shadow != 0;
    bool inside = analytic && in_shadow(bc);

    vec2 t = (pixel - 1.0) * 0.5;
    ivec2 base = ivec2(floor(t));
    vec2 f = t - vec2(base);

    vec3 emission = vec3(0.0);
    float transmittance = 0.0;
    float weight_sum = 0.0;
    vec3 directions[4];
    float sky_weights[4];
    vec3 direction_sum = vec3(0.0);
    float best_weight = -1.0;
    vec3 best_direction = vec3(0.0, 0.0, 1.0);
    float best_depth = 0.0;
    for (int k = 0; k < 4; ++k) {
        ivec2 offset = ivec2(k & 1, k >> 1);
        ivec2 q = clamp(base + offset, ivec2(0), u_trace_size - 1);
        float w = (offset.x == 1 ? f.x : 1.0 - f.x) * (offset.y == 1 ? f.y : 1.0 - f.y);
        w = max(w, 1.0e-4);
        if (analytic && in_shadow(impact(2.0 * vec2(q) + 1.0)) != inside) {
            w *= 1.0e-4; // other side of the critical curve
        }
        vec4 disk = texelFetch(u_disk, q, 0);
        vec4 sky = texelFetch(u_sky, q, 0);
        emission += w * disk.rgb;
        transmittance += w * disk.a;
        weight_sum += w;
        directions[k] = sky.xyz;
        sky_weights[k] = w * disk.a;
        direction_sum += sky_weights[k] * sky.xyz;
        if (w > best_weight) {
            best_weight = w;
            best_depth = sky.w;
            best_direction = sky.xyz;
        }
    }
    emission /= weight_sum;
    transmittance /= weight_sum;

    vec3 direction = length(direction_sum) > 0.0 ? normalize(direction_sum) : best_direction;
    for (int k = 0; k < 4; ++k) {
        if (sky_weights[k] > 0.0 && dot(directions[k], direction) < kJumpCos) {
            direction = normalize(best_direction);
            break;
        }
    }

    // Anti-aliased shadow edge from the analytic impact parameter.
    float coverage = 1.0;
    float b_width = max(fwidth(bc.x), 1.0e-6);
    if (analytic && bc.y < 0.0) {
        coverage = clamp((bc.x - kCriticalB) / b_width + 0.5, 0.0, 1.0);
    }

    // Mip level from the angular footprint of the lensed direction against
    // the angular size of a cube texel along it, (2 / N) m^(3/2) with m the
    // largest absolute component.
    vec3 footprint_x = dFdx(direction);
    vec3 footprint_y = dFdy(direction);
    float footprint = max(length(footprint_x), length(footprint_y));
    vec3 a = abs(direction);
    float m = max(a.x, max(a.y, a.z));
    float texel = u_environment_texel * m * sqrt(m);
    float lod = clamp(log2(max(footprint / texel, 1.0e-6)), 0.0, u_environment_max_lod);
    vec3 sky = textureLod(u_environment, direction, lod).rgb * u_environment_scale;

    vec3 radiance = emission + transmittance * coverage * sky;
    float peak = max(radiance.r, max(radiance.g, radiance.b));
    if (peak > kMaxRadiance) {
        radiance *= kMaxRadiance / peak;
    }
    o_color = vec4(radiance, 1.0);
    // Shadow and opaque disk hide what lies behind them (bodies, trails).
    gl_FragDepth = transmittance < 0.5 && best_depth > 0.0
                       ? orrery_log_depth(best_depth * u_mass_length)
                       : 1.0;
}
