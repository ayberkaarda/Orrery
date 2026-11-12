#version 460 core
// Schwarzschild null-geodesic ray marcher (docs/rendering.md, "Black hole";
// ADR-003, ADR-017). One ray per half-resolution sample is traced backwards
// from a static camera. In units of M = G M / c^2 (horizon at r = 2) the
// photon moves in a plane; with u = 1/r and phi the angle in that plane
//
//     d^2u/dphi^2 = -u + 3 u^2,
//
// integrated with RK4 at a step of at most 4 pi / u_max_steps, shortened for
// steep (nearly radial) rays. A ray ends when it crosses the horizon
// (u >= 1/2), reaches infinity (u <= 0), saturates on the disk or runs out of
// steps. Crossings of the disk plane fall at known angles (every pi), so they
// are found exactly and u there is interpolated with the RK4 dense output.
// CPU twin (double precision, same rules): orrery::core::geodesic::trace_ray.
// noise.glsl is composed in before this file.

layout(local_size_x = 8, local_size_y = 8) in;

// Radiance output:  o_disk = disk emission (rgb), sky transmittance (a; 0 when
//                   the ray does not escape); o_sky = escape direction (xyz),
//                   view depth of the first opaque hit or of the hole [M] (w).
// Geometry output:  o_disk = first on-disk radius, its g, on-disk crossings,
//                   fate (0 escaped, 1 captured, 2 exhausted); o_sky = escape
//                   direction, impact parameter.
layout(rgba32f, binding = 0) uniform writeonly image2D o_disk;
layout(rgba32f, binding = 1) uniform writeonly image2D o_sky;
layout(binding = 0) uniform sampler2D u_blackbody; // N x 1: linear sRGB radiance, even in ln T
layout(binding = 1) uniform samplerCube u_environment; // the unlensed sky (radiance output only)

uniform ivec2 u_trace_size;
uniform vec2 u_viewport;          // full-resolution size [px]
uniform mat3 u_camera_basis;      // camera -> world: columns right, up, back
uniform vec2 u_tan_half_fov;      // tan of the half field of view (x, y)
uniform vec3 u_camera_position;   // static camera relative to the hole [M]
uniform vec3 u_disk_axis;         // unit normal; the gas orbits counter-clockwise about it
uniform vec3 u_disk_x;            // unit vectors spanning the disk plane
uniform vec3 u_disk_y;
uniform vec2 u_disk_radii;        // inner (ISCO), outer [M]
uniform int u_disk_enabled;
uniform float u_disk_temperature; // peak temperature [K]
uniform float u_disk_radiance;    // radiance factor (unit luminance at the peak temperature)
uniform vec2 u_blackbody_range;   // ln T of the first texel, texels per unit ln T
uniform float u_blackbody_texels;
uniform int u_max_steps;
uniform float u_base_step;        // 4 pi / u_max_steps
uniform float u_observer_shift;   // 1 / sqrt(1 - 2 / r_camera)
uniform int u_geometry_output;
uniform float u_environment_texel;   // 2 / face size: texel angle at a face centre [rad]
uniform float u_environment_max_lod;
uniform float u_environment_scale;   // 0 hides the sky (starfield off)

const float kPi = 3.14159265358979;
const float kStepGrowth = 0.25;     // core::geodesic::kStepGrowth
const float kStepReferenceU = 0.01; // core::geodesic::kStepReferenceU
const float kRadialThreshold = 1.0e-7;
const float kSaturated = 1.0e-3;    // transmittance that ends a ray on the disk
const float kCriticalB = 5.196152422706632; // 3 sqrt(3)
const float kRingBand = 0.08;       // relative band of b around b_c that is supersampled
const float kMaxRadiance = 1.0e4;   // radiance bound (black_hole_composite.frag.glsl)

vec2 orbit_rhs(vec2 s) {
    return vec2(s.y, -s.x + 3.0 * s.x * s.x);
}

vec2 rk4_step(vec2 s, float h) {
    vec2 k1 = orbit_rhs(s);
    vec2 k2 = orbit_rhs(s + 0.5 * h * k1);
    vec2 k3 = orbit_rhs(s + 0.5 * h * k2);
    vec2 k4 = orbit_rhs(s + h * k3);
    return s + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

float step_size(vec2 s) {
    return min(u_base_step, kStepGrowth * max(s.x, kStepReferenceU) / max(abs(s.y), 1.0e-30));
}

float hermite(vec2 a, vec2 b, float h, float t) {
    float t2 = t * t;
    float t3 = t2 * t;
    return (2.0 * t3 - 3.0 * t2 + 1.0) * a.x + (t3 - 2.0 * t2 + t) * h * a.y +
           (-2.0 * t3 + 3.0 * t2) * b.x + (t3 - t2) * h * b.y;
}

// Thin-disk temperature with a zero-torque inner edge, normalised to 1 at
// its maximum r = (49/36) r_in (core::geodesic::thin_disk_temperature).
float disk_temperature(float r) {
    float x = r / u_disk_radii.x;
    if (x <= 1.0) {
        return 0.0;
    }
    const float peak = 0.48758; // x^(-3/4) (1 - x^(-1/2))^(1/4) at x = 49/36
    return pow(x, -0.75) * pow(1.0 - inversesqrt(x), 0.25) / peak;
}

// Visible radiance of a black body at the observed temperature.
vec3 blackbody(float kelvin) {
    float x = (log(max(kelvin, 1.0)) - u_blackbody_range.x) * u_blackbody_range.y;
    if (x < 0.0) {
        return vec3(0.0);
    }
    float coordinate = (min(x, u_blackbody_texels - 1.0) + 0.5) / u_blackbody_texels;
    return textureLod(u_blackbody, vec2(coordinate, 0.5), 0.0).rgb;
}

// APPROXIMATION: disk opacity and texture - the disk is optically thick
// (opacity 1) out to 82 percent of the outer radius and thins smoothly to 0
// at the outer radius, and its emission carries a sheared value-noise
// pattern of fine orbital streaks (factor 0.4 .. 1.8, mean about 1) - a
// thin disk has neither a sharp outer rim nor a known surface pattern; the
// streaks only let the eye follow the orbital flow through the lensed
// images - display only: temperature profile, Doppler and gravitational
// shifts are not affected.
float disk_opacity(float r) {
    return 1.0 - smoothstep(0.82 * u_disk_radii.y, u_disk_radii.y, r);
}

float disk_texture(vec3 hit, float r) {
    float azimuth = atan(dot(hit, u_disk_y), dot(hit, u_disk_x));
    // Differential rotation shears the pattern into trailing spirals; the
    // coordinates run around the circle, so the pattern has no seam.
    float lr = log(r);
    float sheared = azimuth + 3.0 * lr;
    float warp = orrery_fbm(vec3(cos(sheared) * 3.0, sin(sheared) * 3.0, lr * 4.0) - 7.0, 3);
    vec3 q = vec3(cos(sheared) * 2.2, sin(sheared) * 2.2, lr * 18.0 + 1.5 * warp);
    float streaks = orrery_fbm(q, 5);
    float clumps = orrery_fbm(vec3(cos(sheared) * 5.0, sin(sheared) * 5.0, lr * 6.0) + 11.0, 3);
    return clamp(0.4 + 1.25 * streaks * (0.65 + 0.7 * clumps), 0.4, 1.8);
}

struct Trace {
    vec3 radiance;       // disk emission reaching the camera
    float transmittance; // of the disk in front of what lies behind
    vec3 direction;      // escape direction (or the direction where the ray ended)
    float depth;         // view depth of the first opaque hit [M], 0 for none
    int fate;            // 0 escaped, 1 captured or covered, 2 out of steps
    float first_radius;  // first on-disk crossing [M]
    float first_g;
    int on_disk;         // on-disk crossings
    float b;             // impact parameter [M]
    bool shadow;         // inwards with b < b_c: inside the critical curve
};

// APPROXIMATION: static camera and step budget - the camera is a static
// observer at r0 (no aberration or Doppler shift from its own motion, which
// the space camera does not have physically), and a ray that uses up the
// step budget (it has circled the photon sphere about twice) counts as
// captured - exact for a camera at rest; with 512 steps only rays within
// about 1e-4 of b_c are cut short, a band far below one pixel.
Trace trace_ray(vec3 d) {
    Trace t;
    t.radiance = vec3(0.0);
    t.transmittance = 1.0;
    t.direction = d;
    t.depth = 0.0;
    t.fate = 2;
    t.first_radius = 0.0;
    t.first_g = 0.0;
    t.on_disk = 0;
    t.b = 0.0;
    t.shadow = false;
    vec3 back = u_camera_basis[2];
    float r0 = length(u_camera_position);
    vec3 e1 = u_camera_position / r0;
    float cos_a = dot(d, e1);
    vec3 transverse = d - cos_a * e1;
    float sin_a = length(transverse);
    if (r0 <= 2.0) {
        t.fate = 1; // the camera is inside the horizon
        t.depth = 1.0e-3;
        return t;
    }
    if (sin_a < kRadialThreshold) {
        t.fate = cos_a < 0.0 ? 1 : 0; // straight in or straight out
        return t;
    }
    vec3 e2 = transverse / sin_a;
    float u0 = 1.0 / r0;
    t.b = r0 * sin_a * u_observer_shift;
    t.shadow = cos_a < 0.0 && t.b < kCriticalB;
    // The physical photon runs the traced path backwards.
    float lambda_z = -t.b * dot(cross(e1, e2), u_disk_axis);

    float ca = dot(e1, u_disk_axis);
    float cc = dot(e2, u_disk_axis);
    bool crosses = length(vec2(ca, cc)) > kRadialThreshold;
    float next_crossing = mod(atan(cc, ca) + 0.5 * kPi + 2.0 * kPi, kPi);
    if (next_crossing <= 0.0) {
        next_crossing += kPi;
    }

    vec2 s = vec2(u0, -u0 * cos_a / (sin_a * u_observer_shift));
    float phi = 0.0;
    for (int step = 0; step < u_max_steps; ++step) {
        float h = step_size(s);
        vec2 next = rk4_step(s, h);
        while (crosses && next_crossing <= phi + h) {
            float uc = hermite(s, next, h, (next_crossing - phi) / h);
            float r = uc > 0.0 ? 1.0 / uc : 0.0;
            if (uc > 0.0 && uc < 0.5 && r >= u_disk_radii.x && r <= u_disk_radii.y) {
                float g = sqrt(1.0 - 3.0 / r) / (1.0 - lambda_z / (r * sqrt(r))) *
                          u_observer_shift;
                if (t.on_disk == 0) {
                    t.first_radius = r;
                    t.first_g = g;
                }
                ++t.on_disk;
                if (u_geometry_output == 0 && u_disk_enabled != 0) {
                    vec3 hit = (cos(next_crossing) * e1 + sin(next_crossing) * e2) * r;
                    float opacity = disk_opacity(r);
                    vec3 emission = blackbody(g * u_disk_temperature * disk_temperature(r)) *
                                    (u_disk_radiance * disk_texture(hit, r));
                    t.radiance += t.transmittance * opacity * emission;
                    float after = t.transmittance * (1.0 - opacity);
                    if (t.depth == 0.0 && after < 0.5) {
                        t.depth = dot(u_camera_position - hit, back);
                    }
                    t.transmittance = after;
                }
            }
            next_crossing += kPi;
        }
        phi += h;
        s = next;
        if (s.x >= 0.5) {
            t.fate = 1;
            break;
        }
        if (s.x <= 0.0) {
            // Infinity lies just behind: near u = 0 the orbit is the straight
            // line u = A sin(phi_inf - phi) (u'' = -u + 3u^2 ~ -u).
            float phi_inf = phi + atan(s.x, -s.y);
            t.direction = cos(phi_inf) * e1 + sin(phi_inf) * e2;
            t.fate = 0;
            break;
        }
        if (t.transmittance < kSaturated) {
            t.fate = 1; // fully covered by the disk: nothing behind it shows
            break;
        }
    }
    if (t.fate != 0) {
        // Direction of travel where the ray ended: keeps the stored field
        // continuous for the upsampling (the sky weight is 0 there).
        t.direction = cos(phi) * e1 + sin(phi) * e2;
    }
    return t;
}

// APPROXIMATION: radiance bound - the light of a sample is scaled down, hue
// kept, so that no component exceeds kMaxRadiance (scene-linear; the disk's
// peak temperature shows at about 2.2); the composite applies the same bound
// to every pixel it writes - a camera close to the horizon sees
// the disk blueshifted by g ~ 1 / sqrt(1 - 2 / r_camera) without limit (the
// look-up of B(g T) itself stops at the table's 200000 K), and the visible
// radiance of B(g T) outgrows B(T) by 10^5 and more (2000 K disk, g = 11),
// beyond the largest half float of the HDR target (65504), which would store
// infinity and turn into NaN in tone mapping - display only: the bound lies
// 4500 times above the disk's peak and still maps to white at the lowest
// exposure (1/64); it is reached only for g of 2 or more even with the
// coolest, brightest disk (2000 K, brightness 4), that is for a camera within
// a few M of the horizon, never for a distant one (g of about sqrt(2) at
// most).
vec3 bound_radiance(vec3 c) {
    float peak = max(c.r, max(c.g, c.b));
    return peak > kMaxRadiance ? c * (kMaxRadiance / peak) : c;
}

vec3 sample_ray(vec2 pixel) {
    vec2 ndc = pixel / u_viewport * 2.0 - 1.0;
    return normalize(u_camera_basis * vec3(ndc * u_tan_half_fov, -1.0));
}

void main() {
    ivec2 cell = ivec2(gl_GlobalInvocationID.xy);
    if (cell.x >= u_trace_size.x || cell.y >= u_trace_size.y) {
        return;
    }
    vec2 pixel = 2.0 * vec2(cell) + 1.0;
    Trace t = trace_ray(sample_ray(pixel));
    if (u_geometry_output != 0) {
        imageStore(o_disk, cell, vec4(t.first_radius, t.first_g, float(t.on_disk), float(t.fate)));
        imageStore(o_sky, cell, vec4(t.direction, t.b));
        return;
    }
    vec3 radiance = t.radiance;
    float sky = t.fate == 0 ? t.transmittance : 0.0;
    // Close to the critical curve the higher-order images of the disk and of
    // the sky (the photon ring) are far thinner than a trace sample and the
    // escape directions change faster than the composite can interpolate:
    // trace the four pixel centres that the sample stands for and resolve
    // their light here, the sky filtered over the spread of their escape
    // directions. The sample then carries no sky for the composite.
    // Only sub-rays on the sample's own side of the critical curve count
    // (the composite blends samples per side), unless none is.
    if (abs(t.b / kCriticalB - 1.0) < kRingBand) {
        Trace subs[4];
        float spread = 0.0;
        int same_side = 0;
        for (int k = 0; k < 4; ++k) {
            vec2 offset = vec2(float(k & 1), float(k >> 1)) - 0.5;
            subs[k] = trace_ray(sample_ray(pixel + offset));
            if (subs[k].fate == 0) {
                spread = max(spread, length(subs[k].direction - t.direction));
            }
            same_side += subs[k].shadow == t.shadow ? 1 : 0;
        }
        radiance = vec3(0.0);
        float weight = same_side > 0 ? 1.0 / float(same_side) : 0.25;
        for (int k = 0; k < 4; ++k) {
            if (same_side > 0 && subs[k].shadow != t.shadow) {
                continue;
            }
            radiance += weight * subs[k].radiance;
            if (subs[k].fate == 0) {
                vec3 dir = subs[k].direction;
                vec3 a = abs(dir);
                float m = max(a.x, max(a.y, a.z));
                float texel = u_environment_texel * m * sqrt(m);
                float lod = clamp(log2(max(spread / texel, 1.0e-6)), 0.0, u_environment_max_lod);
                radiance += weight * subs[k].transmittance * u_environment_scale *
                            textureLod(u_environment, dir, lod).rgb;
            }
        }
        sky = 0.0;
    }
    float depth = t.depth;
    if (t.fate != 0 && depth == 0.0) {
        depth = max(dot(u_camera_position, u_camera_basis[2]), 1.0e-3);
    }
    imageStore(o_disk, cell, vec4(bound_radiance(radiance), sky));
    imageStore(o_sky, cell, vec4(t.direction, depth));
}
