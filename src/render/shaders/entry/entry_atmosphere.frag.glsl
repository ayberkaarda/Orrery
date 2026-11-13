#version 460 core
// Atmosphere shell of the entry view, drawn with fullscreen.vert.glsl after
// the planet and blended over it (SRC_ALPHA, ONE_MINUS_SRC_ALPHA; no depth
// test: the planet is intersected here).
//
// APPROXIMATION: a single-colour exponential haze (scale height a quarter of
// the shell thickness) ray-marched in 16 samples between the shell and the
// planet; each sample is lit by a wrapped sun term. Distances are in planet
// radii, so the intersections stay well conditioned in float.

in vec2 v_uv;

uniform vec3 u_right;    // camera right x tan(fov / 2) x aspect, render axes
uniform vec3 u_up;       // camera up x tan(fov / 2)
uniform vec3 u_forward;  // camera forward
uniform vec3 u_center;   // planet centre relative to the eye [planet radii]
uniform float u_shell;   // shell radius [planet radii], > 1
uniform vec3 u_sun;      // towards the sun (normalised)
uniform vec3 u_color;    // scene-linear haze colour on the day side
uniform float u_strength; // optical depth of one scale height

layout(location = 0) out vec4 o_color;

void main() {
    vec2 ndc = v_uv * 2.0 - 1.0;
    vec3 rd = normalize(u_forward + ndc.x * u_right + ndc.y * u_up);
    float b = dot(rd, u_center);
    vec3 closest = rd * b - u_center;
    float d2 = dot(closest, closest);
    float s2 = u_shell * u_shell;
    if (d2 >= s2) {
        discard;
    }
    float half_chord = sqrt(s2 - d2);
    float t0 = max(b - half_chord, 0.0);
    float t1 = b + half_chord;
    if (d2 < 1.0) {
        float ground = b - sqrt(1.0 - d2);
        if (ground > 0.0) {
            t1 = min(t1, ground);
        }
    }
    if (t1 <= t0) {
        discard;
    }
    float h = max((u_shell - 1.0) * 0.25, 1.0e-6);
    float dt = (t1 - t0) / 16.0;
    float depth = 0.0;
    float light = 0.0;
    for (int i = 0; i < 16; ++i) {
        vec3 p = rd * (t0 + (float(i) + 0.5) * dt) - u_center;
        float r = length(p);
        float density = exp(-max(r - 1.0, 0.0) / h) * dt;
        float sun = clamp(dot(p / r, u_sun) * 1.4 + 0.35, 0.0, 1.0);
        depth += density;
        light += density * (0.12 + 0.88 * sun);
    }
    float alpha = 1.0 - exp(-u_strength * depth / h);
    if (alpha <= 0.001) {
        discard;
    }
    vec3 color = u_color * (light / max(depth, 1.0e-9));
    o_color = vec4(color, alpha);
}
