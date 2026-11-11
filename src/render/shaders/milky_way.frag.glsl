#version 460 core
// Milky Way band: diffuse galactic light behind the stars, evaluated per
// pixel from the view direction (rotation only: the band is at infinity).
// Used with fullscreen.vert.glsl; noise.glsl is composed in before this file.

in vec2 v_uv;

uniform mat4 u_inverse_view_projection; // clip -> render space
uniform mat3 u_ecliptic_to_galactic;
uniform float u_intensity; // scene-linear radiance scale

layout(location = 0) out vec4 o_color;

const float kPi = 3.14159265358979;

void main() {
    vec4 ndc = vec4(v_uv * 2.0 - 1.0, 0.0, 1.0);
    vec4 point = u_inverse_view_projection * ndc;
    vec3 direction = normalize(point.xyz / point.w);
    vec3 g = u_ecliptic_to_galactic * direction;
    float b = asin(clamp(g.z, -1.0, 1.0)); // galactic latitude
    float l = atan(g.y, g.x);              // galactic longitude, 0 towards the centre

    // APPROXIMATION: Milky Way surface brightness - a Gaussian disc in
    // latitude that thickens and brightens towards the galactic centre, a
    // central bulge, value-noise mottling and dust lanes along the plane -
    // reproduces the band's position and overall shape, not its detail.
    float centre = exp(-(l * l) / (2.0 * 0.9 * 0.9));
    vec3 p = direction * 4.0;
    // Latitude wobble so that the band edges are not perfectly straight.
    float wobble = (orrery_fbm(p * 0.7 + vec3(11.0, 3.0, -2.0), 3) - 0.5) * 0.08;
    float bw = b + wobble;
    float width = 0.09 + 0.08 * centre;
    float disc = exp(-(bw * bw) / (2.0 * width * width)) * (0.30 + 0.70 * centre);
    float halo = exp(-(b * b) / (2.0 * 0.45 * 0.45)) * (0.04 + 0.10 * centre);
    float bulge = exp(-((l * l) / (2.0 * 0.22 * 0.22) + (b * b) / (2.0 * 0.16 * 0.16)));

    // Star clouds: clumpy brightness along the plane.
    float clouds = orrery_fbm(p * 2.6, 6);
    float clumps = smoothstep(0.30, 0.80, clouds);
    float grain = orrery_fbm(p * 70.0 + vec3(3.1, -1.7, 8.2), 2);
    float brightness = disc * (0.45 + 0.9 * clumps) * (0.7 + 0.6 * grain) + halo +
                       1.6 * bulge * (0.75 + 0.5 * clumps);

    // Dark rifts: a meandering lane along the plane, broken up by noise.
    float lane_centre = (orrery_fbm(p * 1.1 + vec3(-4.0, 2.0, 1.0), 4) - 0.5) * 0.10;
    float lane_width = 0.025 + 0.03 * centre;
    float lane = exp(-((b - lane_centre) * (b - lane_centre)) / (2.0 * lane_width * lane_width));
    float rifts = smoothstep(0.45, 0.65, orrery_fbm(p * 3.5 + vec3(7.0, -3.0, 5.0), 5));
    float dust = lane * rifts;
    brightness *= 1.0 - 0.85 * dust;

    vec3 warm = vec3(1.00, 0.91, 0.80);
    vec3 cool = vec3(0.80, 0.87, 1.00);
    vec3 tint = mix(cool, warm, clamp(0.15 + 0.85 * centre + 0.5 * bulge, 0.0, 1.0));
    o_color = vec4(tint * brightness * u_intensity, 1.0);
}
