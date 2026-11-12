#version 460 core
// Milky Way band: diffuse galactic light behind the stars, evaluated per
// pixel from the view direction (rotation only: the band is at infinity).
// Used with fullscreen.vert.glsl; noise.glsl is composed in before this file.

in vec2 v_uv;

uniform mat4 u_inverse_view_projection; // clip -> render space
uniform mat3 u_ecliptic_to_galactic;
uniform float u_intensity; // scene-linear radiance scale

layout(location = 0) out vec4 o_color;

void main() {
    vec4 ndc = vec4(v_uv * 2.0 - 1.0, 0.0, 1.0);
    vec4 point = u_inverse_view_projection * ndc;
    vec3 direction = normalize(point.xyz / point.w);
    vec3 g = u_ecliptic_to_galactic * direction;
    float b = asin(clamp(g.z, -1.0, 1.0)); // galactic latitude
    float l = atan(g.y, g.x);              // galactic longitude, 0 towards the centre

    // APPROXIMATION: Milky Way surface brightness - a Gaussian disc in
    // latitude that thickens and brightens towards the galactic centre, a
    // central bulge, value-noise star clouds and filamentary dust lanes along
    // the plane - reproduces the band's position and overall shape, not its
    // detail.
    float centre = exp(-(l * l) / (2.0 * 0.9 * 0.9));
    // Noise coordinates stretched along the plane (galactic z weighs 1.5 times as
    // much as x and y), so that clouds and lanes run along the band instead
    // of forming round blobs, then domain-warped so that they meander; the
    // galactic axes avoid any seam.
    vec3 q = vec3(g.xy * 3.0, g.z * 4.5);
    q += 0.9 * vec3(orrery_fbm(q * 0.9 + vec3(1.7, 9.2, -3.3), 3) - 0.5,
                    orrery_fbm(q * 0.9 + vec3(-8.3, 2.8, 5.1), 3) - 0.5, 0.0);
    // Latitude wobble so that the band edges are not perfectly straight.
    float wobble = (orrery_fbm(q * 0.4 + vec3(11.0, 3.0, -2.0), 3) - 0.5) * 0.06;
    float bw = b + wobble;
    float width = 0.10 + 0.08 * centre;
    float disc = exp(-(bw * bw) / (2.0 * width * width)) * (0.30 + 0.70 * centre);
    float halo = exp(-(b * b) / (2.0 * 0.45 * 0.45)) * (0.03 + 0.08 * centre);
    float bulge = exp(-((l * l) / (2.0 * 0.22 * 0.22) + (b * b) / (2.0 * 0.16 * 0.16)));

    // Star clouds: elongated brightness clumps along the plane, with a gentle
    // finer texture (the resolved stars come from the starfield).
    float clouds = orrery_fbm(q * 2.4, 6);
    float clumps = smoothstep(0.30, 0.80, clouds);
    float texture_fine = orrery_fbm(q * 9.0 + vec3(3.1, -1.7, 8.2), 3);
    float brightness = disc * (0.40 + 0.95 * clumps) * (0.88 + 0.24 * texture_fine) + halo +
                       1.15 * bulge * (0.75 + 0.5 * clumps);

    // Dust: a meandering lane along the plane, broken into filaments; partial
    // extinction keeps some light behind it, as in the Great Rift.
    float lane_centre = (orrery_fbm(q * 0.6 + vec3(-4.0, 2.0, 1.0), 4) - 0.5) * 0.08;
    float lane_width = 0.022 + 0.03 * centre;
    float lane = exp(-((b - lane_centre) * (b - lane_centre)) / (2.0 * lane_width * lane_width));
    float filaments = smoothstep(0.42, 0.68, orrery_fbm(q * 3.2 + vec3(7.0, -3.0, 5.0), 5));
    float dust = lane * (0.35 + 0.65 * filaments);
    brightness *= 1.0 - 0.7 * dust;

    // Old disc light is close to neutral white with a slight blue cast away
    // from the centre; the bulge is warmer. Kept desaturated: the band is a
    // backdrop, the bodies and trails carry the colour.
    vec3 warm = vec3(1.00, 0.90, 0.78);
    vec3 cool = vec3(0.86, 0.91, 1.00);
    vec3 tint = mix(cool, warm, clamp(0.10 + 0.60 * centre + 0.6 * bulge, 0.0, 1.0));
    o_color = vec4(tint * brightness * u_intensity, 1.0);
}
