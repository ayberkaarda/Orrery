// Procedural noise snippet. Composed after the #version line of a stage (see
// orrery::render::compose_shader) and never compiled on its own.
//
// Integer hashing (PCG3D, Jarzynski and Olano 2020) keeps the noise identical
// on every driver: no sin() hashes, no textures.

uvec3 orrery_pcg3d(uvec3 v) {
    v = v * 1664525u + 1013904223u;
    v.x += v.y * v.z;
    v.y += v.z * v.x;
    v.z += v.x * v.y;
    v ^= v >> 16u;
    v.x += v.y * v.z;
    v.y += v.z * v.x;
    v.z += v.x * v.y;
    return v;
}

// Hash of a lattice point to [0, 1).
float orrery_hash(ivec3 cell) {
    return float(orrery_pcg3d(uvec3(cell)).x >> 8u) * (1.0 / 16777216.0);
}

// Trilinear value noise in [0, 1) with quintic fade.
float orrery_value_noise(vec3 p) {
    ivec3 i = ivec3(floor(p));
    vec3 f = fract(p);
    vec3 u = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
    float n000 = orrery_hash(i);
    float n100 = orrery_hash(i + ivec3(1, 0, 0));
    float n010 = orrery_hash(i + ivec3(0, 1, 0));
    float n110 = orrery_hash(i + ivec3(1, 1, 0));
    float n001 = orrery_hash(i + ivec3(0, 0, 1));
    float n101 = orrery_hash(i + ivec3(1, 0, 1));
    float n011 = orrery_hash(i + ivec3(0, 1, 1));
    float n111 = orrery_hash(i + ivec3(1, 1, 1));
    return mix(mix(mix(n000, n100, u.x), mix(n010, n110, u.x), u.y),
               mix(mix(n001, n101, u.x), mix(n011, n111, u.x), u.y), u.z);
}

// Fractal sum of `octaves` value-noise octaves (lacunarity 2, gain 0.5),
// normalised to [0, 1).
float orrery_fbm(vec3 p, int octaves) {
    float sum = 0.0;
    float amplitude = 0.5;
    float norm = 0.0;
    for (int o = 0; o < octaves; ++o) {
        sum += amplitude * orrery_value_noise(p);
        norm += amplitude;
        p = p * 2.03 + vec3(17.1, -9.7, 4.3);
        amplitude *= 0.5;
    }
    return sum / norm;
}
