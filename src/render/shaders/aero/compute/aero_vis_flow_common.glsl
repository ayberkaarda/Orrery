// Flow field access shared by the aero_vis_flow_* kernels. Mirrors the CPU
// reference in src/render/aero/aero_streamlines.cpp and aero_field.cpp:
// positions are grid index coordinates (node (i, j, k) at (i, j, k)),
// velocity is trilinear between nodes (bilinear on a 2D grid, nz = 1), and a
// point is in a body where its nearest node is solid.

layout(std430, binding = 0) readonly buffer FlowVelocity {
    vec4 velocity[]; // m/s per node, w unused
};
layout(std430, binding = 1) readonly buffer FlowSolid {
    uint solid_words[]; // one byte per node, four nodes per word
};

uniform uint u_nx;
uniform uint u_ny;
uniform uint u_nz;

uint flow_node(uvec3 n) {
    return n.x + u_nx * (n.y + u_ny * n.z);
}

vec3 flow_last() {
    return vec3(float(u_nx - 1u), float(u_ny - 1u), float(max(u_nz, 1u) - 1u));
}

bool flow_inside(vec3 p) {
    vec3 last = flow_last();
    const float kTolerance = 1.0e-5;
    bool z_ok = u_nz < 2u || (p.z >= -kTolerance && p.z <= last.z + kTolerance);
    return p.x >= -kTolerance && p.x <= last.x + kTolerance && p.y >= -kTolerance &&
           p.y <= last.y + kTolerance && z_ok;
}

bool flow_solid(vec3 p) {
    uvec3 n = uvec3(clamp(round(p), vec3(0.0), flow_last()));
    if (u_nz < 2u) {
        n.z = 0u;
    }
    uint node = flow_node(n);
    return ((solid_words[node >> 2u] >> ((node & 3u) * 8u)) & 0xffu) != 0u;
}

// Trilinear velocity at p (clamped into the grid).
vec3 flow_velocity(vec3 p) {
    vec3 last = flow_last();
    vec3 q = clamp(p, vec3(0.0), last);
    vec3 base = min(floor(q), max(last - 1.0, vec3(0.0)));
    vec3 w = q - base;
    uvec3 c = uvec3(base);
    uint kz = u_nz < 2u ? 1u : 2u;
    if (u_nz < 2u) {
        c.z = 0u;
        w.z = 0.0;
    }
    vec3 sum = vec3(0.0);
    for (uint dz = 0u; dz < kz; ++dz) {
        float wz = kz == 1u ? 1.0 : (dz == 0u ? 1.0 - w.z : w.z);
        for (uint dy = 0u; dy < 2u; ++dy) {
            float wy = dy == 0u ? 1.0 - w.y : w.y;
            for (uint dx = 0u; dx < 2u; ++dx) {
                float wx = dx == 0u ? 1.0 - w.x : w.x;
                sum += velocity[flow_node(c + uvec3(dx, dy, dz))].xyz * (wx * wy * wz);
            }
        }
    }
    return sum;
}
