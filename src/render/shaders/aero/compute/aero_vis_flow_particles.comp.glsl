#version 460 core
// Particles: one invocation per particle advances it by u_dt seconds with
// the midpoint rule (orrery::render::advect_particle) in index coordinates.
// A particle that leaves the grid, enters a body or outlives u_lifetime is
// released again on a seed (u_release = 1, with a deterministic jitter) or
// marked dead (w = -1). Its trace keeps the last u_trace_points positions in
// a ring; slot u_head receives this step's position, w = age (or -1 when the
// slot is not part of the current life).

layout(local_size_x = 128, local_size_y = 1, local_size_z = 1) in;

layout(std430, binding = 2) readonly buffer Seeds {
    vec4 seeds[];
};
layout(std430, binding = 3) buffer Particles {
    vec4 particles[]; // index position, age [s] (< 0: dead)
};
layout(std430, binding = 4) buffer Traces {
    vec4 traces[]; // particle * u_trace_points + slot
};

uniform uint u_count;
uniform uint u_seed_count;
uniform uint u_trace_points;
uniform uint u_head;
uniform uint u_release;
uniform uint u_frame;
uniform float u_dt;       // [s]
uniform float u_dt_index; // u_dt / grid spacing [s / m]
uniform float u_lifetime; // [s]
uniform float u_jitter;   // release spread [grid spacings]

uint hash(uint x) {
    x ^= x >> 16u;
    x *= 0x7feb352du;
    x ^= x >> 15u;
    x *= 0x846ca68bu;
    x ^= x >> 16u;
    return x;
}

float unit_random(uint x) {
    return float(hash(x) >> 8u) * (1.0 / 16777216.0);
}

bool usable(vec3 p) {
    return flow_inside(p) && !flow_solid(p);
}

bool advect(vec3 p, out vec3 next) {
    next = p;
    if (!usable(p)) {
        return false;
    }
    vec3 middle = p + 0.5 * u_dt_index * flow_velocity(p);
    if (!usable(middle)) {
        return false;
    }
    next = p + u_dt_index * flow_velocity(middle);
    return usable(next);
}

void main() {
    uint i = gl_GlobalInvocationID.x;
    if (i >= u_count) {
        return;
    }
    vec4 state = particles[i];
    bool alive = state.w >= 0.0;
    if (alive) {
        vec3 next;
        float age = state.w + u_dt;
        if (age <= u_lifetime && advect(state.xyz, next)) {
            state = vec4(next, age);
        } else {
            alive = false;
        }
    }
    bool released = false;
    if (!alive) {
        if (u_release == 1u && u_seed_count > 0u) {
            uint key = hash(i * 0x9e3779b9u ^ u_frame);
            vec3 seed = seeds[key % u_seed_count].xyz;
            vec3 jitter = vec3(unit_random(key + 1u), unit_random(key + 2u),
                               unit_random(key + 3u)) - 0.5;
            if (u_nz < 2u) {
                jitter.z = 0.0;
            }
            state = vec4(seed + u_jitter * jitter, 0.0);
            released = usable(state.xyz);
            if (!released) {
                state.w = -1.0;
            }
        } else {
            state.w = -1.0;
        }
    }
    particles[i] = state;
    if (u_trace_points > 0u) {
        uint base = i * u_trace_points;
        if (released) {
            for (uint k = 0u; k < u_trace_points; ++k) {
                traces[base + k] = vec4(state.xyz, -1.0);
            }
        }
        traces[base + (u_head % u_trace_points)] = vec4(state.xyz, state.w);
    }
}
