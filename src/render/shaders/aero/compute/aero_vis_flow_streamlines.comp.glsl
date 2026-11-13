#version 460 core
// Streamlines: one invocation per seed integrates dx/ds = sign u / |u| with
// classical RK4 in arc length (u_step grid spacings per step) and writes up
// to u_max_points points (index coordinates, w = |u| in m/s) and their count.
// Same stopping rules as orrery::render::trace_streamlines.

layout(local_size_x = 64, local_size_y = 1, local_size_z = 1) in;

layout(std430, binding = 2) readonly buffer Seeds {
    vec4 seeds[];
};
layout(std430, binding = 3) writeonly buffer Points {
    vec4 points[];
};
layout(std430, binding = 4) writeonly buffer Counts {
    uint counts[];
};

uniform uint u_seed_count;
uniform uint u_max_points;
uniform float u_step;
uniform float u_sign;
uniform float u_stagnation; // m/s

// Unit direction at p; false where a line stops.
bool direction(vec3 p, out vec3 d) {
    d = vec3(0.0);
    if (!flow_inside(p) || flow_solid(p)) {
        return false;
    }
    vec3 u = flow_velocity(p);
    float speed = length(u);
    if (!(speed > u_stagnation)) {
        return false;
    }
    d = u * (u_sign / speed);
    return true;
}

void main() {
    uint s = gl_GlobalInvocationID.x;
    if (s >= u_seed_count) {
        return;
    }
    uint base = s * u_max_points;
    vec3 p = seeds[s].xyz;
    vec3 k1;
    uint count = 0u;
    if (u_max_points > 0u && direction(p, k1)) {
        points[base] = vec4(p, length(flow_velocity(p)));
        count = 1u;
        while (count < u_max_points) {
            vec3 k2;
            vec3 k3;
            vec3 k4;
            vec3 probe;
            if (!direction(p, k1) || !direction(p + 0.5 * u_step * k1, k2) ||
                !direction(p + 0.5 * u_step * k2, k3) || !direction(p + u_step * k3, k4)) {
                break;
            }
            vec3 next = p + (u_step / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
            if (!direction(next, probe)) {
                break;
            }
            p = next;
            points[base + count] = vec4(p, length(flow_velocity(p)));
            ++count;
        }
    }
    counts[s] = count;
}
