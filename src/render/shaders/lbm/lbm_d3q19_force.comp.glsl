#version 460 core
// Momentum exchange per boundary link: link k = (cell, j) at binding 4 has a
// solid source node in direction j, so the population leaving towards the
// wall is o = opposite(j) and the one coming back is the reflected f_j of
// the next step. The kernel writes the sum of both, shifted (binding 5); the
// host adds the weights back and sums c_o (f*_o + f_j) over all links in
// double precision.

layout(local_size_x = 64, local_size_y = 1, local_size_z = 1) in;

layout(std430, binding = 4) readonly buffer Lbm3Links {
    uint links[];
};
layout(std430, binding = 5) writeonly buffer Lbm3LinkForce {
    float link_force[];
};

uniform uint u_link_count;

void main() {
    uint k = gl_GlobalInvocationID.x;
    if (k >= u_link_count) {
        return;
    }
    uint cell = links[2u * k];
    int j = int(links[2u * k + 1u]);
    uint plane = u_nx * u_ny;
    int z = int(cell / plane);
    int y = int((cell % plane) / u_nx);
    int x = int(cell % u_nx);
    float back = lbm3_reflected(j, cell, x, y, z, lbm3_wall_fraction(node_class[cell], j));
    link_force[k] = lbm3_source(LBM3_OPPOSITE[j], cell) + back;
}
