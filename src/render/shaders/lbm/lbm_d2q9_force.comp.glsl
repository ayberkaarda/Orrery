#version 460 core
// Momentum exchange per boundary link: link k = (cell, j) at binding 4 has
// a solid source node in direction j, so the population leaving towards the
// wall is o = opposite(j) and the wall receives 2 c_o f*_o per step. Written
// as the shifted part 2 c_o fs_o (binding 5); the host adds the weight part
// 2 c_o w_o and sums all links in double precision.

layout(local_size_x = 128, local_size_y = 1, local_size_z = 1) in;

layout(std430, binding = 4) readonly buffer LbmLinks {
    uint links[];
};
layout(std430, binding = 5) writeonly buffer LbmLinkForce {
    float link_force[];
};

uniform uint u_link_count;

void main() {
    uint k = gl_GlobalInvocationID.x;
    if (k >= u_link_count) {
        return;
    }
    uint cell = links[2u * k];
    int out_direction = LBM_OPPOSITE[int(links[2u * k + 1u])];
    float f = lbm_source(out_direction, cell);
    link_force[2u * k] = 2.0 * float(LBM_CX[out_direction]) * f;
    link_force[2u * k + 1u] = 2.0 * float(LBM_CY[out_direction]) * f;
}
