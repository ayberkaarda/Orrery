#version 460 core
// One fused D2Q9 time step: pull-stream with boundaries, then collide
// (lbm_d2q9_common.glsl). Reads binding 0, writes binding 2; solid nodes are
// left untouched.

layout(local_size_x = 128, local_size_y = 1, local_size_z = 1) in;

layout(std430, binding = 2) writeonly buffer LbmDestination {
    float f_dst[];
};

void main() {
    uvec2 id = gl_GlobalInvocationID.xy;
    if (id.x >= u_nx || id.y >= u_ny) {
        return;
    }
    uint here = id.x + u_nx * id.y;
    if (solid[here] != 0u) {
        return;
    }
    float f[9];
    lbm_gather(int(id.x), int(id.y), f);
    lbm_collide(f);
    uint cells = lbm_cells();
    for (int i = 0; i < 9; ++i) {
        f_dst[uint(i) * cells + here] = f[i];
    }
}
