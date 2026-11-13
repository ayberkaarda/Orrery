#version 460 core
// One fused D3Q19 time step: pull-stream with boundaries, collide, then the
// absorbing layers (lbm_d3q19_common.glsl). Reads binding 0, writes binding
// 2; solid nodes are left untouched.

layout(local_size_x = 64, local_size_y = 1, local_size_z = 1) in;

layout(std430, binding = 2) writeonly buffer Lbm3Destination {
    float f_dst[];
};

void main() {
    uvec3 id = gl_GlobalInvocationID;
    if (id.x >= u_nx || id.y >= u_ny || id.z >= u_nz) {
        return;
    }
    uint here = id.x + u_nx * (id.y + u_ny * id.z);
    uint cls = node_class[here];
    if (cls == 0u) {
        return;
    }
    float f[19];
    lbm3_gather(int(id.x), int(id.y), int(id.z), cls, f);
    lbm3_collide(f);
    float s = lbm3_sponge(id.x, id.y, id.z);
    if (s > 0.0) {
        lbm3_absorb(f, s);
    }
    uint cells = lbm3_cells();
    for (int i = 0; i < 19; ++i) {
        f_dst[uint(i) * cells + here] = f[i];
    }
}
