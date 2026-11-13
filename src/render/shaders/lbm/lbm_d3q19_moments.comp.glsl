#version 460 core
// Macroscopic fields of the stored post-collision populations (binding 0):
// rho - 1, ux, uy and uz, the velocity with the Guo half-force correction
// (post-collision momentum is the pre-collision one plus F). Written to
// binding 3 as four planes of nx * ny * nz values; solid nodes report rest.

layout(local_size_x = 64, local_size_y = 1, local_size_z = 1) in;

layout(std430, binding = 3) writeonly buffer Lbm3Moments {
    float moments[];
};

void main() {
    uvec3 id = gl_GlobalInvocationID;
    if (id.x >= u_nx || id.y >= u_ny || id.z >= u_nz) {
        return;
    }
    uint here = id.x + u_nx * (id.y + u_ny * id.z);
    uint cells = lbm3_cells();
    float drho = 0.0;
    float ux = 0.0;
    float uy = 0.0;
    float uz = 0.0;
    if (node_class[here] != 0u) {
        float jx = 0.0;
        float jy = 0.0;
        float jz = 0.0;
        for (int i = 0; i < 19; ++i) {
            float f = lbm3_source(i, here);
            drho += f;
            jx += float(LBM3_CX[i]) * f;
            jy += float(LBM3_CY[i]) * f;
            jz += float(LBM3_CZ[i]) * f;
        }
        float rho = 1.0 + drho;
        ux = (jx - 0.5 * u_force_x) / rho;
        uy = (jy - 0.5 * u_force_y) / rho;
        uz = (jz - 0.5 * u_force_z) / rho;
    }
    moments[here] = drho;
    moments[cells + here] = ux;
    moments[2u * cells + here] = uy;
    moments[3u * cells + here] = uz;
}
