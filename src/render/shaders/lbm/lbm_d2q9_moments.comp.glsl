#version 460 core
// Macroscopic fields of the stored post-collision populations (binding 0):
// rho - 1, ux and uy, the velocity with the Guo half-force correction
// (post-collision momentum is the pre-collision one plus F). Written to
// binding 3 as three planes of nx * ny values; solid nodes report rest.

layout(local_size_x = 128, local_size_y = 1, local_size_z = 1) in;

layout(std430, binding = 3) writeonly buffer LbmMoments {
    float moments[];
};

void main() {
    uvec2 id = gl_GlobalInvocationID.xy;
    if (id.x >= u_nx || id.y >= u_ny) {
        return;
    }
    uint here = id.x + u_nx * id.y;
    uint cells = lbm_cells();
    float drho = 0.0;
    float ux = 0.0;
    float uy = 0.0;
    if (solid[here] == 0u) {
        float jx = 0.0;
        float jy = 0.0;
        for (int i = 0; i < 9; ++i) {
            float f = lbm_source(i, here);
            drho += f;
            jx += float(LBM_CX[i]) * f;
            jy += float(LBM_CY[i]) * f;
        }
        float rho = 1.0 + drho;
        ux = (jx - 0.5 * u_force_x) / rho;
        uy = (jy - 0.5 * u_force_y) / rho;
    }
    moments[here] = drho;
    moments[cells + here] = ux;
    moments[2u * cells + here] = uy;
}
