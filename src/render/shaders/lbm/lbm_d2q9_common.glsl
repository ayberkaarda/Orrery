// D2Q9 lattice and the boundary rules of the pull step, shared by the LBM
// kernels. Mirrors src/aero/lbm/include/orrery/aero/lbm/d2q9.h and
// src/aero/lbm/lbm2d_internal.h (resolve_source, zou_he_west); keep them in
// step. Populations are stored shifted, fs_i = f_i - w_i, as structure of
// arrays: element i * cells + x + nx * y.

layout(std430, binding = 0) readonly buffer LbmSource {
    float f_src[];
};
layout(std430, binding = 1) readonly buffer LbmSolid {
    uint solid[];
};
// Convective outlet history: previous post-streaming populations 3, 6, 7 of
// the outlet column, element slot * ny + y (step kernel only).
layout(std430, binding = 6) buffer LbmOutlet {
    float outlet_history[];
};

uniform uint u_nx;
uniform uint u_ny;
uniform uint u_x_mode;      // 0 periodic, 1 Zou-He inlet at x = 0 and extrapolation outlet
uniform uint u_bottom_kind; // 0 periodic, 1 free slip, 2 no-slip wall at y = -1/2
uniform uint u_top_kind;    // same at y = ny - 1/2
uniform float u_bottom_ux;  // tangential wall speeds (no-slip)
uniform float u_top_ux;
uniform float u_inlet_ux;
uniform float u_inlet_uy;
uniform float u_tau;
uniform float u_smagorinsky_c2; // C^2, 0 when the closure is off
uniform float u_force_x;        // body force density (Guo forcing)
uniform float u_force_y;
uniform uint u_outlet_mode;    // 0 convective, 1 zero-gradient extrapolation
uniform float u_outlet_speed;  // convection speed U of the convective outlet

const int LBM_CX[9] = int[9](0, 1, 0, -1, 0, 1, -1, -1, 1);
const int LBM_CY[9] = int[9](0, 0, 1, 0, -1, 1, 1, -1, -1);
const float LBM_W[9] = float[9](4.0 / 9.0, 1.0 / 9.0, 1.0 / 9.0, 1.0 / 9.0, 1.0 / 9.0,
                                1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0);
const int LBM_OPPOSITE[9] = int[9](0, 3, 4, 1, 2, 7, 8, 5, 6);
const int LBM_MIRROR_Y[9] = int[9](0, 1, 4, 3, 2, 8, 7, 6, 5);

uint lbm_cells() {
    return u_nx * u_ny;
}

float lbm_source(int i, uint cell) {
    return f_src[uint(i) * lbm_cells() + cell];
}

// Zou-He velocity inlet on the west face in shifted form: the weights of the
// known populations f0 + f2 + f4 + 2 (f3 + f6 + f7) add up to 1, so
// rho - 1 = (S + ux) / (1 - ux) with S the same sum of shifted values, and
// the reconstruction keeps its form because w1 = w3, w5 = w7, w8 = w6. The
// node is then regularized: f = f_eq + w / (2 c_s^4) (c c - c_s^2 I) : Pi_neq.
void lbm_zou_he_west(inout float f[9]) {
    float ux = u_inlet_ux;
    float uy = u_inlet_uy;
    float s = f[0] + f[2] + f[4] + 2.0 * (f[3] + f[6] + f[7]);
    float drho = (s + ux) / (1.0 - ux);
    float rho = 1.0 + drho;
    f[1] = f[3] + (2.0 / 3.0) * rho * ux;
    f[5] = f[7] - 0.5 * (f[2] - f[4]) + (1.0 / 6.0) * rho * ux + 0.5 * rho * uy;
    f[8] = f[6] + 0.5 * (f[2] - f[4]) + (1.0 / 6.0) * rho * ux - 0.5 * rho * uy;

    float uu = ux * ux + uy * uy;
    float feq[9];
    float pxx = 0.0;
    float pyy = 0.0;
    float pxy = 0.0;
    for (int i = 0; i < 9; ++i) {
        float ex = float(LBM_CX[i]);
        float ey = float(LBM_CY[i]);
        float cu = ex * ux + ey * uy;
        feq[i] = LBM_W[i] * (drho + rho * (3.0 * cu + 4.5 * cu * cu - 1.5 * uu));
        float neq = f[i] - feq[i];
        pxx += ex * ex * neq;
        pyy += ey * ey * neq;
        pxy += ex * ey * neq;
    }
    for (int i = 0; i < 9; ++i) {
        float ex = float(LBM_CX[i]);
        float ey = float(LBM_CY[i]);
        f[i] = feq[i] + 4.5 * LBM_W[i] *
                            ((ex * ex - 1.0 / 3.0) * pxx + (ey * ey - 1.0 / 3.0) * pyy +
                             2.0 * ex * ey * pxy);
    }
}

// Post-streaming shifted populations of the fluid node (x, y).
void lbm_gather(int x, int y, out float f[9]) {
    int nx = int(u_nx);
    int ny = int(u_ny);
    uint here = uint(x) + u_nx * uint(y);
    bool inner = x > 0 && x < nx - 1 && y > 0 && y < ny - 1;
    // Density of this node at the previous step (collision keeps it): the
    // wall density of the moving-wall term, summed on first use.
    float wall_rho = -1.0;
    for (int i = 0; i < 9; ++i) {
        int sx = x - LBM_CX[i];
        int sy = y - LBM_CY[i];
        int direction = i;
        bool resolved = false;
        if (!inner) {
            if (sy < 0 || sy >= ny) {
                uint kind = sy < 0 ? u_bottom_kind : u_top_kind;
                if (kind == 0u) {
                    sy = (sy + ny) % ny;
                } else if (kind == 1u) {
                    sy = y;
                    direction = LBM_MIRROR_Y[i];
                } else {
                    float wall_ux = sy < 0 ? u_bottom_ux : u_top_ux;
                    if (wall_rho < 0.0) {
                        float drho = 0.0;
                        for (int k = 0; k < 9; ++k) {
                            drho += lbm_source(k, here);
                        }
                        wall_rho = 1.0 + drho;
                    }
                    f[i] = lbm_source(LBM_OPPOSITE[i], here) +
                           6.0 * LBM_W[i] * wall_rho * float(LBM_CX[i]) * wall_ux;
                    resolved = true;
                }
            }
            if (!resolved && (sx < 0 || sx >= nx)) {
                if (u_x_mode == 0u) {
                    sx = (sx + nx) % nx;
                } else if (sx < 0) {
                    f[i] = 0.0; // reconstructed by the inlet below
                    resolved = true;
                } else {
                    // Outlet: the zero-gradient value, blended with the
                    // node's history by the convective outlet.
                    uint column = u_nx - 1u + u_nx * uint(sy);
                    float upstream = lbm_source(direction, column);
                    if (u_outlet_mode == 0u) {
                        uint slot = uint(i == 3 ? 0 : (i == 6 ? 1 : 2)) * u_ny + uint(y);
                        upstream = (outlet_history[slot] + u_outlet_speed * upstream) /
                                   (1.0 + u_outlet_speed);
                        outlet_history[slot] = upstream;
                    }
                    f[i] = upstream;
                    resolved = true;
                }
            }
        }
        if (!resolved) {
            uint source = uint(sx) + u_nx * uint(sy);
            f[i] = solid[source] != 0u ? lbm_source(LBM_OPPOSITE[i], here)
                                       : lbm_source(direction, source);
        }
    }
    if (u_x_mode == 1u && x == 0) {
        lbm_zou_he_west(f);
    }
}

// BGK collision with Guo forcing and the optional Smagorinsky closure, on
// shifted populations: f_eq,i - w_i = w_i (drho + rho (3 c.u + 4.5 (c.u)^2
// - 1.5 u.u)).
void lbm_collide(inout float f[9]) {
    float drho = 0.0;
    float jx = 0.0;
    float jy = 0.0;
    for (int i = 0; i < 9; ++i) {
        drho += f[i];
        jx += float(LBM_CX[i]) * f[i];
        jy += float(LBM_CY[i]) * f[i];
    }
    float rho = 1.0 + drho;
    float ux = (jx + 0.5 * u_force_x) / rho;
    float uy = (jy + 0.5 * u_force_y) / rho;
    float uu = ux * ux + uy * uy;

    float feq[9];
    float pxx = 0.0;
    float pyy = 0.0;
    float pxy = 0.0;
    for (int i = 0; i < 9; ++i) {
        float ex = float(LBM_CX[i]);
        float ey = float(LBM_CY[i]);
        float cu = ex * ux + ey * uy;
        feq[i] = LBM_W[i] * (drho + rho * (3.0 * cu + 4.5 * cu * cu - 1.5 * uu));
        float neq = f[i] - feq[i];
        pxx += ex * ex * neq;
        pyy += ey * ey * neq;
        pxy += ex * ey * neq;
    }
    float tau = u_tau;
    if (u_smagorinsky_c2 > 0.0) {
        float strain = sqrt(pxx * pxx + pyy * pyy + 2.0 * pxy * pxy);
        tau = 0.5 * (tau + sqrt(tau * tau + 25.455844122715710 * u_smagorinsky_c2 * strain / rho));
    }
    float omega = 1.0 / tau;
    float force_scale = 1.0 - 0.5 * omega;
    bool forced = u_force_x != 0.0 || u_force_y != 0.0;
    for (int i = 0; i < 9; ++i) {
        float value = f[i] - omega * (f[i] - feq[i]);
        if (forced) {
            float ex = float(LBM_CX[i]);
            float ey = float(LBM_CY[i]);
            float cu = ex * ux + ey * uy;
            value += force_scale * LBM_W[i] *
                     (3.0 * ((ex - ux) * u_force_x + (ey - uy) * u_force_y) +
                      9.0 * cu * (ex * u_force_x + ey * u_force_y));
        }
        f[i] = value;
    }
}
