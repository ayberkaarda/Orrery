// D3Q19 lattice and the boundary rules of the pull step, shared by the
// lbm_d3q19_* kernels (the backend composes a "<family>_common" snippet only
// into the kernels of its family). Mirrors
// src/aero/lbm3d/include/orrery/aero/lbm3d/d3q19.h and
// src/aero/lbm3d/lbm3d_internal.h (resolve_source, velocity_inlet_west);
// keep them in step. Populations are stored shifted, fs_i = f_i - w_i, as
// structure of arrays: element i * cells + x + nx * (y + ny * z).

layout(std430, binding = 0) readonly buffer Lbm3Source {
    float f_src[];
};
// Node classes: 0 solid, 1 bulk fluid (plain streaming), 2 boundary fluid;
// 3 + s a boundary fluid node with wall fractions in slot s of binding 7
// (interpolated wall).
layout(std430, binding = 1) readonly buffer Lbm3Class {
    uint node_class[];
};
// Wall fraction q of the link in direction i of slot s: element 19 s + i.
layout(std430, binding = 7) readonly buffer Lbm3Wall {
    float wall_q[];
};
// Convective outlet history: previous post-streaming populations 2, 8, 10,
// 12, 14 entering the outlet face, element slot * ny * nz + y + ny * z
// (step kernel only).
layout(std430, binding = 6) buffer Lbm3Outlet {
    float outlet_history[];
};

uniform uint u_nx;
uniform uint u_ny;
uniform uint u_nz;
uniform uint u_x_mode;      // 0 periodic, 1 velocity inlet at x = 0 and outlet at nx - 1
uniform uint u_y_low_kind;  // 0 periodic, 1 free slip, 2 no-slip wall at y = -1/2
uniform uint u_y_high_kind; // same at y = ny - 1/2
uniform uint u_z_low_kind;  // same at z = -1/2
uniform uint u_z_high_kind; // same at z = nz - 1/2
uniform float u_y_low_ux;   // wall speeds along x (no-slip walls)
uniform float u_y_high_ux;
uniform float u_z_low_ux;
uniform float u_z_high_ux;
uniform float u_inlet_ux;
uniform float u_inlet_uy;
uniform float u_inlet_uz;
uniform float u_tau;
uniform float u_smagorinsky_c2; // C^2, 0 when the closure is off
uniform float u_force_x;        // body force density (Guo forcing)
uniform float u_force_y;
uniform float u_force_z;
uniform uint u_outlet_mode;     // 0 convective, 1 zero-gradient extrapolation
uniform float u_outlet_speed;   // convection speed U of the convective outlet
uniform float u_sponge_strength; // absorbing layers (Sponge), 0 when off
uniform uint u_sponge_inlet;     // layer thicknesses, nodes
uniform uint u_sponge_outlet;
uniform uint u_sponge_sides;

const int LBM3_CX[19] = int[19](0, 1, -1, 0, 0, 0, 0, 1, -1, 1, -1, 1, -1, 1, -1, 0, 0, 0, 0);
const int LBM3_CY[19] = int[19](0, 0, 0, 1, -1, 0, 0, 1, -1, -1, 1, 0, 0, 0, 0, 1, -1, 1, -1);
const int LBM3_CZ[19] = int[19](0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 1, -1, -1, 1, 1, -1, -1, 1);
const float LBM3_W[19] = float[19](1.0 / 3.0, 1.0 / 18.0, 1.0 / 18.0, 1.0 / 18.0, 1.0 / 18.0,
                                   1.0 / 18.0, 1.0 / 18.0, 1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0,
                                   1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0,
                                   1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0);
const int LBM3_OPPOSITE[19] = int[19](0, 2, 1, 4, 3, 6, 5, 8, 7, 10, 9, 12, 11, 14, 13, 16, 15, 18, 17);
const int LBM3_MIRROR_Y[19] = int[19](0, 1, 2, 4, 3, 5, 6, 9, 10, 7, 8, 11, 12, 13, 14, 18, 17, 16, 15);
const int LBM3_MIRROR_Z[19] = int[19](0, 1, 2, 3, 4, 6, 5, 7, 8, 9, 10, 13, 14, 11, 12, 17, 18, 15, 16);

uint lbm3_cells() {
    return u_nx * u_ny * u_nz;
}

uint lbm3_index(int x, int y, int z) {
    return uint(x) + u_nx * (uint(y) + u_ny * uint(z));
}

float lbm3_source(int i, uint cell) {
    return f_src[uint(i) * lbm3_cells() + cell];
}

// Slot of an outlet population (2, 8, 10, 12, 14) in the outlet history.
uint lbm3_outlet_slot(int i) {
    return i == 2 ? 0u : (i == 8 ? 1u : (i == 10 ? 2u : (i == 12 ? 3u : 4u)));
}

// Rule of one transverse side when the source coordinate s leaves [0, n):
// periodic wrap, specular reflection (s stays at the node's own coordinate,
// the direction is mirrored), or a no-slip wall (returns true, s unchanged).
bool lbm3_side(uint low_kind, uint high_kind, inout int s, int n, int own, inout int direction,
               bool y_axis) {
    if (s >= 0 && s < n) {
        return false;
    }
    uint kind = s < 0 ? low_kind : high_kind;
    if (kind == 0u) {
        s = (s + n) % n;
        return false;
    }
    if (kind == 1u) {
        s = own;
        direction = y_axis ? LBM3_MIRROR_Y[direction] : LBM3_MIRROR_Z[direction];
        return false;
    }
    return true;
}

// Velocity inlet on the west face in shifted form: the weights of the known
// populations (c_x = 0 once, c_x = -1 twice) add up to 1, so
// rho - 1 = (S + ux) / (1 - ux) with S the same sum of shifted values; the
// unknown populations come from bounce-back of their non-equilibrium part
// (w_i = w_opp(i), so the shifted form is the same), and the node is then
// regularized: f = f_eq + w / (2 c_s^4) (c c - c_s^2 I) : Pi_neq.
void lbm3_inlet_west(inout float f[19]) {
    float ux = u_inlet_ux;
    float uy = u_inlet_uy;
    float uz = u_inlet_uz;
    float s = 0.0;
    for (int i = 0; i < 19; ++i) {
        if (LBM3_CX[i] == 0) {
            s += f[i];
        } else if (LBM3_CX[i] < 0) {
            s += 2.0 * f[i];
        }
    }
    float drho = (s + ux) / (1.0 - ux);
    float rho = 1.0 + drho;
    for (int i = 0; i < 19; ++i) {
        if (LBM3_CX[i] > 0) {
            float cu = float(LBM3_CX[i]) * ux + float(LBM3_CY[i]) * uy + float(LBM3_CZ[i]) * uz;
            f[i] = f[LBM3_OPPOSITE[i]] + 6.0 * LBM3_W[i] * rho * cu;
        }
    }
    float uu = ux * ux + uy * uy + uz * uz;
    float feq[19];
    float pxx = 0.0;
    float pyy = 0.0;
    float pzz = 0.0;
    float pxy = 0.0;
    float pxz = 0.0;
    float pyz = 0.0;
    for (int i = 0; i < 19; ++i) {
        float ex = float(LBM3_CX[i]);
        float ey = float(LBM3_CY[i]);
        float ez = float(LBM3_CZ[i]);
        float cu = ex * ux + ey * uy + ez * uz;
        feq[i] = LBM3_W[i] * (drho + rho * (3.0 * cu + 4.5 * cu * cu - 1.5 * uu));
        float neq = f[i] - feq[i];
        pxx += ex * ex * neq;
        pyy += ey * ey * neq;
        pzz += ez * ez * neq;
        pxy += ex * ey * neq;
        pxz += ex * ez * neq;
        pyz += ey * ez * neq;
    }
    for (int i = 0; i < 19; ++i) {
        float ex = float(LBM3_CX[i]);
        float ey = float(LBM3_CY[i]);
        float ez = float(LBM3_CZ[i]);
        f[i] = feq[i] + 4.5 * LBM3_W[i] *
                            ((ex * ex - 1.0 / 3.0) * pxx + (ey * ey - 1.0 / 3.0) * pyy +
                             (ez * ez - 1.0 / 3.0) * pzz +
                             2.0 * (ex * ey * pxy + ex * ez * pxz + ey * ez * pyz));
    }
}

// The population node `here` = (x, y, z) receives from the wall of its link
// in direction i (source solid): population o = opposite(i) of the node for
// the halfway wall (q = 1/2), else the interpolated bounce-back (Bouzidi et
// al. 2001; a link with q < 1/2 always has a fluid node at x + c_i). The
// weights of the combinations add up to 1 and w_o = w_i, so the shifted form
// is the same.
float lbm3_reflected(int i, uint here, int x, int y, int z, float q) {
    int o = LBM3_OPPOSITE[i];
    float leaving = lbm3_source(o, here);
    if (q == 0.5) {
        return leaving;
    }
    if (q < 0.5) {
        uint farther = lbm3_index(x + LBM3_CX[i], y + LBM3_CY[i], z + LBM3_CZ[i]);
        return 2.0 * q * leaving + (1.0 - 2.0 * q) * lbm3_source(o, farther);
    }
    return (leaving + (2.0 * q - 1.0) * lbm3_source(i, here)) / (2.0 * q);
}

float lbm3_wall_fraction(uint cls, int i) {
    return cls >= 3u ? wall_q[(cls - 3u) * 19u + uint(i)] : 0.5;
}

// Post-streaming shifted populations of the fluid node (x, y, z) of class
// `cls` (1 bulk, 2 or more boundary).
void lbm3_gather(int x, int y, int z, uint cls, out float f[19]) {
    uint here = lbm3_index(x, y, z);
    int nx = int(u_nx);
    int ny = int(u_ny);
    int nz = int(u_nz);
    if (cls == 1u) {
        int plane = nx * ny;
        for (int i = 0; i < 19; ++i) {
            int offset = LBM3_CX[i] + nx * LBM3_CY[i] + plane * LBM3_CZ[i];
            f[i] = lbm3_source(i, uint(int(here) - offset));
        }
        return;
    }
    // Density of this node at the previous step (collision keeps it): the
    // wall density of the moving-wall term, summed on first use.
    float wall_rho = -1.0;
    for (int i = 0; i < 19; ++i) {
        int sx = x - LBM3_CX[i];
        int sy = y - LBM3_CY[i];
        int sz = z - LBM3_CZ[i];
        int direction = i;
        bool wall = false;
        float wall_ux = 0.0;
        if (lbm3_side(u_y_low_kind, u_y_high_kind, sy, ny, y, direction, true)) {
            wall = true;
            wall_ux = sy < 0 ? u_y_low_ux : u_y_high_ux;
        } else if (lbm3_side(u_z_low_kind, u_z_high_kind, sz, nz, z, direction, false)) {
            wall = true;
            wall_ux = sz < 0 ? u_z_low_ux : u_z_high_ux;
        }
        if (wall) {
            if (wall_rho < 0.0) {
                float drho = 0.0;
                for (int k = 0; k < 19; ++k) {
                    drho += lbm3_source(k, here);
                }
                wall_rho = 1.0 + drho;
            }
            f[i] = lbm3_source(LBM3_OPPOSITE[i], here) +
                   6.0 * LBM3_W[i] * wall_rho * float(LBM3_CX[i]) * wall_ux;
            continue;
        }
        if (sx < 0 || sx >= nx) {
            if (u_x_mode == 0u) {
                sx = (sx + nx) % nx;
            } else if (sx < 0) {
                f[i] = 0.0; // reconstructed by the inlet below
                continue;
            } else {
                // Outlet: the zero-gradient value, blended with the node's
                // history by the convective outlet.
                float upstream = lbm3_source(direction, lbm3_index(nx - 1, sy, sz));
                if (u_outlet_mode == 0u) {
                    uint slot = lbm3_outlet_slot(i) * u_ny * u_nz + uint(y) + u_ny * uint(z);
                    upstream = (outlet_history[slot] + u_outlet_speed * upstream) /
                               (1.0 + u_outlet_speed);
                    outlet_history[slot] = upstream;
                }
                f[i] = upstream;
                continue;
            }
        }
        uint source = lbm3_index(sx, sy, sz);
        f[i] = node_class[source] == 0u
                   ? lbm3_reflected(i, here, x, y, z, lbm3_wall_fraction(cls, i))
                   : lbm3_source(direction, source);
    }
    if (u_x_mode == 1u && x == 0) {
        lbm3_inlet_west(f);
    }
}

// BGK collision with Guo forcing and the optional Smagorinsky closure, on
// shifted populations: f_eq,i - w_i = w_i (drho + rho (3 c.u + 4.5 (c.u)^2
// - 1.5 u.u)).
void lbm3_collide(inout float f[19]) {
    float drho = 0.0;
    float jx = 0.0;
    float jy = 0.0;
    float jz = 0.0;
    for (int i = 0; i < 19; ++i) {
        drho += f[i];
        jx += float(LBM3_CX[i]) * f[i];
        jy += float(LBM3_CY[i]) * f[i];
        jz += float(LBM3_CZ[i]) * f[i];
    }
    float rho = 1.0 + drho;
    float ux = (jx + 0.5 * u_force_x) / rho;
    float uy = (jy + 0.5 * u_force_y) / rho;
    float uz = (jz + 0.5 * u_force_z) / rho;
    float uu = ux * ux + uy * uy + uz * uz;

    float feq[19];
    for (int i = 0; i < 19; ++i) {
        float cu = float(LBM3_CX[i]) * ux + float(LBM3_CY[i]) * uy + float(LBM3_CZ[i]) * uz;
        feq[i] = LBM3_W[i] * (drho + rho * (3.0 * cu + 4.5 * cu * cu - 1.5 * uu));
    }
    float tau = u_tau;
    if (u_smagorinsky_c2 > 0.0) {
        float pxx = 0.0;
        float pyy = 0.0;
        float pzz = 0.0;
        float pxy = 0.0;
        float pxz = 0.0;
        float pyz = 0.0;
        for (int i = 0; i < 19; ++i) {
            float ex = float(LBM3_CX[i]);
            float ey = float(LBM3_CY[i]);
            float ez = float(LBM3_CZ[i]);
            float neq = f[i] - feq[i];
            pxx += ex * ex * neq;
            pyy += ey * ey * neq;
            pzz += ez * ez * neq;
            pxy += ex * ey * neq;
            pxz += ex * ez * neq;
            pyz += ey * ez * neq;
        }
        float strain = sqrt(pxx * pxx + pyy * pyy + pzz * pzz +
                            2.0 * (pxy * pxy + pxz * pxz + pyz * pyz));
        tau = 0.5 * (tau + sqrt(tau * tau + 25.455844122715710 * u_smagorinsky_c2 * strain / rho));
    }
    float omega = 1.0 / tau;
    float force_scale = 1.0 - 0.5 * omega;
    bool forced = u_force_x != 0.0 || u_force_y != 0.0 || u_force_z != 0.0;
    for (int i = 0; i < 19; ++i) {
        float value = f[i] - omega * (f[i] - feq[i]);
        if (forced) {
            float ex = float(LBM3_CX[i]);
            float ey = float(LBM3_CY[i]);
            float ez = float(LBM3_CZ[i]);
            float cu = ex * ux + ey * uy + ez * uz;
            value += force_scale * LBM3_W[i] *
                     (3.0 * ((ex - ux) * u_force_x + (ey - uy) * u_force_y + (ez - uz) * u_force_z) +
                      9.0 * cu * (ex * u_force_x + ey * u_force_y + ez * u_force_z));
        }
        f[i] = value;
    }
}

float lbm3_sponge_layer(uint distance, uint thickness) {
    if (distance < thickness) {
        float r = float(thickness - distance) / float(thickness);
        return r * r;
    }
    return 0.0;
}

// Relaxation rate of the absorbing layers at node (x, y, z), as
// sponge_sigma of lbm3d_internal.h.
float lbm3_sponge(uint x, uint y, uint z) {
    if (!(u_sponge_strength > 0.0) || u_x_mode != 1u) {
        return 0.0;
    }
    float best = max(lbm3_sponge_layer(x, u_sponge_inlet),
                     lbm3_sponge_layer(u_nx - 1u - x, u_sponge_outlet));
    if (u_y_low_kind != 0u) {
        best = max(best, max(lbm3_sponge_layer(y, u_sponge_sides),
                             lbm3_sponge_layer(u_ny - 1u - y, u_sponge_sides)));
    }
    if (u_z_low_kind != 0u) {
        best = max(best, max(lbm3_sponge_layer(z, u_sponge_sides),
                             lbm3_sponge_layer(u_nz - 1u - z, u_sponge_sides)));
    }
    return u_sponge_strength * best;
}

// Absorbing-layer relaxation of rate s towards the free-stream equilibrium
// (density 1, the inlet velocity), shifted: f_eq - w = w (3 c.u + 4.5 (c.u)^2
// - 1.5 u.u).
void lbm3_absorb(inout float f[19], float s) {
    float ux = u_inlet_ux;
    float uy = u_inlet_uy;
    float uz = u_inlet_uz;
    float uu = ux * ux + uy * uy + uz * uz;
    for (int i = 0; i < 19; ++i) {
        float cu = float(LBM3_CX[i]) * ux + float(LBM3_CY[i]) * uy + float(LBM3_CZ[i]) * uz;
        float target = LBM3_W[i] * (3.0 * cu + 4.5 * cu * cu - 1.5 * uu);
        f[i] -= s * (f[i] - target);
    }
}
