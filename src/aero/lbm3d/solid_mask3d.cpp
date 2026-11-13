#include "orrery/aero/lbm3d/solid_mask3d.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace orrery::aero::lbm {

namespace {

// a * b without wrapping, empty on overflow.
std::optional<std::size_t> checked_product(std::size_t a, std::size_t b) noexcept {
    if (a != 0 && b > std::numeric_limits<std::size_t>::max() / a) {
        return std::nullopt;
    }
    return a * b;
}

} // namespace

std::optional<std::size_t> lattice_cells_3d(std::size_t nx, std::size_t ny,
                                            std::size_t nz) noexcept {
    if (nx == 0 || ny == 0 || nz == 0) {
        return std::nullopt;
    }
    const std::optional<std::size_t> plane = checked_product(nx, ny);
    if (!plane) {
        return std::nullopt;
    }
    const std::optional<std::size_t> cells = checked_product(*plane, nz);
    if (!cells || *cells > geom::max_grid_cells) {
        return std::nullopt;
    }
    return cells;
}

std::size_t SolidMask3D::count() const noexcept {
    return static_cast<std::size_t>(
        std::count_if(solid.begin(), solid.end(), [](std::uint8_t s) { return s != 0; }));
}

SolidMask3D fluid_mask_3d(std::size_t nx, std::size_t ny, std::size_t nz) {
    const std::optional<std::size_t> cells = lattice_cells_3d(nx, ny, nz);
    if (!cells) {
        throw std::invalid_argument(
            "lbm3d: mask dimensions must be positive with at most geom::max_grid_cells nodes");
    }
    SolidMask3D mask;
    mask.nx = nx;
    mask.ny = ny;
    mask.nz = nz;
    mask.solid.assign(*cells, std::uint8_t{0});
    return mask;
}

SolidMask3D mask_from_sdf_3d(std::size_t nx, std::size_t ny, std::size_t nz,
                             const std::function<double(double, double, double)>& sdf) {
    SolidMask3D mask = fluid_mask_3d(nx, ny, nz);
    mask.distance.assign(mask.solid.size(), 0.0);
    for (std::size_t z = 0; z < nz; ++z) {
        for (std::size_t y = 0; y < ny; ++y) {
            for (std::size_t x = 0; x < nx; ++x) {
                const std::size_t c = mask.index(x, y, z);
                const double d =
                    sdf(static_cast<double>(x), static_cast<double>(y), static_cast<double>(z));
                mask.distance[c] = d;
                if (d <= 0.0) {
                    mask.solid[c] = 1;
                }
            }
        }
    }
    return mask;
}

SolidMask3D sphere_mask(std::size_t nx, std::size_t ny, std::size_t nz,
                        const std::array<double, 3>& centre, double radius) {
    return mask_from_sdf_3d(nx, ny, nz, [centre, radius](double x, double y, double z) {
        const double dx = x - centre[0];
        const double dy = y - centre[1];
        const double dz = z - centre[2];
        return std::sqrt(dx * dx + dy * dy + dz * dz) - radius;
    });
}

SolidMask3D box_mask(std::size_t nx, std::size_t ny, std::size_t nz,
                     const std::array<double, 3>& lo, const std::array<double, 3>& hi) {
    return mask_from_sdf_3d(nx, ny, nz, [lo, hi](double x, double y, double z) {
        // Signed distance of a box: per axis the excess over the half
        // extent around the centre.
        const std::array<double, 3> p{x, y, z};
        double outside = 0.0;
        double inside = -std::numeric_limits<double>::infinity();
        for (std::size_t a = 0; a < 3; ++a) {
            const double excess = std::abs(p[a] - 0.5 * (lo[a] + hi[a])) - 0.5 * (hi[a] - lo[a]);
            outside += std::max(excess, 0.0) * std::max(excess, 0.0);
            inside = std::max(inside, excess);
        }
        return std::sqrt(outside) + std::min(inside, 0.0);
    });
}

namespace {

// Checks a grid of `values` entries placed at `offset` in `mask`.
void check_placement(const SolidMask3D& mask, const geom::GridSpec& spec, std::size_t values,
                     const std::array<std::size_t, 3>& offset) {
    const std::optional<std::size_t> cells = lattice_cells_3d(spec.nx, spec.ny, spec.nz);
    if (!cells) {
        throw std::invalid_argument(
            "lbm3d: voxel grid dimensions must be positive with at most geom::max_grid_cells "
            "cells");
    }
    if (values != *cells) {
        throw std::invalid_argument("lbm3d: the voxel vector does not match its grid");
    }
    const std::optional<std::size_t> mask_cells = lattice_cells_3d(mask.nx, mask.ny, mask.nz);
    if (!mask_cells || mask.solid.size() != *mask_cells) {
        throw std::invalid_argument("lbm3d: the mask does not match its dimensions");
    }
    if (offset[0] > mask.nx || spec.nx > mask.nx - offset[0] || offset[1] > mask.ny ||
        spec.ny > mask.ny - offset[1] || offset[2] > mask.nz || spec.nz > mask.nz - offset[2]) {
        throw std::invalid_argument("lbm3d: the voxel grid does not fit in the lattice");
    }
}

} // namespace

void place_voxels(SolidMask3D& mask, const geom::VoxelGrid& voxels,
                  const std::array<std::size_t, 3>& offset) {
    const geom::GridSpec& spec = voxels.spec;
    check_placement(mask, spec, voxels.solid.size(), offset);
    for (std::size_t k = 0; k < spec.nz; ++k) {
        for (std::size_t j = 0; j < spec.ny; ++j) {
            for (std::size_t i = 0; i < spec.nx; ++i) {
                if (voxels.solid[spec.index(i, j, k)] != 0) {
                    mask.solid[mask.index(i + offset[0], j + offset[1], k + offset[2])] = 1;
                }
            }
        }
    }
}

void place_distance(SolidMask3D& mask, const geom::SignedDistanceField& sdf,
                    const std::array<std::size_t, 3>& offset) {
    const geom::GridSpec& spec = sdf.spec;
    check_placement(mask, spec, sdf.distance.size(), offset);
    if (!(spec.spacing > 0.0) || !std::isfinite(spec.spacing)) {
        throw std::invalid_argument("lbm3d: the distance grid needs a positive spacing");
    }
    if (mask.distance.size() != mask.solid.size()) {
        mask.distance.assign(mask.solid.size(), far_distance);
    }
    for (std::size_t k = 0; k < spec.nz; ++k) {
        for (std::size_t j = 0; j < spec.ny; ++j) {
            for (std::size_t i = 0; i < spec.nx; ++i) {
                mask.distance[mask.index(i + offset[0], j + offset[1], k + offset[2])] =
                    sdf.distance[spec.index(i, j, k)] / spec.spacing;
            }
        }
    }
}

} // namespace orrery::aero::lbm
