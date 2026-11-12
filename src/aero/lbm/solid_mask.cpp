#include "orrery/aero/lbm/solid_mask.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace orrery::aero::lbm {

std::size_t SolidMask2D::count() const noexcept {
    return static_cast<std::size_t>(
        std::count_if(solid.begin(), solid.end(), [](std::uint8_t s) { return s != 0; }));
}

SolidMask2D fluid_mask(std::size_t nx, std::size_t ny) {
    if (nx == 0 || ny == 0 || nx > geom::max_grid_cells / ny) {
        throw std::invalid_argument("solid mask: dimensions must be at least 1 and the node "
                                    "count at most geom::max_grid_cells");
    }
    SolidMask2D mask;
    mask.nx = nx;
    mask.ny = ny;
    mask.solid.assign(nx * ny, std::uint8_t{0});
    return mask;
}

SolidMask2D mask_from_sdf(std::size_t nx, std::size_t ny,
                          const std::function<double(double, double)>& sdf) {
    SolidMask2D mask = fluid_mask(nx, ny);
    for (std::size_t y = 0; y < ny; ++y) {
        for (std::size_t x = 0; x < nx; ++x) {
            if (sdf(static_cast<double>(x), static_cast<double>(y)) <= 0.0) {
                mask.solid[x + nx * y] = 1;
            }
        }
    }
    return mask;
}

SolidMask2D circle_mask(std::size_t nx, std::size_t ny, double cx, double cy, double radius) {
    return mask_from_sdf(nx, ny, [cx, cy, radius](double x, double y) {
        return std::hypot(x - cx, y - cy) - radius;
    });
}

SolidMask2D polygon_mask(std::size_t nx, std::size_t ny,
                         const std::vector<geom::SectionPoint>& polygon) {
    SolidMask2D mask = fluid_mask(nx, ny);
    const std::size_t n = polygon.size();
    if (n < 3) {
        return mask;
    }
    for (std::size_t y = 0; y < ny; ++y) {
        const auto py = static_cast<double>(y);
        for (std::size_t x = 0; x < nx; ++x) {
            const auto px = static_cast<double>(x);
            bool inside = false;
            for (std::size_t a = 0, b = n - 1; a < n; b = a++) {
                const geom::SectionPoint& pa = polygon[a];
                const geom::SectionPoint& pb = polygon[b];
                if ((pa.z > py) != (pb.z > py)) {
                    const double cross = pa.x + (py - pa.z) * (pb.x - pa.x) / (pb.z - pa.z);
                    if (px < cross) {
                        inside = !inside;
                    }
                }
            }
            if (inside) {
                mask.solid[x + nx * y] = 1;
            }
        }
    }
    return mask;
}

std::vector<geom::SectionPoint> place_section(const geom::Naca4& airfoil,
                                              const SectionPlacement& placement) {
    std::vector<geom::SectionPoint> points = airfoil.section(placement.points);
    const double c = std::cos(placement.angle_of_attack);
    const double s = std::sin(placement.angle_of_attack);
    for (geom::SectionPoint& p : points) {
        const double x = p.x * placement.chord;
        const double z = p.z * placement.chord;
        // Nose-up rotation about the leading edge for flow along +x.
        p = geom::SectionPoint{placement.leading_edge_x + x * c + z * s,
                               placement.leading_edge_y - x * s + z * c};
    }
    return points;
}

SolidMask2D naca_mask(std::size_t nx, std::size_t ny, const geom::Naca4& airfoil,
                      const SectionPlacement& placement) {
    return polygon_mask(nx, ny, place_section(airfoil, placement));
}

SolidMask2D mask_from_voxels(const geom::VoxelGrid& voxels, VoxelSlice plane, std::size_t index) {
    const geom::GridSpec& spec = voxels.spec;
    if (voxels.solid.size() != spec.cells() || spec.cells() == 0) {
        throw std::invalid_argument("solid mask: voxel vector does not match its grid spec");
    }
    const std::size_t limit = plane == VoxelSlice::xy ? spec.nz : spec.ny;
    if (index >= limit) {
        throw std::invalid_argument("solid mask: slice index outside the voxel grid");
    }
    const std::size_t ny = plane == VoxelSlice::xy ? spec.ny : spec.nz;
    SolidMask2D mask = fluid_mask(spec.nx, ny);
    for (std::size_t y = 0; y < ny; ++y) {
        for (std::size_t x = 0; x < spec.nx; ++x) {
            const std::size_t cell =
                plane == VoxelSlice::xy ? spec.index(x, y, index) : spec.index(x, index, y);
            mask.solid[x + spec.nx * y] = static_cast<std::uint8_t>(voxels.solid[cell] != 0);
        }
    }
    return mask;
}

} // namespace orrery::aero::lbm
