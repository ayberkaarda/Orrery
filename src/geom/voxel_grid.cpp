#include "orrery/geom/voxel_grid.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "orrery/geom/winding_number.h"

namespace orrery::geom {

Vector3 GridSpec::centre(std::size_t i, std::size_t j, std::size_t k) const noexcept {
    return origin + Vector3{(static_cast<double>(i) + 0.5) * spacing,
                            (static_cast<double>(j) + 0.5) * spacing,
                            (static_cast<double>(k) + 0.5) * spacing};
}

std::size_t VoxelGrid::solid_count() const noexcept {
    return static_cast<std::size_t>(std::count(solid.begin(), solid.end(), std::uint8_t{1}));
}

GridSpec grid_around(const Bounds& box, double spacing, std::size_t padding) {
    if (!std::isfinite(spacing) || !(spacing > 0.0)) {
        throw std::invalid_argument("grid_around: spacing must be positive and finite");
    }
    const Vector3 e = box.extent();
    auto cells = [&](double extent) {
        const double n = std::ceil(extent / spacing) + 2.0 * static_cast<double>(padding);
        if (!std::isfinite(n) || n > static_cast<double>(max_grid_cells)) {
            throw std::invalid_argument("grid_around: grid too large");
        }
        return std::max<std::size_t>(1, static_cast<std::size_t>(n));
    };
    GridSpec g;
    g.spacing = spacing;
    g.nx = cells(e.x);
    g.ny = cells(e.y);
    g.nz = cells(e.z);
    if (g.nx > max_grid_cells / g.ny || g.nx * g.ny > max_grid_cells / g.nz) {
        throw std::invalid_argument("grid_around: grid too large");
    }
    // Centre the grid on the box.
    const Vector3 size{static_cast<double>(g.nx) * spacing, static_cast<double>(g.ny) * spacing,
                       static_cast<double>(g.nz) * spacing};
    g.origin = box.centre() - size * 0.5;
    return g;
}

VoxelGrid voxelize(const TriangleMesh& mesh, const GridSpec& spec, double threshold) {
    if (spec.nx == 0 || spec.ny == 0 || spec.nz == 0 || spec.nx > max_grid_cells / spec.ny ||
        spec.nx * spec.ny > max_grid_cells / spec.nz) {
        throw std::invalid_argument("voxelize: grid empty or too large");
    }
    const MeshTree tree(mesh);
    VoxelGrid grid;
    grid.spec = spec;
    grid.solid.assign(spec.cells(), 0);
    grid.winding.assign(spec.cells(), 0.0);
    for (std::size_t k = 0; k < spec.nz; ++k) {
        for (std::size_t j = 0; j < spec.ny; ++j) {
            for (std::size_t i = 0; i < spec.nx; ++i) {
                const std::size_t c = spec.index(i, j, k);
                const double w = tree.winding_number(spec.centre(i, j, k));
                grid.winding[c] = w;
                grid.solid[c] = static_cast<std::uint8_t>(w >= threshold);
            }
        }
    }
    return grid;
}

SignedDistanceField signed_distance_band(const TriangleMesh& mesh, const VoxelGrid& voxels,
                                         double band_cells) {
    if (!std::isfinite(band_cells) || !(band_cells > 0.0)) {
        throw std::invalid_argument("signed_distance_band: band must be positive");
    }
    const GridSpec& spec = voxels.spec;
    if (voxels.solid.size() != spec.cells()) {
        throw std::invalid_argument("signed_distance_band: voxel grid does not match its spec");
    }
    const MeshTree tree(mesh);
    SignedDistanceField sdf;
    sdf.spec = spec;
    sdf.band = band_cells * spec.spacing;
    sdf.distance.assign(spec.cells(), 0.0);
    sdf.in_band.assign(spec.cells(), 0);
    for (std::size_t k = 0; k < spec.nz; ++k) {
        for (std::size_t j = 0; j < spec.ny; ++j) {
            for (std::size_t i = 0; i < spec.nx; ++i) {
                const std::size_t c = spec.index(i, j, k);
                const double sign = voxels.solid[c] != 0 ? -1.0 : 1.0;
                const double d = tree.distance(spec.centre(i, j, k), sdf.band);
                sdf.in_band[c] = static_cast<std::uint8_t>(d < sdf.band);
                sdf.distance[c] = sign * d;
            }
        }
    }
    return sdf;
}

} // namespace orrery::geom
