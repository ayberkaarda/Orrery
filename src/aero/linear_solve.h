#pragma once

#include <cstddef>
#include <vector>

namespace orrery::aero::detail {

// Dense n x n system A x = b, A stored column-major (A(i, j) = a[j * n + i]).
struct DenseSystem {
    std::size_t n = 0;
    std::vector<double> a;
    std::vector<double> b;

    explicit DenseSystem(std::size_t size) : n(size), a(size * size, 0.0), b(size, 0.0) {}

    [[nodiscard]] double& at(std::size_t row, std::size_t col) noexcept { return a[col * n + row]; }
};

struct GmresSettings {
    int restart = 100;
    double tolerance = 1e-10;
    int max_iterations = 2000;
};

struct LinearSolution {
    std::vector<double> x;
    // ||A x - b|| / ||b||.
    double residual = 0.0;
    std::size_t iterations = 0;
};

// Partial-pivot LU (Eigen).
[[nodiscard]] LinearSolution solve_lu(const DenseSystem& system);

// Restarted GMRES without preconditioner (Eigen unsupported IterativeSolvers).
[[nodiscard]] LinearSolution solve_gmres(const DenseSystem& system, const GmresSettings& settings);

} // namespace orrery::aero::detail
