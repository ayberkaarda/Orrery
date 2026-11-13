#include "linear_solve.h"

// GCC 13 to 15 at -O3 report -Wnull-dereference inside Eigen's inlined LU,
// permutation and product kernels (an empty matrix has a null data pointer
// on paths the solver never takes: every system here has n >= 1). The
// warnings point into Eigen's headers, so they are silenced for this
// translation unit only, which holds all of the project's Eigen code.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic ignored "-Wnull-dereference"
#endif

#include <algorithm>
#include <cstddef>
#include <stdexcept>

#include <Eigen/Dense>
#include <unsupported/Eigen/IterativeSolvers>

namespace orrery::aero::detail {

namespace {

using ConstMatrixMap = Eigen::Map<const Eigen::MatrixXd>;
using ConstVectorMap = Eigen::Map<const Eigen::VectorXd>;

void check(const DenseSystem& system) {
    if (system.n == 0 || system.a.size() != system.n * system.n || system.b.size() != system.n) {
        throw std::invalid_argument("linear solve: inconsistent system size");
    }
}

LinearSolution finish(const DenseSystem& system, const Eigen::VectorXd& x, std::size_t iterations) {
    const auto n = static_cast<Eigen::Index>(system.n);
    const ConstMatrixMap a(system.a.data(), n, n);
    const ConstVectorMap b(system.b.data(), n);
    LinearSolution out;
    out.x.assign(x.data(), x.data() + x.size());
    out.residual = (a * x - b).norm() / std::max(b.norm(), 1e-300);
    out.iterations = iterations;
    return out;
}

} // namespace

LinearSolution solve_lu(const DenseSystem& system) {
    check(system);
    const auto n = static_cast<Eigen::Index>(system.n);
    const ConstMatrixMap a(system.a.data(), n, n);
    const ConstVectorMap b(system.b.data(), n);
    const Eigen::PartialPivLU<Eigen::MatrixXd> lu(a);
    const Eigen::VectorXd x = lu.solve(b);
    return finish(system, x, 1);
}

LinearSolution solve_gmres(const DenseSystem& system, const GmresSettings& settings) {
    check(system);
    const auto n = static_cast<Eigen::Index>(system.n);
    const Eigen::MatrixXd a = ConstMatrixMap(system.a.data(), n, n);
    const ConstVectorMap b(system.b.data(), n);
    Eigen::GMRES<Eigen::MatrixXd, Eigen::IdentityPreconditioner> gmres;
    gmres.set_restart(settings.restart);
    gmres.setTolerance(settings.tolerance);
    gmres.setMaxIterations(settings.max_iterations);
    gmres.compute(a);
    const Eigen::VectorXd x = gmres.solve(b);
    return finish(system, x, static_cast<std::size_t>(gmres.iterations()));
}

} // namespace orrery::aero::detail
