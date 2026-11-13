// The D3Q19 lattice: weights, symmetry tables and the isotropy of its
// moments up to fourth order (Kruger et al., The Lattice Boltzmann Method,
// 2017, section 3.4), and the equilibrium's moments.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <cstddef>

#include "orrery/aero/lbm3d/d3q19.h"

using namespace orrery::aero::lbm;
using Catch::Approx;

namespace {

double c(std::size_t i, std::size_t axis) {
    const int v = axis == 0 ? d3q19::cx[i] : (axis == 1 ? d3q19::cy[i] : d3q19::cz[i]);
    return static_cast<double>(v);
}

double delta(std::size_t a, std::size_t b) {
    return a == b ? 1.0 : 0.0;
}

} // namespace

TEST_CASE("D3Q19 weights and velocity moments are isotropic", "[aero][lbm3d]") {
    double sum = 0.0;
    for (const double w : d3q19::w) {
        sum += w;
    }
    CHECK(sum == Approx(1.0).epsilon(1e-15));
    for (std::size_t a = 0; a < 3; ++a) {
        double first = 0.0;
        for (std::size_t i = 0; i < d3q19::q; ++i) {
            first += d3q19::w[i] * c(i, a);
        }
        CHECK(std::abs(first) < 1e-15);
        for (std::size_t b = 0; b < 3; ++b) {
            double second = 0.0;
            double third = 0.0;
            for (std::size_t i = 0; i < d3q19::q; ++i) {
                second += d3q19::w[i] * c(i, a) * c(i, b);
                third += d3q19::w[i] * c(i, a) * c(i, b) * c(i, (a + b) % 3);
            }
            CHECK(second == Approx(d3q19::cs2 * delta(a, b)).margin(1e-15));
            CHECK(std::abs(third) < 1e-15);
            for (std::size_t g = 0; g < 3; ++g) {
                for (std::size_t d = 0; d < 3; ++d) {
                    double fourth = 0.0;
                    for (std::size_t i = 0; i < d3q19::q; ++i) {
                        fourth += d3q19::w[i] * c(i, a) * c(i, b) * c(i, g) * c(i, d);
                    }
                    const double expected = d3q19::cs2 * d3q19::cs2 *
                                            (delta(a, b) * delta(g, d) + delta(a, g) * delta(b, d) +
                                             delta(a, d) * delta(b, g));
                    CHECK(fourth == Approx(expected).margin(1e-15));
                }
            }
        }
    }
}

TEST_CASE("D3Q19 opposite and mirror tables", "[aero][lbm3d]") {
    for (std::size_t i = 0; i < d3q19::q; ++i) {
        const std::size_t o = d3q19::opposite[i];
        CHECK(d3q19::opposite[o] == i);
        CHECK(d3q19::w[o] == d3q19::w[i]);
        for (std::size_t a = 0; a < 3; ++a) {
            CHECK(c(o, a) == -c(i, a));
        }
        const std::size_t my = d3q19::mirror_y[i];
        CHECK(c(my, 0) == c(i, 0));
        CHECK(c(my, 1) == -c(i, 1));
        CHECK(c(my, 2) == c(i, 2));
        const std::size_t mz = d3q19::mirror_z[i];
        CHECK(c(mz, 0) == c(i, 0));
        CHECK(c(mz, 1) == c(i, 1));
        CHECK(c(mz, 2) == -c(i, 2));
        // Every direction is distinct.
        for (std::size_t j = 0; j < i; ++j) {
            CHECK((c(i, 0) != c(j, 0) || c(i, 1) != c(j, 1) || c(i, 2) != c(j, 2)));
        }
    }
}

TEST_CASE("The D3Q19 equilibrium carries density and momentum", "[aero][lbm3d]") {
    const d3q19::Moments m{1.2, 0.04, -0.03, 0.02};
    const d3q19::Populations f = d3q19::equilibrium(m);
    const d3q19::Moments back = d3q19::moments(f);
    CHECK(back.rho == Approx(m.rho).epsilon(1e-14));
    CHECK(back.ux == Approx(m.ux).epsilon(1e-13));
    CHECK(back.uy == Approx(m.uy).epsilon(1e-13));
    CHECK(back.uz == Approx(m.uz).epsilon(1e-13));
    // Momentum flux rho (c_s^2 delta + u u).
    const std::array<double, 3> u{m.ux, m.uy, m.uz};
    for (std::size_t a = 0; a < 3; ++a) {
        for (std::size_t b = 0; b < 3; ++b) {
            double pi = 0.0;
            for (std::size_t i = 0; i < d3q19::q; ++i) {
                pi += c(i, a) * c(i, b) * f[i];
            }
            CHECK(pi == Approx(m.rho * (d3q19::cs2 * delta(a, b) + u[a] * u[b])).margin(1e-14));
        }
    }
}
