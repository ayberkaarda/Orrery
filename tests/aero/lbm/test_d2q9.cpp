#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <cstddef>

#include "orrery/aero/lbm/d2q9.h"

using namespace orrery::aero::lbm;
using d2q9::q;

TEST_CASE("D2Q9 weights reproduce the lattice isotropy moments", "[aero][lbm]") {
    double w0 = 0.0;
    std::array<double, 2> w1{};
    std::array<std::array<double, 2>, 2> w2{};
    double xxxx = 0.0;
    double xxyy = 0.0;
    for (std::size_t i = 0; i < q; ++i) {
        const auto ex = static_cast<double>(d2q9::cx[i]);
        const auto ey = static_cast<double>(d2q9::cy[i]);
        w0 += d2q9::w[i];
        w1[0] += d2q9::w[i] * ex;
        w1[1] += d2q9::w[i] * ey;
        w2[0][0] += d2q9::w[i] * ex * ex;
        w2[0][1] += d2q9::w[i] * ex * ey;
        w2[1][1] += d2q9::w[i] * ey * ey;
        xxxx += d2q9::w[i] * ex * ex * ex * ex;
        xxyy += d2q9::w[i] * ex * ex * ey * ey;
    }
    CHECK(w0 == Catch::Approx(1.0).epsilon(1e-15));
    CHECK(w1[0] == 0.0);
    CHECK(w1[1] == 0.0);
    CHECK(w2[0][0] == Catch::Approx(d2q9::cs2).epsilon(1e-15));
    CHECK(w2[1][1] == Catch::Approx(d2q9::cs2).epsilon(1e-15));
    CHECK(w2[0][1] == 0.0);
    // Fourth-order isotropy: w c_x^4 = 3 cs^4, w c_x^2 c_y^2 = cs^4.
    CHECK(xxxx == Catch::Approx(3.0 * d2q9::cs2 * d2q9::cs2).epsilon(1e-15));
    CHECK(xxyy == Catch::Approx(d2q9::cs2 * d2q9::cs2).epsilon(1e-15));
}

TEST_CASE("D2Q9 opposite and mirror tables are consistent", "[aero][lbm]") {
    for (std::size_t i = 0; i < q; ++i) {
        const std::size_t o = d2q9::opposite[i];
        CHECK(d2q9::cx[o] == -d2q9::cx[i]);
        CHECK(d2q9::cy[o] == -d2q9::cy[i]);
        CHECK(d2q9::opposite[o] == i);
        CHECK(d2q9::w[o] == d2q9::w[i]);
        const std::size_t m = d2q9::mirror_y[i];
        CHECK(d2q9::cx[m] == d2q9::cx[i]);
        CHECK(d2q9::cy[m] == -d2q9::cy[i]);
        CHECK(d2q9::mirror_y[m] == i);
    }
}

TEST_CASE("Equilibrium moments are rho, rho u and rho cs^2 I + rho u u", "[aero][lbm]") {
    const std::array<std::array<double, 3>, 5> states{{
        {1.0, 0.0, 0.0},
        {1.0, 0.05, 0.0},
        {0.98, -0.03, 0.04},
        {1.02, 0.01, -0.057},
        {0.5, 0.1, 0.1},
    }};
    for (const auto& s : states) {
        const double rho = s[0];
        const double ux = s[1];
        const double uy = s[2];
        const d2q9::Populations f = d2q9::equilibrium(rho, ux, uy);
        double m0 = 0.0;
        double jx = 0.0;
        double jy = 0.0;
        double pxx = 0.0;
        double pxy = 0.0;
        double pyy = 0.0;
        for (std::size_t i = 0; i < q; ++i) {
            const auto ex = static_cast<double>(d2q9::cx[i]);
            const auto ey = static_cast<double>(d2q9::cy[i]);
            m0 += f[i];
            jx += ex * f[i];
            jy += ey * f[i];
            pxx += ex * ex * f[i];
            pxy += ex * ey * f[i];
            pyy += ey * ey * f[i];
        }
        INFO("rho " << rho << " u (" << ux << ", " << uy << ")");
        CHECK(std::abs(m0 - rho) < 1e-15);
        CHECK(std::abs(jx - rho * ux) < 1e-15);
        CHECK(std::abs(jy - rho * uy) < 1e-15);
        CHECK(std::abs(pxx - (rho * d2q9::cs2 + rho * ux * ux)) < 1e-15);
        CHECK(std::abs(pyy - (rho * d2q9::cs2 + rho * uy * uy)) < 1e-15);
        CHECK(std::abs(pxy - rho * ux * uy) < 1e-15);

        const d2q9::Moments m = d2q9::moments(f);
        CHECK(std::abs(m.rho - rho) < 1e-15);
        CHECK(std::abs(m.ux - ux) < 1e-15);
        CHECK(std::abs(m.uy - uy) < 1e-15);
    }
}

TEST_CASE("Rest equilibrium is the weight set", "[aero][lbm]") {
    const d2q9::Populations f = d2q9::equilibrium(1.0, 0.0, 0.0);
    for (std::size_t i = 0; i < q; ++i) {
        CHECK(f[i] == d2q9::w[i]);
    }
}
