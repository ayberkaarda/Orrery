#include "orrery/render/blackbody_spectrum.h"

#include <algorithm>
#include <cmath>

#include <glm/common.hpp>

namespace orrery::render {

namespace {

// Second radiation constant c2 = h c / k [m K] (CODATA 2018, exact).
constexpr double kSecondRadiationConstant = 1.438776877e-2;
// First radiation constant for spectral radiance, 2 h c^2 [W m^2 sr^-1].
constexpr double kFirstRadiationConstant = 1.191042972e-16;
constexpr double kLowNm = 360.0;
constexpr double kHighNm = 830.0;

// Piecewise Gaussian lobe of the Wyman-Sloan-Shirley fit.
double lobe(double x, double mu, double sigma_low, double sigma_high) noexcept {
    const double t = (x - mu) / (x < mu ? sigma_low : sigma_high);
    return std::exp(-0.5 * t * t);
}

glm::dvec3 integrate_xyz(double kelvin) noexcept {
    glm::dvec3 sum(0.0);
    const int intervals = static_cast<int>(kHighNm - kLowNm);
    for (int i = 0; i <= intervals; ++i) {
        const double nm = kLowNm + static_cast<double>(i);
        const double weight = (i == 0 || i == intervals) ? 0.5 : 1.0;
        sum += weight * planck_spectral_radiance(nm * 1.0e-9, kelvin) * cie_colour_matching(nm);
    }
    return sum;
}

} // namespace

glm::dvec3 cie_colour_matching(double nm) noexcept {
    const double x = 1.056 * lobe(nm, 599.8, 37.9, 31.0) + 0.362 * lobe(nm, 442.0, 16.0, 26.7) -
                     0.065 * lobe(nm, 501.1, 20.4, 26.2);
    const double y = 0.821 * lobe(nm, 568.8, 46.9, 40.5) + 0.286 * lobe(nm, 530.9, 16.3, 31.1);
    const double z = 1.217 * lobe(nm, 437.0, 11.8, 36.0) + 0.681 * lobe(nm, 459.0, 26.0, 13.8);
    return {x, y, z};
}

double planck_spectral_radiance(double wavelength_m, double kelvin) noexcept {
    if (!(kelvin > 0.0) || !(wavelength_m > 0.0)) {
        return 0.0;
    }
    const double exponent = kSecondRadiationConstant / (wavelength_m * kelvin);
    if (exponent > 700.0) {
        return 0.0; // below the smallest double
    }
    const double l2 = wavelength_m * wavelength_m;
    return kFirstRadiationConstant / (l2 * l2 * wavelength_m) / std::expm1(exponent);
}

glm::dvec3 blackbody_xyz(double kelvin) noexcept {
    static const double reference = integrate_xyz(kBlackbodyReferenceKelvin).y;
    return integrate_xyz(kelvin) / reference;
}

// APPROXIMATION: gamut clipping - negative linear sRGB components (the
// Planckian locus leaves the sRGB gamut below about 1900 K) are set to 0 -
// sRGB has no deeper red - changes the colour of black bodies below 1900 K
// by a few percent of their luminance; above it the conversion is exact.
glm::dvec3 blackbody_radiance_srgb(double kelvin) noexcept {
    const glm::dvec3 xyz = blackbody_xyz(kelvin);
    // XYZ -> linear sRGB (IEC 61966-2-1, D65).
    const glm::dvec3 rgb{3.2404542 * xyz.x - 1.5371385 * xyz.y - 0.4985314 * xyz.z,
                         -0.9692660 * xyz.x + 1.8760108 * xyz.y + 0.0415560 * xyz.z,
                         0.0556434 * xyz.x - 0.2040259 * xyz.y + 1.0572252 * xyz.z};
    return glm::max(rgb, glm::dvec3(0.0));
}

double blackbody_luminance(double kelvin) noexcept {
    return blackbody_xyz(kelvin).y;
}

BlackbodyTable make_blackbody_table(std::size_t size, double min_kelvin, double max_kelvin) {
    BlackbodyTable table;
    table.min_kelvin = min_kelvin;
    table.max_kelvin = max_kelvin;
    const std::size_t n = std::max<std::size_t>(size, 2);
    table.texels.reserve(n);
    const double lo = std::log(min_kelvin);
    const double hi = std::log(max_kelvin);
    for (std::size_t i = 0; i < n; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(n - 1);
        const glm::dvec3 rgb = blackbody_radiance_srgb(std::exp(lo + t * (hi - lo)));
        table.texels.emplace_back(static_cast<float>(rgb.r), static_cast<float>(rgb.g),
                                  static_cast<float>(rgb.b), 1.0F);
    }
    return table;
}

} // namespace orrery::render
