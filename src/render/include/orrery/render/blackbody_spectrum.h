#pragma once

#include <cstddef>
#include <vector>

#include <glm/ext/vector_double3.hpp>
#include <glm/ext/vector_float4.hpp>

// Visible-band radiance of a black body in linear sRGB (docs/rendering.md,
// "Black hole", disk colour). The Planck spectrum is integrated against an
// analytic fit of the CIE 1931 2-degree colour matching functions and the
// result converted from XYZ to linear sRGB (D65). Unlike
// blackbody_linear_srgb (body_visuals.h), which gives only the chromaticity
// at unit luminance, these values keep the absolute brightness, so that the
// strong dependence of visible light on temperature (Wien tail) shows: a
// Doppler-shifted disk element at temperature g T is both bluer and much
// brighter than one at T.
namespace orrery::render {

// CIE 1931 2-degree colour matching functions (x-bar, y-bar, z-bar) at a
// wavelength in nanometres.
//
// APPROXIMATION: colour matching functions - the multi-lobe piecewise
// Gaussian fit of Wyman, Sloan and Shirley (JCGT 2013) instead of the
// tabulated CIE data - within a few percent of the table over 380..780 nm,
// which moves black-body chromaticities by about 1e-3 in xy.
[[nodiscard]] glm::dvec3 cie_colour_matching(double wavelength_nm) noexcept;

// Planck spectral radiance B_lambda(T) [W sr^-1 m^-3] at a wavelength in
// metres. Zero for non-positive temperatures.
[[nodiscard]] double planck_spectral_radiance(double wavelength_m, double kelvin) noexcept;

// CIE XYZ of a black body, the Planck spectrum integrated over 360..830 nm
// (1 nm trapezoid rule) against the colour matching functions, scaled so
// that Y = 1 at kBlackbodyReferenceKelvin.
inline constexpr double kBlackbodyReferenceKelvin = 6500.0;
[[nodiscard]] glm::dvec3 blackbody_xyz(double kelvin) noexcept;

// Linear sRGB (D65 white) of blackbody_xyz; components that fall outside the
// sRGB gamut (slightly negative blue below about 1900 K) are clipped to zero.
[[nodiscard]] glm::dvec3 blackbody_radiance_srgb(double kelvin) noexcept;

// Luminance Y of blackbody_xyz (relative to kBlackbodyReferenceKelvin).
[[nodiscard]] double blackbody_luminance(double kelvin) noexcept;

// Lookup table of blackbody_radiance_srgb for the GPU: `size` texels evenly
// spaced in ln T from min_kelvin to max_kelvin (rgb = radiance, a = 1).
struct BlackbodyTable {
    double min_kelvin = 0.0;
    double max_kelvin = 0.0;
    std::vector<glm::vec4> texels;
};
inline constexpr double kBlackbodyTableMinKelvin = 400.0;
inline constexpr double kBlackbodyTableMaxKelvin = 200000.0;
inline constexpr std::size_t kBlackbodyTableSize = 1024;
[[nodiscard]] BlackbodyTable make_blackbody_table(std::size_t size = kBlackbodyTableSize,
                                                  double min_kelvin = kBlackbodyTableMinKelvin,
                                                  double max_kelvin = kBlackbodyTableMaxKelvin);

} // namespace orrery::render
