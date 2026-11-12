#include "orrery/render/starfield.h"

#include <algorithm>
#include <array>
#include <cmath>

#include <glm/ext/vector_double3.hpp>
#include <glm/geometric.hpp>

#include "orrery/render/body_visuals.h"

namespace orrery::render {

namespace {

constexpr double kPi = 3.141592653589793;
constexpr double kDegree = kPi / 180.0;
constexpr double kObliquityJ2000 = 23.4392911 * kDegree;

glm::dvec3 ecliptic_from_equatorial(double ra_degrees, double dec_degrees) noexcept {
    const double ra = ra_degrees * kDegree;
    const double dec = dec_degrees * kDegree;
    const glm::dvec3 e{std::cos(dec) * std::cos(ra), std::cos(dec) * std::sin(ra), std::sin(dec)};
    const double c = std::cos(kObliquityJ2000);
    const double s = std::sin(kObliquityJ2000);
    return {e.x, c * e.y + s * e.z, -s * e.y + c * e.z};
}

double smoothstep(double edge0, double edge1, double x) noexcept {
    const double t = std::clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

struct SpectralClass {
    double weight;
    double t_min;
    double t_max;
};

// APPROXIMATION: spectral mix of a magnitude-limited sample (O B A F G K M
// with weights 0.3/12/22/17/14/27/7.7 percent, roughly the classes of stars
// brighter than V = 6.5 in the Yale Bright Star Catalogue), temperatures
// log-uniform within each class and no magnitude dependence - colours are
// plausible as a population, not star by star.
constexpr std::array<SpectralClass, 7> kClasses{{
    {0.003, 30000.0, 45000.0},
    {0.120, 10000.0, 30000.0},
    {0.220, 7500.0, 10000.0},
    {0.170, 6000.0, 7500.0},
    {0.140, 5200.0, 6000.0},
    {0.270, 3700.0, 5200.0},
    {0.077, 2400.0, 3700.0},
}};

double sample_temperature(SplitMix64& rng) noexcept {
    double total = 0.0;
    for (const SpectralClass& c : kClasses) {
        total += c.weight;
    }
    double pick = rng.uniform() * total;
    const SpectralClass* chosen = &kClasses.back();
    for (const SpectralClass& c : kClasses) {
        if (pick < c.weight) {
            chosen = &c;
            break;
        }
        pick -= c.weight;
    }
    const double u = rng.uniform();
    return std::exp(std::log(chosen->t_min) +
                    u * (std::log(chosen->t_max) - std::log(chosen->t_min)));
}

// Inverse of the truncated cumulative count N(<m) ~ 10^(k m) on [m0, m1].
double sample_magnitude(SplitMix64& rng, const StarfieldParams& p) noexcept {
    const double k = std::max(p.count_slope, 1.0e-3);
    const double a = std::pow(10.0, k * p.brightest_magnitude);
    const double b = std::pow(10.0, k * p.faintest_magnitude);
    return std::log10(a + rng.uniform() * (b - a)) / k;
}

glm::dvec3 uniform_direction(SplitMix64& rng) noexcept {
    const double z = 2.0 * rng.uniform() - 1.0;
    const double phi = 2.0 * kPi * rng.uniform();
    const double r = std::sqrt(std::max(0.0, 1.0 - z * z));
    return {r * std::cos(phi), r * std::sin(phi), z};
}

// APPROXIMATION: galactic structure of the star counts - a disc population
// with an exponential distribution in sin(b) (scale 0.12) and a longitude
// excess towards the galactic centre, whose share grows from 30 to 75 percent
// between V = 3 and V = 10 - reproduces the visible concentration of faint
// stars towards the Milky Way, not any real star.
glm::dvec3 disc_direction_galactic(SplitMix64& rng) noexcept {
    double longitude = 0.0;
    for (int attempt = 0; attempt < 64; ++attempt) {
        longitude = (2.0 * rng.uniform() - 1.0) * kPi;
        const double density =
            (1.0 + 1.6 * std::exp(-(longitude * longitude) / (2.0 * 0.6 * 0.6))) / 2.6;
        if (rng.uniform() < density) {
            break;
        }
    }
    const double scale =
        0.12 * (1.0 + 0.8 * std::exp(-(longitude * longitude) / (2.0 * 0.4 * 0.4)));
    double sin_b = -scale * std::log(std::max(rng.uniform(), 1.0e-12));
    sin_b = std::min(sin_b, 1.0);
    if (rng.uniform() < 0.5) {
        sin_b = -sin_b;
    }
    const double cos_b = std::sqrt(std::max(0.0, 1.0 - sin_b * sin_b));
    return {cos_b * std::cos(longitude), cos_b * std::sin(longitude), sin_b};
}

} // namespace

std::uint64_t SplitMix64::next() noexcept {
    state_ += 0x9E3779B97F4A7C15ULL;
    std::uint64_t z = state_;
    z = (z ^ (z >> 30U)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27U)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31U);
}

double SplitMix64::uniform() noexcept {
    return static_cast<double>(next() >> 11U) * 0x1.0p-53;
}

glm::dmat3 ecliptic_to_galactic() noexcept {
    const glm::dvec3 pole = glm::normalize(ecliptic_from_equatorial(192.85948, 27.12825));
    const glm::dvec3 centre = ecliptic_from_equatorial(266.40500, -28.93617);
    const glm::dvec3 x = glm::normalize(centre - glm::dot(centre, pole) * pole);
    const glm::dvec3 y = glm::cross(pole, x);
    // glm matrices are column-major: the rows of the rotation are the
    // galactic axes expressed in ecliptic coordinates.
    return glm::transpose(glm::dmat3(x, y, pole));
}

std::vector<StarPoint> generate_starfield(const StarfieldParams& params) {
    SplitMix64 rng(params.seed);
    const glm::dmat3 galactic_to_ecliptic = glm::transpose(ecliptic_to_galactic());
    std::vector<StarPoint> stars;
    stars.reserve(params.count);
    for (std::size_t i = 0; i < params.count; ++i) {
        const double magnitude = sample_magnitude(rng, params);
        const double disc_share = 0.30 + 0.45 * smoothstep(3.0, 10.0, magnitude);
        glm::dvec3 direction{};
        if (rng.uniform() < disc_share) {
            direction = galactic_to_ecliptic * disc_direction_galactic(rng);
        } else {
            direction = uniform_direction(rng);
        }
        const double temperature = sample_temperature(rng);
        StarPoint star;
        star.direction = glm::vec3(glm::normalize(direction));
        star.magnitude = static_cast<float>(magnitude);
        star.temperature = static_cast<float>(temperature);
        star.color = glm::vec3(blackbody_linear_srgb(temperature));
        stars.push_back(star);
    }
    return stars;
}

} // namespace orrery::render
