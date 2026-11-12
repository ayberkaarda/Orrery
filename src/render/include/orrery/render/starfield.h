#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <glm/ext/matrix_double3x3.hpp>
#include <glm/ext/vector_float3.hpp>

// Procedural background sky (docs/rendering.md, "Starfield"): a seeded,
// deterministic set of stars with a plausible magnitude distribution,
// spectral colours and concentration towards the galactic plane, plus the
// galactic frame used by the Milky Way band. No catalogue data is used.
namespace orrery::render {

struct StarPoint {
    glm::vec3 direction{0.0F, 0.0F, 1.0F}; // unit vector, world (ecliptic J2000) axes
    float magnitude = 0.0F;                // apparent visual magnitude
    float temperature = 5772.0F;           // effective temperature [K]
    glm::vec3 color{1.0F};                 // linear sRGB, unit luminance
};

struct StarfieldParams {
    std::uint64_t seed = 0x0A11CE5EEDULL;
    std::size_t count = 100000;
    double brightest_magnitude = -1.5;
    double faintest_magnitude = 10.0;
    // d log10 N(<m) / dm of the cumulative star counts. The whole-sky counts
    // (about 5e3 stars to V = 6 and 1.2e5 to V = 10) give about 0.35.
    double count_slope = 0.35;
    // Stars appended after the generated ones, unchanged (test scenes, for
    // example a single bright star behind a black hole).
    std::vector<StarPoint> extra_stars;
};

// Same parameters, same stars on every platform (own random generator, no
// standard-library distributions); extra_stars follow the generated ones.
[[nodiscard]] std::vector<StarPoint> generate_starfield(const StarfieldParams& params);

// Rotation from world (ecliptic J2000) to galactic coordinates (IAU 1958
// definition: north galactic pole RA 192.85948 deg, Dec 27.12825 deg; centre
// RA 266.40500 deg, Dec -28.93617 deg, ICRS J2000).
[[nodiscard]] glm::dmat3 ecliptic_to_galactic() noexcept;

// Deterministic 64-bit generator (SplitMix64) used by the starfield.
class SplitMix64 {
public:
    explicit SplitMix64(std::uint64_t seed) noexcept : state_(seed) {}
    [[nodiscard]] std::uint64_t next() noexcept;
    // Uniform in [0, 1) with 53 random bits.
    [[nodiscard]] double uniform() noexcept;

private:
    std::uint64_t state_;
};

} // namespace orrery::render
