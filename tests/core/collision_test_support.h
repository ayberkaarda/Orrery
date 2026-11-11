#pragma once

// Shared helpers for the Phase 3b tests (collisions, horizon, events).

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

#include "orrery/core/body_store.h"
#include "orrery/core/conservation.h"
#include "orrery/core/vec3.h"

namespace orrery::core::test {

inline Vec3<space::Length> at(double x, double y, double z) {
    return {space::Length(x), space::Length(y), space::Length(z)};
}

inline Vec3<space::Velocity> vel(double x, double y, double z) {
    return {space::Velocity(x), space::Velocity(y), space::Velocity(z)};
}

// Deterministic generator (SplitMix64), identical on every platform.
struct SplitMix {
    std::uint64_t state = 0;
    double uniform() {
        std::uint64_t z = (state += 0x9e3779b97f4a7c15ULL);
        z = (z ^ (z >> 30U)) * 0xbf58476d1ce4e5b9ULL;
        z = (z ^ (z >> 27U)) * 0x94d049bb133111ebULL;
        z ^= z >> 31U;
        return static_cast<double>(z >> 11U) * 0x1p-53;
    }
    double symmetric() { return 2.0 * uniform() - 1.0; }
};

using Triple = std::array<double, 3>;

// Totals over every body (probes included), with scales for relative errors.
struct Totals {
    double mass = 0.0;
    Triple momentum{};
    Triple moment{};
    Triple angular{};
    double momentum_scale = 0.0;
    double moment_scale = 0.0;
    double angular_scale = 0.0;
};

inline Totals totals(const BodyStore& b) {
    Totals t;
    for (std::size_t i = 0; i < b.size(); ++i) {
        const double m = b.mass(i).value();
        const auto r = b.position(i);
        const auto v = b.velocity(i);
        const Triple rr{r.x.value(), r.y.value(), r.z.value()};
        const Triple vv{v.x.value(), v.y.value(), v.z.value()};
        const Triple l{rr[1] * vv[2] - rr[2] * vv[1], rr[2] * vv[0] - rr[0] * vv[2],
                       rr[0] * vv[1] - rr[1] * vv[0]};
        t.mass += m;
        for (std::size_t k = 0; k < 3; ++k) {
            t.momentum[k] += m * vv[k];
            t.moment[k] += m * rr[k];
            t.angular[k] += m * l[k];
        }
        t.momentum_scale += m * norm(v).value();
        t.moment_scale += m * norm(r).value();
        t.angular_scale += m * std::sqrt(l[0] * l[0] + l[1] * l[1] + l[2] * l[2]);
    }
    return t;
}

inline double difference(const Triple& a, const Triple& b) {
    return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) +
                     (a[2] - b[2]) * (a[2] - b[2]));
}

inline Triple raw(const Vec3<space::AngularMomentum>& v) {
    return {v.x.value(), v.y.value(), v.z.value()};
}

inline Triple plus(const Triple& a, const Triple& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

inline void set_radius_or_throw(BodyStore& store, std::size_t i, double r) {
    if (!store.set_radius(i, space::Length(r))) {
        throw std::runtime_error("set_radius failed");
    }
}

} // namespace orrery::core::test
