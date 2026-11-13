#pragma once

// Private vector conversions between the space units of core (AU, solar mass,
// day) and SI. Not part of the public include directory.

#include "orrery/core/units.h"
#include "orrery/core/vec3.h"

namespace orrery::atmo::detail {

template <typename Q> [[nodiscard]] auto to_si_vec(const core::Vec3<Q>& v) noexcept {
    return core::Vec3<decltype(core::to_si(Q{}))>{core::to_si(v.x), core::to_si(v.y),
                                                  core::to_si(v.z)};
}

template <typename Q> [[nodiscard]] auto to_space_vec(const core::Vec3<Q>& v) noexcept {
    return core::Vec3<decltype(core::to_space(Q{}))>{core::to_space(v.x), core::to_space(v.y),
                                                     core::to_space(v.z)};
}

} // namespace orrery::atmo::detail
