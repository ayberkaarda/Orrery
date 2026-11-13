#include "orrery/aero/aero_result.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace orrery::aero {

namespace {

bool positive_finite(double v) noexcept {
    return std::isfinite(v) && v > 0.0;
}

} // namespace

bool AeroResult::has_warning(std::string_view code) const noexcept {
    return std::any_of(warnings.begin(), warnings.end(),
                       [code](const ValidityWarning& w) { return w.code == code; });
}

void set_coefficients(AeroResult& result, const FlowConditions& conditions,
                      const ReferenceValues& reference) {
    const double q = conditions.dynamic_pressure();
    if (!positive_finite(q) || !positive_finite(reference.area) ||
        !positive_finite(reference.length)) {
        throw std::invalid_argument(
            "set_coefficients: dynamic pressure, reference area and length must be positive");
    }
    const double qs = q * reference.area;
    result.cd = geom::dot(result.force, conditions.drag_direction()) / qs;
    result.cl = geom::dot(result.force, conditions.lift_direction()) / qs;
    result.cy = geom::dot(result.force, conditions.side_direction()) / qs;
    result.cm = result.moment.y / (qs * reference.length);
}

} // namespace orrery::aero
