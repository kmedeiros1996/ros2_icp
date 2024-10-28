#pragma once
#include <cmath>

namespace ICP {
namespace Geometry {

// Angles are normalized between -pi to pi

inline double AbsoluteAngleDifference(const double angle_a_radians, const double angle_b_radians) {
    return M_PI - fabs(fmod(fabs(angle_a_radians - angle_b_radians), 2.0*M_PI) - M_PI);
}
} // namespace Geometry
} // namespace ICP