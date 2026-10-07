#pragma once
#include <limits>

namespace robot_simulation {
    inline constexpr double pi =
        3.14159265358979323846;

    const double epsilon = 1e-9;
    const double infinity = std::numeric_limits<double>::infinity();
}// namespace robot_simulation
