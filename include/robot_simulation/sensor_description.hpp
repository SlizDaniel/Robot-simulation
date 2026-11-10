#pragma once

#include <robot_simulation/types.hpp>
#include <string>

namespace robot_simulation {
    struct SensorDescription {
        DistanceSensorID id;
        std::string role;
        Pose relative_pose;
        double min_distance;
        double max_distance;
        double field_of_view;
    };
}
