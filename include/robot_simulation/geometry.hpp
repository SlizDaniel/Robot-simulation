#pragma once
#include <array>
#include <robot_simulation/types.hpp>
#include <robot_simulation/distance_sensor.hpp>
#include <robot_simulation/obstacle.hpp>

namespace robot_simulation{
    RectangleCorners calculateRectangleCorners (Pose pose, Rectangle size);

    RaycastResult calculateDistanceSensorObstacle (const Pose& distance_sensor_pose,
            const std::array<double,2>& min_max_sensor_distance, const Obstacle& obstacle);
}
