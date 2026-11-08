#pragma once
#include <array>

namespace robot_simulation{

    struct Pose
    {
        double x{};
        double y{};
        double theta{};
    };

    struct Velocity{
        double linear_velocity{};
        double angular_velocity{};
    };

    struct Rectangle{
            double width{};
            double length{};
            
    };
    
    struct Point{
        double x{};
        double y{};
    };

    using RectangleCorners = std::array<Point, 4>;

    using DistanceSensorID = std::size_t;
}// namespace robot_simulation