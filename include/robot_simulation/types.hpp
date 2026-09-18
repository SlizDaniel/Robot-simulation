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
        double linearVelocity{};
        double angularVelocity{};
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
    // struct RectangleCorners{
    //     Point A;
    //     Point B;
    //     Point C;
    //     Point D; 
    // };
}// namespace robot_simulation