#include <robot_simulation/obstacle.hpp>
#include <robot_simulation/geometry.hpp>

namespace robot_simulation{
    RectangleCorners Obstacle::getObstacleCorners () const {
        return calculateRectangleCorners(pose_, size_);
    }
}