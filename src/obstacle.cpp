#include <robot_simulation/obstacle.hpp>
#include <robot_simulation/geometry.hpp>
#include <stdexcept>

namespace robot_simulation{
    Obstacle::Obstacle(Pose pose, Rectangle size):
    pose_(pose),
    size_(size){
        if(size_.length <= 0 || size_.width <= 0){
            throw std::invalid_argument("obstacle size must be positive!");
        }
    }

    RectangleCorners Obstacle::getObstacleCorners () const {
        return calculateRectangleCorners(pose_, size_);
    }
}