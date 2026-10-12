#include <robot_simulation/obstacle.hpp>
#include <robot_simulation/geometry.hpp>
#include <cmath>
#include <stdexcept>

namespace robot_simulation{
    Obstacle::Obstacle(Pose pose, Rectangle size):
    pose_(pose),
    size_(size){
        if (!std::isfinite(pose_.x) || !std::isfinite(pose_.y) || !std::isfinite(pose_.theta) ||
            !std::isfinite(size_.length) || !std::isfinite(size_.width)){
            throw std::invalid_argument("Obstacle parameters must be finite!");
        }
        if(size_.length <= 0 || size_.width <= 0){
            throw std::invalid_argument("obstacle size must be positive!");
        }
    }

    RectangleCorners Obstacle::getObstacleCorners () const {
        return calculateRectangleCorners(pose_, size_);
    }
}
