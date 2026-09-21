#pragma once
#include <robot_simulation/types.hpp>

namespace robot_simulation{
    class Obstacle{
        private:
        
        Pose pose_;
        
        Rectangle size_;

        public:

        Obstacle(Pose pose, Rectangle rectangle);

        Pose get_obstacle_pose() const {return pose_;}

        Rectangle get_obstacle_size() const {return size_;}

        RectangleCorners getObstacleCorners() const;
    };
}// namespace robot_simulation