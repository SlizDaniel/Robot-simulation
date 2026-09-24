#pragma once
#include <vector>
#include <robot_simulation/obstacle.hpp>
#include <robot_simulation/types.hpp>
#include <robot_simulation/robot.hpp>

namespace robot_simulation{
    class Environment{
        private:
        
        Rectangle size_;

        std::vector<Obstacle> obstacles_{}; //tutaj powinno byc cos z &
        
        public:

        Environment(Rectangle size);

        bool collides(const RectangleCorners& robot_corners, const Pose& robot_position) const;

        bool isRobotInside(const RectangleCorners& robot_corners) const;

        bool collides (const Robot& robot) const;

        bool isRobotInside(const Robot& robot) const;

        void addObstacle(const Obstacle& obstacle);

        Rectangle getEnvironmentSize() const {return size_;};

        const std::vector<Obstacle>& getObstacles() const {return obstacles_;}
    };
}// namespace robot_simulation
