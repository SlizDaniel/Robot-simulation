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

        Environment(Rectangle rectangle){
            size_ = rectangle;
        }

        bool collides(Robot& robot) const;

        void addObstacle(Obstacle& obstacle);

        Rectangle getEnvironmentSize() const {return size_;};

        std::vector<Obstacle> getObstacles() const {return obstacles_;}
    };
}// namespace robot_simulation
