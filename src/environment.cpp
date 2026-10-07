#include <robot_simulation/environment.hpp>
#include <cmath>
#include <array>
#include <robot_simulation/math_constants.hpp>
#include <algorithm>
#include <vector>
#include <stdexcept>

namespace robot_simulation{
    namespace{
        bool collisionObstacleRobot(const RectangleCorners& robot_corners,
            const Pose& robot_position, const Obstacle& obstacle){

            const RectangleCorners obstacle_corners = obstacle.getObstacleCorners();
            const Pose obstacle_position = obstacle.get_obstacle_pose();
            std::vector<std::array<double, 2>> rotations = {};
            rotations.push_back({std::cos(robot_position.theta), 
                std::sin(robot_position.theta)});
            rotations.push_back({std::cos(robot_position.theta+90*pi/180), 
                std::sin(robot_position.theta+90*pi/180)});
            rotations.push_back({std::cos(obstacle_position.theta), 
                std::sin(obstacle_position.theta)});
            rotations.push_back({std::cos(obstacle_position.theta+90*pi/180), 
                std::sin(obstacle_position.theta+90*pi/180)});

            std::size_t collisions = 0;

            for (std::size_t i = 0; i<rotations.size(); i++){
                std::array<double, 4> robot_projections = {};
                std::array<double, 4> obstacle_projections = {};
                for (std::size_t j=0; j<robot_corners.size(); j++){
                    robot_projections[j] = robot_corners[j].x
                    * rotations[i][0] + robot_corners[j].y * rotations[i][1];
                    obstacle_projections[j] = obstacle_corners[j].x
                    * rotations[i][0] + obstacle_corners[j].y * rotations[i][1];
                }
                double max_robot = *std::max_element(
                    robot_projections.begin(),
                    robot_projections.end());
                double min_robot = *std::min_element(
                    robot_projections.begin(),
                    robot_projections.end());
                double max_obstacle = *std::max_element(
                    obstacle_projections.begin(),
                    obstacle_projections.end());
                double min_obstacle = *std::min_element(
                    obstacle_projections.begin(),
                    obstacle_projections.end());
                if (!(max_robot < min_obstacle || max_obstacle < min_robot)){
                    collisions +=1;
                }    
            }
            return(collisions == rotations.size());
        }  
    }//namespace
    Environment::Environment(Rectangle size):
        size_(size){
            if (size.length <= 0 || size.width <= 0){
                throw std::invalid_argument("Environment size must be positive!");
            }
        }

    bool Environment::collides(const RectangleCorners& robot_corners,
        const Pose& robot_position) const{
        for(std::size_t i = 0; i<obstacles_.size(); i++){
            if (collisionObstacleRobot(robot_corners, robot_position, obstacles_[i])){
                return true;
            }
        }
        return false;
    }

    bool Environment::isRobotInside(const RectangleCorners& robot_corners) const {
        for (std::size_t i = 0; i < robot_corners.size(); i++){
            if (!(-size_.length/2 <= robot_corners[i].x && robot_corners[i].x <= size_.length/2
                && -size_.width/2 <= robot_corners[i].y && robot_corners[i].y <= size_.width/2)){
                    return false;
                }
        }
        return true;
    }

    void Environment::addObstacle(const Obstacle& obstacle){
        obstacles_.push_back(obstacle);
    }

    bool Environment::collides(const Robot& robot) const {
        return collides(robot.getRobotCorners(), robot.getPose());
    }

    bool Environment::isRobotInside(const Robot& robot) const {
        return isRobotInside(robot.getRobotCorners());
    }
}//robot_simulation
