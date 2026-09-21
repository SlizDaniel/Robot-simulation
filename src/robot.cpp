#include <robot_simulation/robot.hpp>
#include <robot_simulation/geometry.hpp>
#include <cmath>
#include <stdexcept>

namespace robot_simulation{

    Robot::Robot(Pose pose, Velocity velocity, Rectangle size):
            pose_(pose),
            velocity_(velocity),
            size_(size){
        if (size.length <= 0 || size.width <= 0){
            throw std::invalid_argument("Robot size must be positive!");
        }
    }

    void Robot::stop(){
        velocity_ = {0.0 , 0.0};        
    }

    void Robot::move(double dt){
        double dx = 0.0;
        double dy = 0.0;
        if (std::abs(velocity_.angularVelocity) * dt < 1e-6){
            dy = velocity_.linearVelocity * dt * std::sin(pose_.theta);
            dx = velocity_.linearVelocity * dt *std::cos(pose_.theta);
            pose_.theta +=velocity_.angularVelocity * dt;
        }
        else{
            double theta_next = pose_.theta + velocity_.angularVelocity * dt;
            dy = (velocity_.linearVelocity/velocity_.angularVelocity) * (std::cos(pose_.theta)-std::cos(theta_next));
            dx = (velocity_.linearVelocity/velocity_.angularVelocity) * (std::sin(theta_next)-std::sin(pose_.theta));
            pose_.theta = theta_next;
        }
        pose_.x += dx;
        pose_.y += dy;
    }

    RectangleCorners Robot::getRobotCorners() const {
        return calculateRectangleCorners(pose_, size_);
    }
}