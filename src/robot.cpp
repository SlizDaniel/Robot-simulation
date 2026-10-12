#include <robot_simulation/robot.hpp>
#include <robot_simulation/geometry.hpp>
#include <robot_simulation/math_constants.hpp>
#include <cmath>
#include <stdexcept>

namespace robot_simulation{

    Robot::Robot(Pose pose, Velocity velocity, Rectangle size):
            pose_(pose),
            velocity_(velocity),
            size_(size){
        if (!std::isfinite(pose_.x) || !std::isfinite(pose_.y) || !std::isfinite(pose_.theta) ||
            !std::isfinite(velocity_.linearVelocity) || !std::isfinite(velocity_.angularVelocity) ||
            !std::isfinite(size_.length) || !std::isfinite(size_.width)){
            throw std::invalid_argument("Robot parameters must be finite");
        }
        if (size.length <= 0 || size.width <= 0){
            throw std::invalid_argument("Robot size must be positive!");
        }
        if (velocity.angularVelocity < -2.0 * pi || velocity.angularVelocity > 2.0 * pi){
            throw std::invalid_argument("Robot angular velocity must be in range [-2*pi, 2*pi]");
        }
        addPointToTrajectory(pose_);
    }

    void Robot::stop(){
        velocity_ = {0.0 , 0.0};        
    }

namespace{
    Pose calculateNextPose(Pose pose, Velocity velocity, double dt) {
        double dx = 0.0;
        double dy = 0.0;
        if (std::abs(velocity.angularVelocity) * dt < 1e-6){
            dy = velocity.linearVelocity * dt * std::sin(pose.theta);
            dx = velocity.linearVelocity * dt *std::cos(pose.theta);
            pose.theta +=velocity.angularVelocity * dt;
        }
        else{
            double theta_next = pose.theta + velocity.angularVelocity * dt;
            dy = (velocity.linearVelocity/velocity.angularVelocity)
            * (std::cos(pose.theta)-std::cos(theta_next));
            dx = (velocity.linearVelocity/velocity.angularVelocity) *
            (std::sin(theta_next)-std::sin(pose.theta));
            pose.theta = theta_next;
        }

        return(Pose{pose.x + dx, pose.y + dy, pose.theta});
    }
}//namespace

    Pose Robot::robotNextPose(double dt) const {
        return(calculateNextPose(pose_, velocity_, dt));
    }

    RectangleCorners Robot::getRobotCorners() const {
        return calculateRectangleCorners(pose_, size_);
    }

    void Robot::setVelocity(double linear_velocity, double angular_velocity){
        if (!std::isfinite(linear_velocity) || !std::isfinite(angular_velocity)){
            throw std::invalid_argument("Robot velocities must be finite");
        }
        if (angular_velocity < -2.0 * pi || angular_velocity > 2.0 * pi){
            throw std::invalid_argument("Robot angular velocity must be in range [-2*pi, 2*pi]");
        }
        velocity_.linearVelocity = linear_velocity;
        velocity_.angularVelocity = angular_velocity;
    }

    void Robot::move(double dt) {
        pose_ = calculateNextPose(pose_, velocity_, dt);
        addPointToTrajectory(pose_);
    }

    DistanceSensorID Robot::addDistanceSensor (DistanceSensor distance_sensor){
        distance_sensors_.push_back(distance_sensor);
        return(distance_sensors_.size() - 1);
    }
}//namespace robot_simulation
