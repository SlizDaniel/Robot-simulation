#include <robot_simulation/distance_sensor.hpp>
#include <robot_simulation/math_constans.hpp>
#include <stdexcept>
#include <cmath>

namespace robot_simulation{
    DistanceSensor::DistanceSensor(double min_distance, double max_distance,
        Pose to_robot_position, double field_of_view):
        max_distance_(max_distance),
        min_distance_(min_distance),
        field_of_view_(field_of_view),
        relativeToRobotPose_(to_robot_position){
            if(std::isnan(min_distance_) || std::isnan(max_distance_) ||
                std::isnan(field_of_view_) || std::isnan(relativeToRobotPose_.x) ||
                std::isnan(relativeToRobotPose_.y) || std::isnan(relativeToRobotPose_.theta)){
                throw std::invalid_argument("Sensor parameters must not be NaN");
            }
            if(min_distance_ < 0 || max_distance_ < 0){
                throw std::invalid_argument("Sensor distances must be positive numbers");
            }
            if(max_distance_ < min_distance_){
                throw std::invalid_argument("You must have mixed up max and min distance");
            }
            if(field_of_view_ <= 0 || field_of_view_ > pi){
                throw std::invalid_argument("Field of view must be in range (0, pi]");
            }
        }

    Pose DistanceSensor::getWorldPose(const Pose& robot_pose) const {
        const double sin_theta = std::sin(robot_pose.theta);
        const double cos_theta = std::cos(robot_pose.theta);
        const double dx = relativeToRobotPose_.x * cos_theta -
                    sin_theta * relativeToRobotPose_.y;
        const double dy = relativeToRobotPose_.y * cos_theta +
                    sin_theta * relativeToRobotPose_.x;
        return (Pose{robot_pose.x + dx, robot_pose.y + dy, robot_pose.theta + relativeToRobotPose_.theta});
    }
}//namespace robot_simulation
