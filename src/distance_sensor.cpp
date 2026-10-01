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
}//namespace robot_simulation
