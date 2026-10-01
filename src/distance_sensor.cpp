#include <robot_simulation/distance_sensor.hpp>
#include <robot_simulation/math_constans.hpp>
#include <robot_simulation/geometry.hpp>
#include <robot_simulation/types.hpp>
#include <robot_simulation/environment.hpp>
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

    RaycastResult DistanceSensor::nearestObstacleDetected (const Environment& environment, int ray_count,
            const Pose& robot_pose) const{
        if (ray_count % 2 == 0){
            throw std::invalid_argument("ray_count must be odd number for better results");
        }
        double distance_to_nearest_obstacle = infinity;
        DetectionType detection_type = DetectionType::NOOBJECTDETECTED;
        const std::vector<Obstacle>& obstacles = environment.getObstacles();
        Pose sensor_world_pose = getWorldPose(robot_pose);
        std::array<double, 2> min_max = {getSensorMinDistance(), getSensorMaxDistance()};
        double ray_step = 0.0;
        if (ray_count <= 0){
            throw std::invalid_argument("Raycount must be positive");
        }
        else if (ray_count == 1){
            for(const Obstacle& obstacle: obstacles){
                    RaycastResult result = calculateDistanceSensorObstacle({sensor_world_pose.x, sensor_world_pose.y,
                        sensor_world_pose.theta}, min_max, obstacle);
                        if (result.distance_to_object < distance_to_nearest_obstacle){
                            distance_to_nearest_obstacle = result.distance_to_object;
                            detection_type = result.detection_type;
                        }
                }
                if (distance_to_nearest_obstacle != infinity && detection_type == DetectionType::OBJECTDETECTED){
                    return {true, distance_to_nearest_obstacle, detection_type};
                }
            return {false, infinity, detection_type};
        }
        else{
            ray_step = field_of_view_/(ray_count - 1);
        }
        for (int ray_index = 0; ray_index < ray_count; ++ray_index){
                const double current_angle = -field_of_view_/2 + ray_index * ray_step;
                for(const Obstacle& obstacle: obstacles){
                    RaycastResult result = calculateDistanceSensorObstacle({sensor_world_pose.x, sensor_world_pose.y,
                        sensor_world_pose.theta + current_angle}, min_max, obstacle);
                        if (result.distance_to_object < distance_to_nearest_obstacle){
                            distance_to_nearest_obstacle = result.distance_to_object;
                            detection_type = result.detection_type;
                        }
                }
            }
        if (distance_to_nearest_obstacle != infinity && detection_type == DetectionType::OBJECTDETECTED){
            return {true, distance_to_nearest_obstacle, detection_type};
        }
        return {false, infinity, detection_type};
    }
}//namespace robot_simulation
