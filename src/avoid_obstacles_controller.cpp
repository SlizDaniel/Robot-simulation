#include <robot_simulation/avoid_obstacles_controller.hpp>
#include <robot_simulation/distance_sensors_front_setup.hpp>
#include <robot_simulation/robot.hpp>
#include <algorithm>
#include <cmath>
#include <vector>
#include <stdexcept>

namespace robot_simulation {
    namespace {
        constexpr double critical_turn_rate = pi / 2.0;
    }

    AvoidObstaclesController::AvoidObstaclesController (double safe_distance, double critical_distance, double velocity_gain):
                            safe_distance_(safe_distance), critical_distance_(critical_distance), velocity_gain_critical_(velocity_gain){
        if (!std::isfinite(safe_distance_) || !std::isfinite(critical_distance_) ||
            !std::isfinite(velocity_gain_critical_)){
            throw std::invalid_argument("Controller parameters must be finite");
        }
        if (safe_distance_ < 0 || critical_distance_ < 0){
            throw std::invalid_argument("Controller distances must be non-negative numbers!");
        }
        if (safe_distance_ < critical_distance_){
            throw std::invalid_argument("Safe distance should be higher than critical distance!");
        }
        if (velocity_gain_critical_ < 0.0 || velocity_gain_critical_ > 0.5){
            throw std::invalid_argument("Velocity critical distance gain should be in range [0, 0.5]");
        }
    }
    namespace{
        double detectedDistance(const RaycastResult& result){
            if (!result.object_detected || result.detection_type != DetectionType::OBJECTDETECTED){
                return infinity;
            }
            return result.distance_to_object;
        }

        Velocity setNewVelocities(double left_sensor_distance, double right_sensor_distance,
                                  Velocity current_velocity, double velocity_gain_critical){
            if (left_sensor_distance > right_sensor_distance){
                if (current_velocity.angularVelocity <= 0){
                    return {-velocity_gain_critical * current_velocity.linearVelocity,
                        -current_velocity.angularVelocity * velocity_gain_critical / 4};
                }
                else {
                    return {-velocity_gain_critical * current_velocity.linearVelocity,
                        -pi * velocity_gain_critical / 2};
                }
            }
            if (left_sensor_distance <= right_sensor_distance) {
                if (current_velocity.angularVelocity <= 0) {
                    return {-velocity_gain_critical * current_velocity.linearVelocity,
                        current_velocity.angularVelocity * velocity_gain_critical / 4};
                } 
                else {
                    return {-velocity_gain_critical * current_velocity.linearVelocity,
                        pi * velocity_gain_critical / 2};
                }
            }
        return {0.0, 0.0};
        }
    }//namespace

    Velocity AvoidObstaclesController::calculateVelocity(Robot& robot, 
            Environment& environment, int ray_count, 
            DistanceSensorsFrontSetup distance_front_sensors) const {
        Velocity current_robot_velocity = robot.getVelocity();
        Pose robot_position = robot.getPose();
        const std::vector<DistanceSensor>& robot_distance_sensors = robot.getDistanceSensors();
        SensorInfo front_sensor = distance_front_sensors.front_sensor;
        SensorInfo left_sensor = distance_front_sensors.front_lef_sensor;
        SensorInfo right_sensor = distance_front_sensors.front_right_sensor;
        if (front_sensor.id >= robot_distance_sensors.size() ||
            left_sensor.id >= robot_distance_sensors.size() ||
            right_sensor.id >= robot_distance_sensors.size()){
            throw std::invalid_argument("Distance sensor ID is invalid");
        }
        DistanceSensor front_robot_sensor = robot_distance_sensors[front_sensor.id];
        DistanceSensor front_left_robot_sensor = robot_distance_sensors[left_sensor.id];
        DistanceSensor front_right_robot_sensor = robot_distance_sensors[right_sensor.id];
        RaycastResult front_sensor_result = front_robot_sensor.nearestObjectDetected
                                            (environment, ray_count, robot_position);
        RaycastResult left_sensor_result = front_left_robot_sensor.nearestObjectDetected
                                            (environment, ray_count, robot_position);
        RaycastResult right_sensor_result = front_right_robot_sensor.nearestObjectDetected
                                            (environment, ray_count, robot_position);
        const double front_sensor_distance = detectedDistance(front_sensor_result);
        const double left_sensor_distance = detectedDistance(left_sensor_result);
        const double right_sensor_distance = detectedDistance(right_sensor_result);
        Velocity velocity_correction = {0.0, 0.0};
        if (front_sensor_distance < critical_distance_){
            double angular_velocity = 0.0;
            if (left_sensor_distance > right_sensor_distance){
                angular_velocity = critical_turn_rate;
            }
            else{
                angular_velocity = -critical_turn_rate;
            }
            return {0.0, angular_velocity};
        }
        else if (front_sensor_distance < safe_distance_){
            velocity_correction = setNewVelocities(left_sensor_distance, right_sensor_distance,
                current_robot_velocity, velocity_gain_critical_);
        }
        return {current_robot_velocity.linearVelocity + velocity_correction.linearVelocity,
                std::clamp(current_robot_velocity.angularVelocity + velocity_correction.angularVelocity,
                    -2.0 * pi, 2.0 * pi)};
    }
}//robot_simulation
