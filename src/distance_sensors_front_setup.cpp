#include <robot_simulation/distance_sensors_front_setup.hpp>
#include <robot_simulation/robot.hpp>
#include <cmath>
#include <stdexcept>

namespace robot_simulation {
    DistanceSensorsFrontSetup setupDistanceSensorsOnRobot(Robot& robot, 
        double single_sensor_field_of_view){
        if (!std::isfinite(single_sensor_field_of_view)){
            throw std::invalid_argument("Sensor rotation must be finite");
        }
        DistanceSensor front_left_sensor(0.0, 20.0, Pose{}, pi / 2.0);
        DistanceSensor front_sensor(0.0, 20.0, Pose{}, pi / 2.0);
        DistanceSensor front_right_sensor(0.0, 20.0, Pose{}, pi / 2.0);
        front_left_sensor.setToRobotRotation(single_sensor_field_of_view);
        front_right_sensor.setToRobotRotation(-single_sensor_field_of_view);
        DistanceSensorID front_left_sensor_id = robot.addDistanceSensor(front_left_sensor);
        DistanceSensorID front_sensor_id = robot.addDistanceSensor(front_sensor);
        DistanceSensorID front_right_sensor_id = robot.addDistanceSensor(front_right_sensor);
        return {{InitialPositions::FRONTLEFT, front_left_sensor_id},
                {InitialPositions::FRONT, front_sensor_id},
                {InitialPositions::FRONTRIGHT, front_right_sensor_id}};
    }
}
