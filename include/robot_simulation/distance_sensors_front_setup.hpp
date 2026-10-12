#pragma once
#include <robot_simulation/distance_sensor.hpp>
#include <robot_simulation/robot.hpp>
#include <array>

namespace robot_simulation {
    enum class InitialPositions{
        FRONTLEFT,
        FRONT,
        FRONTRIGHT
    };

    struct SensorInfo{
        InitialPositions position;
        DistanceSensorID id;
    };

    struct DistanceSensorsFrontSetup{
        SensorInfo front_lef_sensor;
        SensorInfo front_sensor;
        SensorInfo front_right_sensor;
    };

    DistanceSensorsFrontSetup setupDistanceSensorsOnRobot(Robot& robot,
        double single_sensor_field_of_view);
}//robot_simulation