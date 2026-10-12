#pragma once
#include <robot_simulation/types.hpp>
#include <robot_simulation/robot.hpp>
#include <robot_simulation/environment.hpp>
#include <robot_simulation/distance_sensors_front_setup.hpp>

namespace robot_simulation{
    class Controller{
        public:
        virtual ~Controller() = default;
        virtual Velocity calculateVelocity(Robot& robot, Environment& environment, 
            int ray_count, DistanceSensorsFrontSetup distnce_front_sensors) const = 0;
    };
}