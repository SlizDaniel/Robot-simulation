#pragma once
#include <robot_simulation/controller.hpp>
#include <robot_simulation/robot.hpp>
#include <robot_simulation/environment.hpp>
#include <robot_simulation/distance_sensors_front_setup.hpp>

namespace robot_simulation{
    class AvoidObstaclesController: public Controller {
        private:
        double safe_distance_;
        double critical_distance_;
        double velocity_gain_critical_;

        public:
        AvoidObstaclesController (double safe_distance, double critical_distance,
            double velocity_gain_critical = 0.2);

        Velocity calculateVelocity(Robot& robot, Environment& environment, 
            int ray_count, DistanceSensorsFrontSetup distnce_front_sensors) const override;
    
        const double getSafeDistance() const {return safe_distance_;}

        const double getCriticalDistance() const {return critical_distance_;}
        };
}//robot_simulation