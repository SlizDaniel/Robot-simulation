#pragma once
#include <robot_simulation/robot.hpp>
#include <robot_simulation/environment.hpp>

namespace robot_simulation{
    enum class StopReason{
        None,
        OutOfEnvironment,
        ObstacleCollision
    };

    struct SimulationUpdate {
        StopReason stop_reason;
        bool step_accepted;
    };

    class Simulation{
        private:
        double step_time_;
        Robot& robot_;
        Environment& environment_;

        public:
        Simulation(double dt, Robot& robot, Environment& environment);

        SimulationUpdate update();
    };
}// namespace robot_simulation