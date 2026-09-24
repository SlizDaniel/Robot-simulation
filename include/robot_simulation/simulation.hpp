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

    struct SimulationResult {
        std::size_t executed_steps;
        bool simulation_completed;
        StopReason stop_reason;
    };

    class Simulation{
        private:
        double step_time_;
        Robot& robot_;
        Environment& environment_;
        std::size_t simulation_step_ = 0;

        public:
        Simulation(double dt, Robot& robot, Environment& environment);

        SimulationUpdate update();

        SimulationResult runSteps(std::size_t step_count);

        std::size_t getSimulationStep() const {return simulation_step_;}

        double getSimulationTime() const {return static_cast<double>(simulation_step_) * step_time_;}
    };
}// namespace robot_simulation