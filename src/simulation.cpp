#include <robot_simulation/simulation.hpp>
#include <robot_simulation/geometry.hpp>
#include <stdexcept>

namespace robot_simulation{
    Simulation::Simulation(double dt, Robot& robot, Environment& environment):
        step_time_(dt),
        robot_(robot),
        environment_(environment)
        {
            if (step_time_ <= 0.0){
                throw std::invalid_argument("Simulation step time must be positive number");
            }
        }
    SimulationUpdate Simulation::update(){
        Pose next_pose = robot_.robotNextPose(step_time_);
        const RectangleCorners predicted_corners = calculateRectangleCorners(next_pose, robot_.getSize());
        if (!(environment_.isRobotInside(predicted_corners))){
            robot_.stop();
            return(SimulationUpdate{StopReason::OutOfEnvironment, false});
        }
        if (environment_.collides(predicted_corners, next_pose)){
            robot_.stop();
            return(SimulationUpdate{StopReason::ObstacleCollision, false});
        }
        robot_.move(step_time_);
        simulation_step_ += 1;

        return(SimulationUpdate{StopReason::None, true});
    }

    SimulationResult Simulation::runSteps(std::size_t step_count){
        std::size_t executed_steps = 0;
        while(executed_steps < step_count){
            SimulationUpdate step_result = update();
            if (step_result.step_accepted != true){
                return(SimulationResult{executed_steps, false, step_result.stop_reason});
            }
            executed_steps += 1;
        }
        return(SimulationResult{executed_steps, true, StopReason::None});
    }
}//namespace robot_simulation