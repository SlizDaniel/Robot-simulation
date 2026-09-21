#include <robot_simulation/simulation.hpp>
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
        Robot predicted_robot = robot_;
        predicted_robot.move(step_time_);
        if (!(environment_.isRobotInside(predicted_robot))){
            robot_.stop();
            return(SimulationUpdate{StopReason::OutOfEnvironment, false});
        }
        if (environment_.collides(predicted_robot)){
            robot_.stop();
            return(SimulationUpdate{StopReason::ObstacleCollision, false});
        }
        robot_.move(step_time_);
        return(SimulationUpdate{StopReason::None, true});
    }
}//namespace robot_simulation