#include <robot_simulation/simulation.hpp>
#include <robot_simulation/geometry.hpp>
#include <stdexcept>

namespace robot_simulation{
    Simulation::Simulation(double dt, Robot& robot, Environment& environment, int sensor_ray_count):
        step_time_(dt),
        robot_(robot),
        environment_(environment),
        sensor_ray_count_(sensor_ray_count)
        {
            if (step_time_ <= 0.0){
                throw std::invalid_argument("Simulation step time must be positive number");
            }
            if (sensor_ray_count_ <= 0 || sensor_ray_count_ % 2 == 0){
                throw std::invalid_argument("Sensor ray count must be positive odd number");
            }
        }

    std::vector<RaycastResult> Simulation::calculateRaycastResults() const {
        std::vector<RaycastResult> results;
        for(const DistanceSensor& sensor : robot_.getDistanceSensors()){
            results.push_back(sensor.nearestObjectDetected(
                environment_, sensor_ray_count_, robot_.getPose()));
        }
        return results;
    }

    SimulationUpdate Simulation::update(){
        Pose next_pose = robot_.robotNextPose(step_time_);
        const RectangleCorners predicted_corners = calculateRectangleCorners(next_pose, robot_.getSize());
        if (!(environment_.isRobotInside(predicted_corners))){
            robot_.stop();
            return(SimulationUpdate{StopReason::OutOfEnvironment, false, calculateRaycastResults()});
        }
        if (environment_.collides(predicted_corners, next_pose)){
            robot_.stop();
            return(SimulationUpdate{StopReason::ObstacleCollision, false, calculateRaycastResults()});
        }
        robot_.move(step_time_);
        simulation_step_ += 1;

        return(SimulationUpdate{StopReason::None, true, calculateRaycastResults()});
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
