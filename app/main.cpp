#include <robot_simulation/environment.hpp>
#include <robot_simulation/obstacle.hpp>
#include <robot_simulation/robot.hpp>
#include <robot_simulation/simulation.hpp>

#include <cstddef>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace {
    using robot_simulation::Pose;
    using robot_simulation::Simulation;
    using robot_simulation::SimulationResult;
    using robot_simulation::StopReason;

    constexpr int separator_width = 64;

    std::string stopReasonToString(StopReason reason){
        switch(reason){
            case StopReason::None:
                return "none";
            case StopReason::OutOfEnvironment:
                return "out of environment";
            case StopReason::ObstacleCollision:
                return "obstacle collision";
        }
        return "unknown";
    }

    void printSeparator(char character = '-'){
        std::cout << std::string(separator_width, character) << '\n';
    }

    void printPose(const std::string& label, const Pose& pose){
        std::cout << std::left << std::setw(22) << label
                  << "x = " << std::right << std::setw(7) << pose.x
                  << ", y = " << std::setw(7) << pose.y
                  << ", theta = " << std::setw(7) << pose.theta << '\n';
    }

    void printPhaseResult(
        const std::string& title,
        const SimulationResult& result,
        const Simulation& simulation,
        const Pose& pose
    ){
        printSeparator();
        std::cout << title << '\n';
        printSeparator();
        std::cout << std::left
                  << std::setw(22) << "Status"
                  << (result.simulation_completed ? "completed" : "stopped") << '\n'
                  << std::setw(22) << "Executed steps"
                  << result.executed_steps << '\n'
                  << std::setw(22) << "Stop reason"
                  << stopReasonToString(result.stop_reason) << '\n'
                  << std::setw(22) << "Total steps"
                  << simulation.getSimulationStep() << '\n'
                  << std::setw(22) << "Simulation time"
                  << simulation.getSimulationTime() << " s\n";
        printPose("Robot pose", pose);
    }

    void printTrajectory(const std::vector<Pose>& trajectory, double step_time){
        printSeparator();
        std::cout << "TRAJECTORY HISTORY\n";
        printSeparator();
        std::cout << std::right
                  << std::setw(7) << "Point"
                  << std::setw(12) << "Time [s]"
                  << std::setw(12) << "X"
                  << std::setw(12) << "Y"
                  << std::setw(12) << "Theta" << '\n';
        printSeparator('.');

        for(std::size_t i = 0; i < trajectory.size(); ++i){
            const Pose& pose = trajectory[i];
            std::cout << std::setw(7) << i
                      << std::setw(12) << static_cast<double>(i) * step_time
                      << std::setw(12) << pose.x
                      << std::setw(12) << pose.y
                      << std::setw(12) << pose.theta << '\n';
        }
    }
}

int main(){
    using namespace robot_simulation;

    constexpr double step_time = 0.5;
    const Rectangle environment_size {12.0, 30.0};
    const Rectangle robot_size {1.5, 2.0};
    const Rectangle obstacle_size {4.0, 2.0};

    Environment environment(environment_size);
    environment.addObstacle(Obstacle(
        Pose{8.0, 0.0, 0.0},
        obstacle_size
    ));

    Robot robot(
        Pose{-8.0, 0.0, 0.0},
        Velocity{2.0, 0.25},
        robot_size
    );
    Simulation simulation(step_time, robot, environment);

    std::cout << std::fixed << std::setprecision(2);
    printSeparator('=');
    std::cout << "                 ROBOT SIMULATOR v0.2 DEMO\n";
    printSeparator('=');
    std::cout << std::left
              << std::setw(22) << "Environment"
              << environment_size.length << " x " << environment_size.width << '\n'
              << std::setw(22) << "Robot size"
              << robot_size.length << " x " << robot_size.width << '\n'
              << std::setw(22) << "Obstacle position"
              << "x = 8.00, y = 0.00\n"
              << std::setw(22) << "Simulation step"
              << step_time << " s\n";
    printPose("Initial robot pose", robot.getPose());

    const SimulationResult first_phase = simulation.runSteps(4);
    printPhaseResult(
        "PHASE 1: linear 2.00, angular 0.25 rad/s",
        first_phase,
        simulation,
        robot.getPose()
    );

    robot.setVelocity(4.0, 0.15);
    const SimulationResult second_phase = simulation.runSteps(10);
    printPhaseResult(
        "PHASE 2: linear 4.00, angular 0.15 rad/s",
        second_phase,
        simulation,
        robot.getPose()
    );

    printTrajectory(robot.getTrajectory(), step_time);
    printSeparator('=');
    std::cout << "Demo finished. The robot remained at the last safe position.\n";
    printSeparator('=');

    return 0;
}
