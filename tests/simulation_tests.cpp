#include <gtest/gtest.h>
#include <robot_simulation/environment.hpp>
#include <robot_simulation/math_constans.hpp>
#include <robot_simulation/obstacle.hpp>
#include <robot_simulation/robot.hpp>
#include <robot_simulation/simulation.hpp>
#include <robot_simulation/types.hpp>
#include <stdexcept>
#include <vector>

#include "helpers.hpp"

namespace robot_simulation{
    TEST(SimulationTests, RejectsZeroStepTime){
        const Pose pose {0.0, 0.0, 0.0};
        const Velocity velocity {0.0, 0.0};
        const Rectangle robot_size {2.0, 4.0};
        const Rectangle environment_size {20.0, 30.0};
        Robot robot(pose, velocity, robot_size);
        Environment environment(environment_size);

        EXPECT_THROW(Simulation(0.0, robot, environment), std::invalid_argument);
    }

    TEST(SimulationTests, RejectsNegativeStepTime){
        const Pose pose {0.0, 0.0, 0.0};
        const Velocity velocity {0.0, 0.0};
        const Rectangle robot_size {2.0, 4.0};
        const Rectangle environment_size {20.0, 30.0};
        Robot robot(pose, velocity, robot_size);
        Environment environment(environment_size);

        EXPECT_THROW(Simulation(-0.5, robot, environment), std::invalid_argument);
    }

    TEST(SimulationTests, MovesRobotWhenPredictedPositionIsSafe){
        const Pose pose {0.0, 0.0, 0.0};
        const Velocity velocity {2.0, 0.0};
        const Rectangle robot_size {2.0, 4.0};
        const Rectangle environment_size {20.0, 30.0};
        Robot robot(pose, velocity, robot_size);
        Environment environment(environment_size);
        Simulation simulation(0.5, robot, environment);

        const SimulationUpdate update = simulation.update();
        const Pose actual_pose = robot.getPose();
        const Velocity actual_velocity = robot.getVelocity();

        EXPECT_EQ(update.stop_reason, StopReason::None);
        EXPECT_TRUE(update.step_accepted);
        EXPECT_NEAR(actual_pose.x, 1.0, tolerance);
        EXPECT_NEAR(actual_pose.y, 0.0, tolerance);
        EXPECT_NEAR(actual_pose.theta, 0.0, tolerance);
        EXPECT_DOUBLE_EQ(actual_velocity.linearVelocity, velocity.linearVelocity);
        EXPECT_DOUBLE_EQ(actual_velocity.angularVelocity, velocity.angularVelocity);
    }

    TEST(SimulationTests, MovesRobotAlongCircularPathWhenPredictedPositionIsSafe){
        const Pose pose {0.0, 0.0, 0.0};
        const Velocity velocity {1.0, 1.0};
        const Rectangle robot_size {2.0, 4.0};
        const Rectangle environment_size {30.0, 30.0};
        Robot robot(pose, velocity, robot_size);
        Environment environment(environment_size);
        Simulation simulation(pi / 2.0, robot, environment);

        const SimulationUpdate update = simulation.update();
        const Pose actual_pose = robot.getPose();

        EXPECT_EQ(update.stop_reason, StopReason::None);
        EXPECT_TRUE(update.step_accepted);
        EXPECT_NEAR(actual_pose.x, 1.0, tolerance);
        EXPECT_NEAR(actual_pose.y, 1.0, tolerance);
        EXPECT_NEAR(actual_pose.theta, pi / 2.0, tolerance);
    }

    TEST(SimulationTests, StopsRobotBeforePredictedObstacleCollision){
        const Pose robot_pose {0.0, 0.0, 0.0};
        const Velocity velocity {2.0, 0.0};
        const Rectangle robot_size {2.0, 4.0};
        const Rectangle environment_size {20.0, 30.0};
        const Pose obstacle_pose {4.5, 0.0, 0.0};
        const Rectangle obstacle_size {2.0, 4.0};
        Robot robot(robot_pose, velocity, robot_size);
        Obstacle obstacle(obstacle_pose, obstacle_size);
        Environment environment(environment_size);
        environment.addObstacle(obstacle);
        Simulation simulation(0.5, robot, environment);

        const SimulationUpdate update = simulation.update();
        const Pose actual_pose = robot.getPose();
        const Velocity actual_velocity = robot.getVelocity();

        EXPECT_EQ(update.stop_reason, StopReason::ObstacleCollision);
        EXPECT_FALSE(update.step_accepted);
        EXPECT_NEAR(actual_pose.x, robot_pose.x, tolerance);
        EXPECT_NEAR(actual_pose.y, robot_pose.y, tolerance);
        EXPECT_NEAR(actual_pose.theta, robot_pose.theta, tolerance);
        EXPECT_DOUBLE_EQ(actual_velocity.linearVelocity, 0.0);
        EXPECT_DOUBLE_EQ(actual_velocity.angularVelocity, 0.0);
    }

    TEST(SimulationTests, StopsRobotBeforeLeavingEnvironment){
        const Pose robot_pose {2.0, 0.0, 0.0};
        const Velocity velocity {4.0, 0.0};
        const Rectangle robot_size {2.0, 4.0};
        const Rectangle environment_size {4.0, 10.0};
        Robot robot(robot_pose, velocity, robot_size);
        Environment environment(environment_size);
        Simulation simulation(0.5, robot, environment);

        const SimulationUpdate update = simulation.update();
        const Pose actual_pose = robot.getPose();
        const Velocity actual_velocity = robot.getVelocity();

        EXPECT_EQ(update.stop_reason, StopReason::OutOfEnvironment);
        EXPECT_FALSE(update.step_accepted);
        EXPECT_NEAR(actual_pose.x, robot_pose.x, tolerance);
        EXPECT_NEAR(actual_pose.y, robot_pose.y, tolerance);
        EXPECT_NEAR(actual_pose.theta, robot_pose.theta, tolerance);
        EXPECT_DOUBLE_EQ(actual_velocity.linearVelocity, 0.0);
        EXPECT_DOUBLE_EQ(actual_velocity.angularVelocity, 0.0);
    }

    TEST(SimulationTests, RunZeroStepsDoesNotChangeSimulation){
        const Pose pose {0.0, 0.0, 0.0};
        const Velocity velocity {2.0, 0.0};
        const Rectangle robot_size {2.0, 4.0};
        const Rectangle environment_size {20.0, 30.0};
        Robot robot(pose, velocity, robot_size);
        Environment environment(environment_size);
        Simulation simulation(0.5, robot, environment);

        const SimulationResult result = simulation.runSteps(0);
        const Pose actual_pose = robot.getPose();

        EXPECT_TRUE(result.simulation_completed);
        EXPECT_EQ(result.executed_steps, 0U);
        EXPECT_EQ(result.stop_reason, StopReason::None);
        EXPECT_EQ(simulation.getSimulationStep(), 0U);
        EXPECT_DOUBLE_EQ(actual_pose.x, pose.x);
        EXPECT_DOUBLE_EQ(actual_pose.y, pose.y);
        EXPECT_DOUBLE_EQ(actual_pose.theta, pose.theta);
        EXPECT_EQ(robot.getTrajectory().size(), 1U);
    }

    TEST(SimulationTests, RunsRequestedNumberOfSteps){
        const Pose pose {0.0, 0.0, 0.0};
        const Velocity velocity {2.0, 0.0};
        const Rectangle robot_size {2.0, 4.0};
        const Rectangle environment_size {20.0, 30.0};
        Robot robot(pose, velocity, robot_size);
        Environment environment(environment_size);
        Simulation simulation(0.5, robot, environment);

        const SimulationResult result = simulation.runSteps(3);
        const Pose actual_pose = robot.getPose();

        EXPECT_TRUE(result.simulation_completed);
        EXPECT_EQ(result.executed_steps, 3U);
        EXPECT_EQ(result.stop_reason, StopReason::None);
        EXPECT_NEAR(actual_pose.x, 3.0, tolerance);
        EXPECT_NEAR(actual_pose.y, 0.0, tolerance);
        EXPECT_NEAR(actual_pose.theta, 0.0, tolerance);
    }

    TEST(SimulationTests, StopsRunBeforeObstacleCollision){
        const Pose robot_pose {0.0, 0.0, 0.0};
        const Velocity velocity {2.0, 0.0};
        const Rectangle robot_size {2.0, 4.0};
        const Rectangle environment_size {20.0, 30.0};
        const Pose obstacle_pose {6.5, 0.0, 0.0};
        const Rectangle obstacle_size {2.0, 4.0};
        Robot robot(robot_pose, velocity, robot_size);
        Obstacle obstacle(obstacle_pose, obstacle_size);
        Environment environment(environment_size);
        environment.addObstacle(obstacle);
        Simulation simulation(0.5, robot, environment);

        const SimulationResult result = simulation.runSteps(5);
        const Pose actual_pose = robot.getPose();

        EXPECT_FALSE(result.simulation_completed);
        EXPECT_EQ(result.executed_steps, 2U);
        EXPECT_EQ(result.stop_reason, StopReason::ObstacleCollision);
        EXPECT_NEAR(actual_pose.x, 2.0, tolerance);
        EXPECT_NEAR(actual_pose.y, 0.0, tolerance);
        EXPECT_NEAR(actual_pose.theta, 0.0, tolerance);
    }

    TEST(SimulationTests, StopsRunBeforeLeavingEnvironment){
        const Pose pose {0.0, 0.0, 0.0};
        const Velocity velocity {2.0, 0.0};
        const Rectangle robot_size {2.0, 4.0};
        const Rectangle environment_size {10.0, 12.0};
        Robot robot(pose, velocity, robot_size);
        Environment environment(environment_size);
        Simulation simulation(1.0, robot, environment);

        const SimulationResult result = simulation.runSteps(5);
        const Pose actual_pose = robot.getPose();

        EXPECT_FALSE(result.simulation_completed);
        EXPECT_EQ(result.executed_steps, 2U);
        EXPECT_EQ(result.stop_reason, StopReason::OutOfEnvironment);
        EXPECT_NEAR(actual_pose.x, 4.0, tolerance);
        EXPECT_NEAR(actual_pose.y, 0.0, tolerance);
        EXPECT_NEAR(actual_pose.theta, 0.0, tolerance);
    }

    TEST(SimulationTests, ExecutedStepsCountsOnlyCurrentRun){
        const Pose pose {0.0, 0.0, 0.0};
        const Velocity velocity {1.0, 0.0};
        const Rectangle robot_size {2.0, 4.0};
        const Rectangle environment_size {20.0, 30.0};
        Robot robot(pose, velocity, robot_size);
        Environment environment(environment_size);
        Simulation simulation(0.5, robot, environment);

        const SimulationResult first_result = simulation.runSteps(2);
        const SimulationResult second_result = simulation.runSteps(3);

        EXPECT_EQ(first_result.executed_steps, 2U);
        EXPECT_EQ(second_result.executed_steps, 3U);
    }

    TEST(SimulationTests, SimulationStepCountsAllAcceptedSteps){
        const Pose pose {0.0, 0.0, 0.0};
        const Velocity velocity {1.0, 0.0};
        const Rectangle robot_size {2.0, 4.0};
        const Rectangle environment_size {20.0, 30.0};
        Robot robot(pose, velocity, robot_size);
        Environment environment(environment_size);
        Simulation simulation(0.5, robot, environment);

        simulation.update();
        simulation.runSteps(2);

        EXPECT_EQ(simulation.getSimulationStep(), 3U);
    }

    TEST(SimulationTests, SupportsMultipleRunStepsCalls){
        const Pose pose {0.0, 0.0, 0.0};
        const Velocity velocity {2.0, 0.0};
        const Rectangle robot_size {2.0, 4.0};
        const Rectangle environment_size {20.0, 30.0};
        Robot robot(pose, velocity, robot_size);
        Environment environment(environment_size);
        Simulation simulation(0.5, robot, environment);

        const SimulationResult first_result = simulation.runSteps(2);
        const Pose pose_after_first_run = robot.getPose();
        const SimulationResult second_result = simulation.runSteps(2);
        const Pose pose_after_second_run = robot.getPose();

        EXPECT_TRUE(first_result.simulation_completed);
        EXPECT_TRUE(second_result.simulation_completed);
        EXPECT_NEAR(pose_after_first_run.x, 2.0, tolerance);
        EXPECT_NEAR(pose_after_second_run.x, 4.0, tolerance);
        EXPECT_EQ(simulation.getSimulationStep(), 4U);
    }

    TEST(SimulationTests, TrajectoryContainsOnlyAcceptedSteps){
        const Pose robot_pose {0.0, 0.0, 0.0};
        const Velocity velocity {2.0, 0.0};
        const Rectangle robot_size {2.0, 4.0};
        const Rectangle environment_size {20.0, 30.0};
        const Pose obstacle_pose {6.5, 0.0, 0.0};
        const Rectangle obstacle_size {2.0, 4.0};
        Robot robot(robot_pose, velocity, robot_size);
        Obstacle obstacle(obstacle_pose, obstacle_size);
        Environment environment(environment_size);
        environment.addObstacle(obstacle);
        Simulation simulation(0.5, robot, environment);

        const SimulationResult result = simulation.runSteps(5);

        const std::vector<Pose>& trajectory = robot.getTrajectory();
        ASSERT_FALSE(result.simulation_completed);
        ASSERT_EQ(trajectory.size(), result.executed_steps + 1U);
        EXPECT_NEAR(trajectory[0].x, 0.0, tolerance);
        EXPECT_NEAR(trajectory[1].x, 1.0, tolerance);
        EXPECT_NEAR(trajectory[2].x, 2.0, tolerance);
        EXPECT_NEAR(trajectory.back().x, robot.getPose().x, tolerance);
    }

    TEST(SimulationTests, SimulationTimeCountsOnlyAcceptedSteps){
        const Pose pose {0.0, 0.0, 0.0};
        const Velocity velocity {2.0, 0.0};
        const Rectangle robot_size {2.0, 4.0};
        const Rectangle environment_size {10.0, 10.0};
        Robot robot(pose, velocity, robot_size);
        Environment environment(environment_size);
        Simulation simulation(0.5, robot, environment);

        EXPECT_DOUBLE_EQ(simulation.getSimulationTime(), 0.0);
        const SimulationResult result = simulation.runSteps(5);

        ASSERT_FALSE(result.simulation_completed);
        EXPECT_EQ(result.executed_steps, 3U);
        EXPECT_DOUBLE_EQ(simulation.getSimulationTime(), 1.5);
    }
}
