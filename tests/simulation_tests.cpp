#include <gtest/gtest.h>
#include <robot_simulation/environment.hpp>
#include <robot_simulation/math_constans.hpp>
#include <robot_simulation/obstacle.hpp>
#include <robot_simulation/robot.hpp>
#include <robot_simulation/simulation.hpp>
#include <robot_simulation/types.hpp>
#include <stdexcept>

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
}
