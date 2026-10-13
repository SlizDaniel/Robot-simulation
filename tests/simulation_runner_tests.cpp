#include <gtest/gtest.h>
#include <robot_simulation/simulation_runner.hpp>

#include <stdexcept>

#include "helpers.hpp"

namespace robot_simulation {
namespace {
SceneConfig makeConfig() {
    return {
        {100.0, 100.0},
        {{0.0, 0.0, 0.0}, {2.0, 2.0}, {2.0, 0.0}},
        {},
        0.5,
        3,
        {0.0, 0.0, 0.2}
    };
}
}

TEST(SimulationRunnerTests, StartsWithConfigurationState) {
    const SceneConfig config = makeConfig();

    const SimulationRunner runner(config);

    const Pose pose = runner.getRobotPose();
    const Velocity velocity = runner.getRobotVelocity();
    EXPECT_DOUBLE_EQ(pose.x, config.robot.pose.x);
    EXPECT_DOUBLE_EQ(pose.y, config.robot.pose.y);
    EXPECT_DOUBLE_EQ(pose.theta, config.robot.pose.theta);
    EXPECT_DOUBLE_EQ(velocity.linearVelocity, config.robot.velocity.linearVelocity);
    EXPECT_DOUBLE_EQ(velocity.angularVelocity, config.robot.velocity.angularVelocity);
    EXPECT_EQ(runner.getSimulationStep(), 0U);
    EXPECT_DOUBLE_EQ(runner.getSimulationTime(), 0.0);
    EXPECT_FALSE(runner.isStopped());
}

TEST(SimulationRunnerTests, AdvancesWithConfiguredVelocityAndReportsFrontSensors) {
    const SceneConfig config = makeConfig();
    SimulationRunner runner(config);

    const SimulationUpdate update = runner.update();

    EXPECT_TRUE(update.step_accepted);
    EXPECT_EQ(update.stop_reason, StopReason::None);
    EXPECT_EQ(update.raycast_results.size(), 3U);
    EXPECT_NEAR(runner.getRobotPose().x, 1.0, tolerance);
    EXPECT_DOUBLE_EQ(runner.getRobotPose().y, 0.0);
    EXPECT_EQ(runner.getSimulationStep(), 1U);
    EXPECT_DOUBLE_EQ(runner.getSimulationTime(), 0.5);
    EXPECT_FALSE(runner.isStopped());
}

TEST(SimulationRunnerTests, StopsAfterPredictedObstacleCollisionAndRejectsFurtherUpdates) {
    SceneConfig config = makeConfig();
    config.dt = 1.0;
    config.obstacles.push_back({{4.0, 0.0, 0.0}, {2.0, 2.0}});
    SimulationRunner runner(config);

    const SimulationUpdate update = runner.update();

    EXPECT_FALSE(update.step_accepted);
    EXPECT_EQ(update.stop_reason, StopReason::ObstacleCollision);
    EXPECT_TRUE(runner.isStopped());
    EXPECT_EQ(runner.getSimulationStep(), 0U);
    EXPECT_DOUBLE_EQ(runner.getRobotVelocity().linearVelocity, 0.0);
    EXPECT_DOUBLE_EQ(runner.getRobotVelocity().angularVelocity, 0.0);
    EXPECT_THROW(runner.update(), std::logic_error);
}

TEST(SimulationRunnerTests, StopsBeforeLeavingEnvironment) {
    SceneConfig config = makeConfig();
    config.environment_size = {10.0, 10.0};
    config.robot.pose = {3.0, 0.0, 0.0};
    config.dt = 1.0;
    SimulationRunner runner(config);

    const SimulationUpdate update = runner.update();

    EXPECT_FALSE(update.step_accepted);
    EXPECT_EQ(update.stop_reason, StopReason::OutOfEnvironment);
    EXPECT_TRUE(runner.isStopped());
    EXPECT_DOUBLE_EQ(runner.getRobotPose().x, 3.0);
    EXPECT_EQ(runner.getSimulationStep(), 0U);
}

TEST(SimulationRunnerTests, ResetRestoresInitialSessionAfterStop) {
    SceneConfig config = makeConfig();
    config.dt = 1.0;
    config.obstacles.push_back({{4.0, 0.0, 0.0}, {2.0, 2.0}});
    SimulationRunner runner(config);
    runner.update();

    runner.reset();

    EXPECT_FALSE(runner.isStopped());
    EXPECT_EQ(runner.getSimulationStep(), 0U);
    EXPECT_DOUBLE_EQ(runner.getSimulationTime(), 0.0);
    EXPECT_DOUBLE_EQ(runner.getRobotPose().x, config.robot.pose.x);
    EXPECT_DOUBLE_EQ(runner.getRobotVelocity().linearVelocity, config.robot.velocity.linearVelocity);
}

TEST(SimulationRunnerTests, RejectsRobotOutsideInitialEnvironment) {
    SceneConfig config = makeConfig();
    config.environment_size = {10.0, 10.0};
    config.robot.pose = {5.0, 0.0, 0.0};

    EXPECT_THROW({ SimulationRunner runner(config); }, std::invalid_argument);
}

TEST(SimulationRunnerTests, RejectsInitialRobotObstacleCollision) {
    SceneConfig config = makeConfig();
    config.obstacles.push_back({{0.0, 0.0, 0.0}, {2.0, 2.0}});

    EXPECT_THROW({ SimulationRunner runner(config); }, std::invalid_argument);
}

TEST(SimulationRunnerTests, RejectsInvalidSimulationRayCount) {
    SceneConfig config = makeConfig();
    config.ray_count = 2;

    EXPECT_THROW({ SimulationRunner runner(config); }, std::invalid_argument);
}
}  // namespace robot_simulation
