#include <gtest/gtest.h>
#include <robot_simulation/avoid_obstacles_controller.hpp>
#include <robot_simulation/math_constants.hpp>
#include <robot_simulation/simulation.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "helpers.hpp"

namespace robot_simulation {
    namespace {
        DistanceSensorsFrontSetup addFrontSensors(Robot& robot, double min_distance = 0.0) {
            const DistanceSensor left_sensor(min_distance, 20.0, Pose{0.0, 0.0, pi / 4.0}, pi / 2.0);
            const DistanceSensor front_sensor(min_distance, 20.0, Pose{}, pi / 2.0);
            const DistanceSensor right_sensor(min_distance, 20.0, Pose{0.0, 0.0, -pi / 4.0}, pi / 2.0);
            return {
                {InitialPositions::FRONTLEFT, robot.addDistanceSensor(left_sensor)},
                {InitialPositions::FRONT, robot.addDistanceSensor(front_sensor)},
                {InitialPositions::FRONTRIGHT, robot.addDistanceSensor(right_sensor)}
            };
        }

        Environment criticalObstacleEnvironment() {
            Environment environment(Rectangle{100.0, 100.0});
            environment.addObstacle(Obstacle(Pose{4.0, 0.0, 0.0}, Rectangle{1.0, 1.0}));
            return environment;
        }
    }

    TEST(AvoidObstaclesControllerTests, StoresConstructorDistances) {
        const AvoidObstaclesController controller(10.0, 5.0);

        EXPECT_DOUBLE_EQ(controller.getSafeDistance(), 10.0);
        EXPECT_DOUBLE_EQ(controller.getCriticalDistance(), 5.0);
    }

    TEST(AvoidObstaclesControllerTests, RejectsInvalidConstructorParameters) {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double infinity = std::numeric_limits<double>::infinity();

        EXPECT_THROW(AvoidObstaclesController(-0.1, 0.0), std::invalid_argument);
        EXPECT_THROW(AvoidObstaclesController(1.0, -0.1), std::invalid_argument);
        EXPECT_THROW(AvoidObstaclesController(4.0, 5.0), std::invalid_argument);
        EXPECT_THROW(AvoidObstaclesController(5.0, 1.0, -0.1), std::invalid_argument);
        EXPECT_THROW(AvoidObstaclesController(5.0, 1.0, 0.51), std::invalid_argument);
        EXPECT_THROW(AvoidObstaclesController(nan, 1.0), std::invalid_argument);
        EXPECT_THROW(AvoidObstaclesController(5.0, infinity), std::invalid_argument);
    }

    TEST(AvoidObstaclesControllerTests, AcceptsBoundaryConstructorParameters) {
        EXPECT_NO_THROW(AvoidObstaclesController(5.0, 5.0, 0.0));
        EXPECT_NO_THROW(AvoidObstaclesController(5.0, 5.0, 0.5));
    }

    TEST(AvoidObstaclesControllerTests, LeavesVelocityUnchangedWithoutDetection) {
        Robot robot(Pose{}, Velocity{2.0, -0.25}, Rectangle{1.0, 2.0});
        const DistanceSensorsFrontSetup front_sensors = addFrontSensors(robot);
        Environment environment(Rectangle{100.0, 100.0});
        const AvoidObstaclesController controller(10.0, 5.0);

        const Velocity velocity = controller.calculateVelocity(robot, environment, 1, front_sensors);

        EXPECT_DOUBLE_EQ(velocity.linearVelocity, 2.0);
        EXPECT_DOUBLE_EQ(velocity.angularVelocity, -0.25);
        EXPECT_DOUBLE_EQ(robot.getVelocity().linearVelocity, 2.0);
        EXPECT_DOUBLE_EQ(robot.getVelocity().angularVelocity, -0.25);
    }

    TEST(AvoidObstaclesControllerTests, StopsAndTurnsRightInCriticalZoneWhenSidesAreEqual) {
        Robot robot(Pose{}, Velocity{2.0, 0.0}, Rectangle{1.0, 2.0});
        const DistanceSensorsFrontSetup front_sensors = addFrontSensors(robot);
        Environment environment = criticalObstacleEnvironment();
        const AvoidObstaclesController controller(10.0, 5.0);

        const Velocity velocity = controller.calculateVelocity(robot, environment, 1, front_sensors);

        EXPECT_DOUBLE_EQ(velocity.linearVelocity, 0.0);
        EXPECT_NEAR(velocity.angularVelocity, -pi / 2.0, tolerance);
    }

    TEST(AvoidObstaclesControllerTests, TurnsLeftWhenLeftSideIsClearerInCriticalZone) {
        Robot robot(Pose{}, Velocity{2.0, 0.0}, Rectangle{1.0, 2.0});
        const DistanceSensorsFrontSetup front_sensors = addFrontSensors(robot);
        Environment environment = criticalObstacleEnvironment();
        environment.addObstacle(Obstacle(
            Pose{3.0 * std::cos(-pi / 4.0), 3.0 * std::sin(-pi / 4.0), 0.0}, Rectangle{1.0, 1.0}));
        const AvoidObstaclesController controller(10.0, 5.0);

        const Velocity velocity = controller.calculateVelocity(robot, environment, 1, front_sensors);

        EXPECT_DOUBLE_EQ(velocity.linearVelocity, 0.0);
        EXPECT_NEAR(velocity.angularVelocity, pi / 2.0, tolerance);
    }

    TEST(AvoidObstaclesControllerTests, TurnsRightWhenRightSideIsClearerInCriticalZone) {
        Robot robot(Pose{}, Velocity{2.0, 0.0}, Rectangle{1.0, 2.0});
        const DistanceSensorsFrontSetup front_sensors = addFrontSensors(robot);
        Environment environment = criticalObstacleEnvironment();
        environment.addObstacle(Obstacle(
            Pose{3.0 * std::cos(pi / 4.0), 3.0 * std::sin(pi / 4.0), 0.0}, Rectangle{1.0, 1.0}));
        const AvoidObstaclesController controller(10.0, 5.0);

        const Velocity velocity = controller.calculateVelocity(robot, environment, 1, front_sensors);

        EXPECT_DOUBLE_EQ(velocity.linearVelocity, 0.0);
        EXPECT_NEAR(velocity.angularVelocity, -pi / 2.0, tolerance);
    }

    TEST(AvoidObstaclesControllerTests, IgnoresOutOfRangeDetection) {
        Robot robot(Pose{}, Velocity{2.0, 0.25}, Rectangle{1.0, 2.0});
        const DistanceSensorsFrontSetup front_sensors = addFrontSensors(robot, 5.0);
        Environment environment = criticalObstacleEnvironment();
        const AvoidObstaclesController controller(10.0, 5.0);

        const Velocity velocity = controller.calculateVelocity(robot, environment, 1, front_sensors);

        EXPECT_DOUBLE_EQ(velocity.linearVelocity, 2.0);
        EXPECT_DOUBLE_EQ(velocity.angularVelocity, 0.25);
    }

    TEST(AvoidObstaclesControllerTests, SlowsDownInSafeZone) {
        Robot robot(Pose{}, Velocity{2.0, 0.0}, Rectangle{1.0, 2.0});
        const DistanceSensorsFrontSetup front_sensors = addFrontSensors(robot);
        Environment environment(Rectangle{100.0, 100.0});
        environment.addObstacle(Obstacle(Pose{7.0, 0.0, 0.0}, Rectangle{1.0, 1.0}));
        const AvoidObstaclesController controller(10.0, 5.0);

        const Velocity velocity = controller.calculateVelocity(robot, environment, 1, front_sensors);

        EXPECT_NEAR(velocity.linearVelocity, 1.6, tolerance);
        EXPECT_LT(velocity.linearVelocity, robot.getVelocity().linearVelocity);
    }

    TEST(AvoidObstaclesControllerTests, UsesSafeZoneAtCriticalDistanceBoundary) {
        Robot robot(Pose{}, Velocity{2.0, 0.0}, Rectangle{1.0, 2.0});
        const DistanceSensorsFrontSetup front_sensors = addFrontSensors(robot);
        Environment environment(Rectangle{100.0, 100.0});
        environment.addObstacle(Obstacle(Pose{5.5, 0.0, 0.0}, Rectangle{1.0, 1.0}));
        const AvoidObstaclesController controller(10.0, 5.0);

        const Velocity velocity = controller.calculateVelocity(robot, environment, 1, front_sensors);

        EXPECT_NEAR(velocity.linearVelocity, 1.6, tolerance);
        EXPECT_DOUBLE_EQ(velocity.angularVelocity, 0.0);
    }

    TEST(AvoidObstaclesControllerTests, LeavesVelocityUnchangedAtSafeDistanceBoundary) {
        Robot robot(Pose{}, Velocity{2.0, 0.0}, Rectangle{1.0, 2.0});
        const DistanceSensorsFrontSetup front_sensors = addFrontSensors(robot);
        Environment environment(Rectangle{100.0, 100.0});
        environment.addObstacle(Obstacle(Pose{10.5, 0.0, 0.0}, Rectangle{1.0, 1.0}));
        const AvoidObstaclesController controller(10.0, 5.0);

        const Velocity velocity = controller.calculateVelocity(robot, environment, 1, front_sensors);

        EXPECT_DOUBLE_EQ(velocity.linearVelocity, 2.0);
        EXPECT_DOUBLE_EQ(velocity.angularVelocity, 0.0);
    }

    TEST(AvoidObstaclesControllerTests, ClampsAngularVelocityInSafeZone) {
        Robot robot(Pose{}, Velocity{2.0, 2.0 * pi}, Rectangle{1.0, 2.0});
        const DistanceSensorsFrontSetup front_sensors = addFrontSensors(robot);
        Environment environment(Rectangle{100.0, 100.0});
        environment.addObstacle(Obstacle(Pose{7.0, 0.0, 0.0}, Rectangle{1.0, 1.0}));
        const AvoidObstaclesController controller(10.0, 5.0);

        const Velocity velocity = controller.calculateVelocity(robot, environment, 1, front_sensors);

        EXPECT_DOUBLE_EQ(velocity.angularVelocity, 2.0 * pi);
    }

    TEST(AvoidObstaclesControllerTests, RejectsInvalidSensorIDsAndRayCounts) {
        Robot robot(Pose{}, Velocity{}, Rectangle{1.0, 2.0});
        const DistanceSensorsFrontSetup front_sensors = addFrontSensors(robot);
        const DistanceSensorsFrontSetup invalid_sensors{
            {InitialPositions::FRONTLEFT, 3U}, front_sensors.front_sensor, front_sensors.front_right_sensor};
        Environment environment(Rectangle{100.0, 100.0});
        const AvoidObstaclesController controller(10.0, 5.0);

        EXPECT_THROW(controller.calculateVelocity(robot, environment, 1, invalid_sensors), std::invalid_argument);
        EXPECT_THROW(controller.calculateVelocity(robot, environment, 0, front_sensors), std::invalid_argument);
        EXPECT_THROW(controller.calculateVelocity(robot, environment, 2, front_sensors), std::invalid_argument);
    }

    TEST(AvoidObstaclesControllerTests, IntegratesWithFrontSensorSetup) {
        Robot robot(Pose{}, Velocity{2.0, 0.0}, Rectangle{1.0, 2.0});
        const DistanceSensorsFrontSetup front_sensors = setupDistanceSensorsOnRobot(robot, pi / 4.0);
        Environment environment = criticalObstacleEnvironment();
        const AvoidObstaclesController controller(10.0, 5.0);

        const Velocity velocity = controller.calculateVelocity(robot, environment, 1, front_sensors);

        EXPECT_DOUBLE_EQ(velocity.linearVelocity, 0.0);
        EXPECT_NEAR(velocity.angularVelocity, -pi / 2.0, tolerance);
    }

    TEST(AvoidObstaclesControllerTests, DetectsObstacleCloserThanPreviousDefaultMinimumRange) {
        Robot robot(Pose{}, Velocity{2.0, 0.0}, Rectangle{0.2, 0.2});
        const DistanceSensorsFrontSetup front_sensors = setupDistanceSensorsOnRobot(robot, pi / 4.0);
        Environment environment(Rectangle{100.0, 100.0});
        environment.addObstacle(Obstacle(Pose{1.0, 0.0, 0.0}, Rectangle{0.2, 0.2}));
        const AvoidObstaclesController controller(10.0, 5.0);

        const Velocity velocity = controller.calculateVelocity(robot, environment, 1, front_sensors);

        EXPECT_DOUBLE_EQ(velocity.linearVelocity, 0.0);
    }

    TEST(AvoidObstaclesControllerTests, StopsLinearMotionDuringSimulationUpdateInCriticalZone) {
        Robot robot(Pose{}, Velocity{2.0, 0.0}, Rectangle{1.0, 2.0});
        const DistanceSensorsFrontSetup front_sensors = addFrontSensors(robot);
        Environment environment = criticalObstacleEnvironment();
        const AvoidObstaclesController controller(10.0, 5.0);
        const Velocity velocity = controller.calculateVelocity(robot, environment, 1, front_sensors);
        robot.setVelocity(velocity.linearVelocity, velocity.angularVelocity);
        Simulation simulation(0.5, robot, environment, 1);

        const SimulationUpdate update = simulation.update();
        const Pose pose = robot.getPose();

        EXPECT_TRUE(update.step_accepted);
        EXPECT_NEAR(pose.x, 0.0, tolerance);
        EXPECT_NEAR(pose.y, 0.0, tolerance);
        EXPECT_NEAR(pose.theta, -pi / 4.0, tolerance);
    }

    TEST(AvoidObstaclesControllerTests, ReturnsCruiseCommandAfterCriticalObstacleLeavesFrontDirection) {
        Robot robot(Pose{}, Velocity{2.0, 0.0}, Rectangle{1.0, 2.0});
        const DistanceSensorsFrontSetup front_sensors = addFrontSensors(robot);
        Environment environment = criticalObstacleEnvironment();
        const AvoidObstaclesController controller(10.0, 5.0);
        const Velocity critical_velocity = controller.calculateVelocity(robot, environment, 1, front_sensors);
        robot.setVelocity(critical_velocity.linearVelocity, critical_velocity.angularVelocity);
        Simulation simulation(0.5, robot, environment, 1);
        ASSERT_TRUE(simulation.update().step_accepted);

        robot.setVelocity(2.0, 0.0);
        const Velocity velocity = controller.calculateVelocity(robot, environment, 1, front_sensors);

        EXPECT_DOUBLE_EQ(velocity.linearVelocity, 2.0);
        EXPECT_DOUBLE_EQ(velocity.angularVelocity, 0.0);
    }
}
