#include <gtest/gtest.h>
#include <robot_simulation/distance_sensors_front_setup.hpp>
#include <robot_simulation/math_constants.hpp>
#include <limits>
#include <stdexcept>

#include "helpers.hpp"

namespace robot_simulation {
    TEST(DistanceSensorsFrontSetupTests, AddsThreeSensorsWithExpectedOrientations) {
        Robot robot(Pose{}, Velocity{}, Rectangle{1.0, 2.0});

        const DistanceSensorsFrontSetup setup = setupDistanceSensorsOnRobot(robot, pi / 4.0);
        const std::vector<DistanceSensor>& sensors = robot.getDistanceSensors();

        ASSERT_EQ(sensors.size(), 3U);
        EXPECT_EQ(setup.front_lef_sensor.position, InitialPositions::FRONTLEFT);
        EXPECT_EQ(setup.front_sensor.position, InitialPositions::FRONT);
        EXPECT_EQ(setup.front_right_sensor.position, InitialPositions::FRONTRIGHT);
        EXPECT_EQ(setup.front_lef_sensor.id, 0U);
        EXPECT_EQ(setup.front_sensor.id, 1U);
        EXPECT_EQ(setup.front_right_sensor.id, 2U);
        EXPECT_DOUBLE_EQ(sensors[setup.front_lef_sensor.id].getSensorMinDistance(), 0.0);
        EXPECT_DOUBLE_EQ(sensors[setup.front_sensor.id].getSensorMinDistance(), 0.0);
        EXPECT_DOUBLE_EQ(sensors[setup.front_right_sensor.id].getSensorMinDistance(), 0.0);
        EXPECT_NEAR(sensors[setup.front_lef_sensor.id].getRelativeToRobotPose().theta, pi / 4.0, tolerance);
        EXPECT_NEAR(sensors[setup.front_sensor.id].getRelativeToRobotPose().theta, 0.0, tolerance);
        EXPECT_NEAR(sensors[setup.front_right_sensor.id].getRelativeToRobotPose().theta, -pi / 4.0, tolerance);
    }

    TEST(DistanceSensorsFrontSetupTests, PreservesExistingSensorsAndReturnsFollowingIDs) {
        Robot robot(Pose{}, Velocity{}, Rectangle{1.0, 2.0});
        const DistanceSensor existing_sensor(0.0, 5.0, Pose{1.0, 0.0, 0.0}, pi / 2.0);
        robot.addDistanceSensor(existing_sensor);

        const DistanceSensorsFrontSetup setup = setupDistanceSensorsOnRobot(robot, 0.0);

        EXPECT_EQ(robot.getDistanceSensors().size(), 4U);
        EXPECT_EQ(setup.front_lef_sensor.id, 1U);
        EXPECT_EQ(setup.front_sensor.id, 2U);
        EXPECT_EQ(setup.front_right_sensor.id, 3U);
        EXPECT_DOUBLE_EQ(robot.getDistanceSensors()[0].getSensorMaxDistance(), 5.0);
    }

    TEST(DistanceSensorsFrontSetupTests, RejectsNonFiniteRotation) {
        Robot robot(Pose{}, Velocity{}, Rectangle{1.0, 2.0});
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double infinity = std::numeric_limits<double>::infinity();

        EXPECT_THROW(setupDistanceSensorsOnRobot(robot, nan), std::invalid_argument);
        EXPECT_THROW(setupDistanceSensorsOnRobot(robot, infinity), std::invalid_argument);
        EXPECT_TRUE(robot.getDistanceSensors().empty());
    }
}
