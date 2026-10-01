#include <gtest/gtest.h>
#include <robot_simulation/distance_sensor.hpp>
#include <robot_simulation/math_constans.hpp>
#include <robot_simulation/types.hpp>
#include <limits>
#include <stdexcept>

#include "helpers.hpp"

namespace robot_simulation{
    TEST(DistanceSensorTests, StoresConstructorArguments){
        const Pose relative_pose {1.0, 2.0, 0.3};

        const DistanceSensor sensor(0.5, 10.0, relative_pose, pi / 3.0);

        const Pose actual_pose = sensor.getRelativeToRobotPose();

        EXPECT_DOUBLE_EQ(sensor.getSensorMinDistance(), 0.5);
        EXPECT_DOUBLE_EQ(sensor.getSensorMaxDistance(), 10.0);
        EXPECT_DOUBLE_EQ(actual_pose.x, relative_pose.x);
        EXPECT_DOUBLE_EQ(actual_pose.y, relative_pose.y);
        EXPECT_DOUBLE_EQ(actual_pose.theta, relative_pose.theta);
    }

    TEST(DistanceSensorTests, RejectsNegativeMinDistance){
        EXPECT_THROW(DistanceSensor(-0.1, 10.0, Pose{}, pi / 3.0), std::invalid_argument);
    }

    TEST(DistanceSensorTests, RejectsNegativeMaxDistance){
        EXPECT_THROW(DistanceSensor(0.0, -10.0, Pose{}, pi / 3.0), std::invalid_argument);
    }

    TEST(DistanceSensorTests, RejectsMaxDistanceSmallerThanMinDistance){
        EXPECT_THROW(DistanceSensor(5.0, 4.0, Pose{}, pi / 3.0), std::invalid_argument);
    }

    TEST(DistanceSensorTests, RejectsNaNParameters){
        const double nan = std::numeric_limits<double>::quiet_NaN();

        EXPECT_THROW(DistanceSensor(nan, 10.0, Pose{}, pi / 3.0), std::invalid_argument);
        EXPECT_THROW(DistanceSensor(0.0, nan, Pose{}, pi / 3.0), std::invalid_argument);
        EXPECT_THROW(DistanceSensor(0.0, 10.0, Pose{}, nan), std::invalid_argument);
        EXPECT_THROW(DistanceSensor(0.0, 10.0, Pose{nan, 0.0, 0.0}, pi / 3.0), std::invalid_argument);
        EXPECT_THROW(DistanceSensor(0.0, 10.0, Pose{0.0, nan, 0.0}, pi / 3.0), std::invalid_argument);
        EXPECT_THROW(DistanceSensor(0.0, 10.0, Pose{0.0, 0.0, nan}, pi / 3.0), std::invalid_argument);
    }

    TEST(DistanceSensorTests, RejectsZeroFieldOfView){
        EXPECT_THROW(DistanceSensor(0.0, 10.0, Pose{}, 0.0), std::invalid_argument);
    }

    TEST(DistanceSensorTests, RejectsNegativeFieldOfView){
        EXPECT_THROW(DistanceSensor(0.0, 10.0, Pose{}, -0.1), std::invalid_argument);
    }

    TEST(DistanceSensorTests, RejectsFieldOfViewGreaterThanPi){
        EXPECT_THROW(DistanceSensor(0.0, 10.0, Pose{}, pi + 0.1), std::invalid_argument);
    }

    TEST(DistanceSensorTests, AcceptsFieldOfViewEqualToPi){
        EXPECT_NO_THROW(DistanceSensor(0.0, 10.0, Pose{}, pi));
    }

    TEST(DistanceSensorTests, CalculatesWorldPoseWithoutRobotRotation){
        const DistanceSensor sensor(0.0, 10.0, Pose{1.0, 2.0, 0.3}, pi / 3.0);
        const Pose robot_pose {10.0, 5.0, 0.0};

        const Pose sensor_pose = sensor.getWorldPose(robot_pose);

        EXPECT_NEAR(sensor_pose.x, 11.0, tolerance);
        EXPECT_NEAR(sensor_pose.y, 7.0, tolerance);
        EXPECT_NEAR(sensor_pose.theta, 0.3, tolerance);
    }

    TEST(DistanceSensorTests, RotatesForwardOffsetWithRobot){
        const DistanceSensor sensor(0.0, 10.0, Pose{1.0, 0.0, 0.0}, pi / 3.0);
        const Pose robot_pose {10.0, 5.0, pi / 2.0};

        const Pose sensor_pose = sensor.getWorldPose(robot_pose);

        EXPECT_NEAR(sensor_pose.x, 10.0, tolerance);
        EXPECT_NEAR(sensor_pose.y, 6.0, tolerance);
        EXPECT_NEAR(sensor_pose.theta, pi / 2.0, tolerance);
    }

    TEST(DistanceSensorTests, RotatesSideOffsetWithRobot){
        const DistanceSensor sensor(0.0, 10.0, Pose{0.0, 1.0, 0.0}, pi / 3.0);
        const Pose robot_pose {0.0, 0.0, pi / 2.0};

        const Pose sensor_pose = sensor.getWorldPose(robot_pose);

        EXPECT_NEAR(sensor_pose.x, -1.0, tolerance);
        EXPECT_NEAR(sensor_pose.y, 0.0, tolerance);
        EXPECT_NEAR(sensor_pose.theta, pi / 2.0, tolerance);
    }

    TEST(DistanceSensorTests, AddsSensorAndRobotOrientations){
        const DistanceSensor sensor(0.0, 10.0, Pose{0.0, 0.0, pi / 4.0}, pi / 3.0);
        const Pose robot_pose {0.0, 0.0, pi / 2.0};

        const Pose sensor_pose = sensor.getWorldPose(robot_pose);

        EXPECT_NEAR(sensor_pose.theta, 3.0 * pi / 4.0, tolerance);
    }
}
