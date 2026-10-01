#include <gtest/gtest.h>
#include <robot_simulation/distance_sensor.hpp>
#include <robot_simulation/math_constans.hpp>
#include <robot_simulation/types.hpp>
#include <limits>
#include <stdexcept>

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
}
