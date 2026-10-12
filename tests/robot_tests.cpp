#include <gtest/gtest.h>
#include <robot_simulation/distance_sensor.hpp>
#include <robot_simulation/robot.hpp>
#include <robot_simulation/types.hpp>
#include "helpers.hpp"
#include <robot_simulation/math_constants.hpp>
#include <limits>
#include <stdexcept>
#include <vector>

namespace robot_simulation{
    TEST(RobotTests, StoresConstructorArguments){
        const Pose pose {5.0, 7.0, 0.2};
        const Rectangle size {2.0, 4.0};
        const Velocity velocity {3.0, 1.0};

        const Robot robot(pose, velocity, size);

        const Pose actual_pose = robot.getPose();
        const Velocity actual_velocity = robot.getVelocity();
        const Rectangle actual_size = robot.getSize();

        EXPECT_DOUBLE_EQ(actual_pose.x, pose.x);
        EXPECT_DOUBLE_EQ(actual_pose.y, pose.y);
        EXPECT_DOUBLE_EQ(actual_pose.theta, pose.theta);
        EXPECT_DOUBLE_EQ(actual_velocity.linearVelocity, velocity.linearVelocity);
        EXPECT_DOUBLE_EQ(actual_velocity.angularVelocity, velocity.angularVelocity);
        EXPECT_DOUBLE_EQ(actual_size.width, size.width);
        EXPECT_DOUBLE_EQ(actual_size.length, size.length);
    }

    TEST(RobotTests, RejectsZeroWidth){
        const Pose pose {0.0, 0.0, 0.0};
        const Velocity velocity {0.0, 0.0};
        const Rectangle size {0.0, 4.0};

        EXPECT_THROW(Robot(pose, velocity, size), std::invalid_argument);
    }

    TEST(RobotTests, RejectsZeroLength){
        const Pose pose {0.0, 0.0, 0.0};
        const Velocity velocity {0.0, 0.0};
        const Rectangle size {2.0, 0.0};

        EXPECT_THROW(Robot(pose, velocity, size), std::invalid_argument);
    }

    TEST(RobotTests, RejectsNegativeWidth){
        const Pose pose {0.0, 0.0, 0.0};
        const Velocity velocity {0.0, 0.0};
        const Rectangle size {-2.0, 4.0};

        EXPECT_THROW(Robot(pose, velocity, size), std::invalid_argument);
    }

    TEST(RobotTests, RejectsNegativeLength){
        const Pose pose {0.0, 0.0, 0.0};
        const Velocity velocity {0.0, 0.0};
        const Rectangle size {2.0, -4.0};

        EXPECT_THROW(Robot(pose, velocity, size), std::invalid_argument);
    }

    TEST(RobotTests, RejectsNonFiniteConstructorParameters){
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double infinity = std::numeric_limits<double>::infinity();

        EXPECT_THROW(Robot(Pose{nan, 0.0, 0.0}, Velocity{}, Rectangle{2.0, 4.0}), std::invalid_argument);
        EXPECT_THROW(Robot(Pose{}, Velocity{infinity, 0.0}, Rectangle{2.0, 4.0}), std::invalid_argument);
        EXPECT_THROW(Robot(Pose{}, Velocity{}, Rectangle{2.0, infinity}), std::invalid_argument);
    }

    TEST(RobotTests, AcceptsAngularVelocityAtLimits){
        const Pose pose {0.0, 0.0, 0.0};
        const Rectangle size {2.0, 4.0};

        EXPECT_NO_THROW(Robot(pose, Velocity{0.0, -2.0 * pi}, size));
        EXPECT_NO_THROW(Robot(pose, Velocity{0.0, 2.0 * pi}, size));
    }

    TEST(RobotTests, RejectsAngularVelocityOutsideLimits){
        const Pose pose {0.0, 0.0, 0.0};
        const Rectangle size {2.0, 4.0};

        EXPECT_THROW(Robot(pose, Velocity{0.0, -2.0 * pi - 0.01}, size), std::invalid_argument);
        EXPECT_THROW(Robot(pose, Velocity{0.0, 2.0 * pi + 0.01}, size), std::invalid_argument);
    }

    TEST(RobotTests, CornersCalculation){
        Pose pose {5.0, 7.0, 0.0};
        Rectangle size {2.0, 4.0};
        Velocity velocity {0.0, 0.0};

        Robot robot(pose, velocity, size);

        const RectangleCorners robot_corners = robot.getRobotCorners();
        const RectangleCorners expected{
            Point{ 7.0,  8.0},
            Point{ 7.0, 6.0},
            Point{3, 6},
            Point{3,  8}
        };
        expectCornersNear(robot_corners, expected, tolerance);
    }

    TEST(RobotTests, RotatedCornersCalculation){
        const Pose pose {0.0, 0.0, pi / 2.0};
        const Rectangle size {2.0, 4.0};
        const Velocity velocity {0.0, 0.0};

        const Robot robot(pose, velocity, size);

        const RectangleCorners robot_corners = robot.getRobotCorners();
        const RectangleCorners expected{
            Point{-1.0,  2.0},
            Point{ 1.0,  2.0},
            Point{ 1.0, -2.0},
            Point{-1.0, -2.0}
        };
        expectCornersNear(robot_corners, expected, tolerance);
    }

    TEST(RobotTests, MovesAlongXAxisWhenHeadingIsZero){
        const Pose pose {0.0, 0.0, 0.0};
        const Rectangle size {2.0, 4.0};
        const Velocity velocity {2.0, 0.0};

        Robot robot(pose, velocity, size);
        robot.move(0.5);

        const Pose actual_pose = robot.getPose();

        EXPECT_NEAR(actual_pose.x, 1.0, tolerance);
        EXPECT_NEAR(actual_pose.y, 0.0, tolerance);
        EXPECT_NEAR(actual_pose.theta, 0.0, tolerance);
    }

    TEST(RobotTests, MovesAlongYAxisWhenHeadingIsNinetyDegrees){
        const Pose pose {0.0, 0.0, pi / 2.0};
        const Rectangle size {2.0, 4.0};
        const Velocity velocity {2.0, 0.0};

        Robot robot(pose, velocity, size);
        robot.move(0.5);

        const Pose actual_pose = robot.getPose();

        EXPECT_NEAR(actual_pose.x, 0.0, tolerance);
        EXPECT_NEAR(actual_pose.y, 1.0, tolerance);
        EXPECT_NEAR(actual_pose.theta, pi / 2.0, tolerance);
    }

    TEST(RobotTests, FollowsCircularMotionWhenTurning){
        const Pose pose {0.0, 0.0, 0.0};
        const Rectangle size {2.0, 4.0};
        const Velocity velocity {1.0, 1.0};

        Robot robot(pose, velocity, size);
        robot.move(pi / 2.0);

        const Pose actual_pose = robot.getPose();

        EXPECT_NEAR(actual_pose.x, 1.0, tolerance);
        EXPECT_NEAR(actual_pose.y, 1.0, tolerance);
        EXPECT_NEAR(actual_pose.theta, pi / 2.0, tolerance);
    }

    TEST(RobotTests, StopSetsBothVelocitiesToZero){
        const Pose pose {0.0, 0.0, 0.0};
        const Rectangle size {2.0, 4.0};
        const Velocity velocity {4.0, 2.0};

        Robot robot(pose, velocity, size);
        robot.stop();

        const Velocity actual_velocity = robot.getVelocity();

        EXPECT_DOUBLE_EQ(actual_velocity.linearVelocity, 0.0);
        EXPECT_DOUBLE_EQ(actual_velocity.angularVelocity, 0.0);
    }

    TEST(RobotTests, TrajectoryStartsWithInitialPose){
        const Pose pose {5.0, 7.0, 0.2};
        const Rectangle size {2.0, 4.0};
        const Velocity velocity {3.0, 1.0};

        const Robot robot(pose, velocity, size);

        const std::vector<Pose>& trajectory = robot.getTrajectory();

        ASSERT_EQ(trajectory.size(), 1U);
        EXPECT_DOUBLE_EQ(trajectory[0].x, pose.x);
        EXPECT_DOUBLE_EQ(trajectory[0].y, pose.y);
        EXPECT_DOUBLE_EQ(trajectory[0].theta, pose.theta);
    }

    TEST(RobotTests, MoveAddsExactlyOnePointToTrajectory){
        const Pose pose {0.0, 0.0, 0.0};
        const Rectangle size {2.0, 4.0};
        const Velocity velocity {2.0, 0.0};
        Robot robot(pose, velocity, size);
        const std::size_t initial_trajectory_size = robot.getTrajectory().size();

        robot.move(0.5);

        const std::vector<Pose>& trajectory = robot.getTrajectory();
        ASSERT_EQ(trajectory.size(), initial_trajectory_size + 1U);
        EXPECT_NEAR(trajectory.back().x, 1.0, tolerance);
        EXPECT_NEAR(trajectory.back().y, 0.0, tolerance);
        EXPECT_NEAR(trajectory.back().theta, 0.0, tolerance);
    }

    TEST(RobotTests, NextPosePredictionDoesNotModifyPoseOrTrajectory){
        const Pose pose {1.0, 2.0, 0.3};
        const Rectangle size {2.0, 4.0};
        const Velocity velocity {2.0, 0.5};
        Robot robot(pose, velocity, size);
        const std::size_t initial_trajectory_size = robot.getTrajectory().size();

        const Pose predicted_pose = robot.robotNextPose(0.5);
        const Pose actual_pose = robot.getPose();

        EXPECT_NE(predicted_pose.x, actual_pose.x);
        EXPECT_DOUBLE_EQ(actual_pose.x, pose.x);
        EXPECT_DOUBLE_EQ(actual_pose.y, pose.y);
        EXPECT_DOUBLE_EQ(actual_pose.theta, pose.theta);
        EXPECT_EQ(robot.getTrajectory().size(), initial_trajectory_size);
    }

    TEST(RobotTests, PredictedPoseMatchesSubsequentMove){
        const Pose pose {1.0, -2.0, 0.3};
        const Rectangle size {2.0, 4.0};
        const Velocity velocity {2.0, 0.5};
        Robot robot(pose, velocity, size);

        const Pose predicted_pose = robot.robotNextPose(0.5);
        robot.move(0.5);
        const Pose actual_pose = robot.getPose();

        EXPECT_NEAR(actual_pose.x, predicted_pose.x, tolerance);
        EXPECT_NEAR(actual_pose.y, predicted_pose.y, tolerance);
        EXPECT_NEAR(actual_pose.theta, predicted_pose.theta, tolerance);
    }

    TEST(RobotTests, SetVelocityUpdatesBothVelocities){
        const Pose pose {0.0, 0.0, 0.0};
        const Rectangle size {2.0, 4.0};
        const Velocity velocity {1.0, 0.5};
        Robot robot(pose, velocity, size);

        robot.setVelocity(3.0, -0.25);
        const Velocity actual_velocity = robot.getVelocity();

        EXPECT_DOUBLE_EQ(actual_velocity.linearVelocity, 3.0);
        EXPECT_DOUBLE_EQ(actual_velocity.angularVelocity, -0.25);
    }

    TEST(RobotTests, SetVelocityAcceptsAngularVelocityAtLimits){
        Robot robot(Pose{}, Velocity{}, Rectangle{2.0, 4.0});

        EXPECT_NO_THROW(robot.setVelocity(1.0, -2.0 * pi));
        EXPECT_NO_THROW(robot.setVelocity(1.0, 2.0 * pi));
    }

    TEST(RobotTests, SetVelocityRejectsAngularVelocityOutsideLimits){
        Robot robot(Pose{}, Velocity{1.0, 0.5}, Rectangle{2.0, 4.0});

        EXPECT_THROW(robot.setVelocity(3.0, -2.0 * pi - 0.01), std::invalid_argument);
        EXPECT_THROW(robot.setVelocity(3.0, 2.0 * pi + 0.01), std::invalid_argument);

        const Velocity actual_velocity = robot.getVelocity();
        EXPECT_DOUBLE_EQ(actual_velocity.linearVelocity, 1.0);
        EXPECT_DOUBLE_EQ(actual_velocity.angularVelocity, 0.5);
    }

    TEST(RobotTests, SetVelocityRejectsNonFiniteValues){
        Robot robot(Pose{}, Velocity{1.0, 0.5}, Rectangle{2.0, 4.0});
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double infinity = std::numeric_limits<double>::infinity();

        EXPECT_THROW(robot.setVelocity(nan, 0.0), std::invalid_argument);
        EXPECT_THROW(robot.setVelocity(0.0, infinity), std::invalid_argument);
        EXPECT_DOUBLE_EQ(robot.getVelocity().linearVelocity, 1.0);
        EXPECT_DOUBLE_EQ(robot.getVelocity().angularVelocity, 0.5);
    }

    TEST(RobotTests, UpdatedVelocityAffectsSubsequentMovement){
        const Pose pose {0.0, 0.0, 0.0};
        const Rectangle size {2.0, 4.0};
        const Velocity velocity {1.0, 0.0};
        Robot robot(pose, velocity, size);

        robot.setVelocity(4.0, 0.0);
        robot.move(0.5);
        const Pose actual_pose = robot.getPose();

        EXPECT_NEAR(actual_pose.x, 2.0, tolerance);
        EXPECT_NEAR(actual_pose.y, 0.0, tolerance);
        EXPECT_NEAR(actual_pose.theta, 0.0, tolerance);
    }

    TEST(RobotTests, StartsWithoutDistanceSensors){
        const Robot robot(Pose{}, Velocity{}, Rectangle{2.0, 4.0});

        EXPECT_TRUE(robot.getDistanceSensors().empty());
    }

    TEST(RobotTests, AddsDistanceSensorAndPreservesConfiguration){
        Robot robot(Pose{}, Velocity{}, Rectangle{2.0, 4.0});
        const Pose relative_pose {1.0, 0.5, pi / 4.0};
        const DistanceSensor sensor(0.5, 10.0, relative_pose, pi / 3.0);

        robot.addDistanceSensor(sensor);
        const std::vector<DistanceSensor>& sensors = robot.getDistanceSensors();

        ASSERT_EQ(sensors.size(), 1U);
        EXPECT_DOUBLE_EQ(sensors[0].getSensorMinDistance(), 0.5);
        EXPECT_DOUBLE_EQ(sensors[0].getSensorMaxDistance(), 10.0);
        EXPECT_DOUBLE_EQ(sensors[0].getRelativeToRobotPose().x, relative_pose.x);
        EXPECT_DOUBLE_EQ(sensors[0].getRelativeToRobotPose().y, relative_pose.y);
        EXPECT_DOUBLE_EQ(sensors[0].getRelativeToRobotPose().theta, relative_pose.theta);
    }
}
