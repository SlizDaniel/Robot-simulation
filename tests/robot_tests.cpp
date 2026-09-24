#include <gtest/gtest.h>
#include <robot_simulation/robot.hpp>
#include <robot_simulation/types.hpp>
#include "helpers.hpp"
#include <robot_simulation/math_constans.hpp>
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
}
