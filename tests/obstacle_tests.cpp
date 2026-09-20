#include <gtest/gtest.h>
#include <robot_simulation/math_constans.hpp>
#include <robot_simulation/obstacle.hpp>
#include <robot_simulation/types.hpp>

#include "helpers.hpp"

namespace robot_simulation{
    TEST(ObstacleTests, StoresConstructorArguments){
        const Pose pose {5.0, 7.0, 0.2};
        const Rectangle size {2.0, 4.0};

        const Obstacle obstacle(pose, size);

        const Pose actual_pose = obstacle.get_obstacle_pose();
        const Rectangle actual_size = obstacle.get_obstacle_size();

        EXPECT_DOUBLE_EQ(actual_pose.x, pose.x);
        EXPECT_DOUBLE_EQ(actual_pose.y, pose.y);
        EXPECT_DOUBLE_EQ(actual_pose.theta, pose.theta);
        EXPECT_DOUBLE_EQ(actual_size.width, size.width);
        EXPECT_DOUBLE_EQ(actual_size.length, size.length);
    }

    TEST(ObstacleTests, CornersCalculation){
        const Pose pose {5.0, 7.0, 0.0};
        const Rectangle size {2.0, 4.0};

        const Obstacle obstacle(pose, size);

        const RectangleCorners obstacle_corners = obstacle.getObstacleCorners();
        const RectangleCorners expected{
            Point{7.0, 8.0},
            Point{7.0, 6.0},
            Point{3.0, 6.0},
            Point{3.0, 8.0}
        };
        expectCornersNear(obstacle_corners, expected, tolerance);
    }

    TEST(ObstacleTests, RotatedCornersCalculation){
        const Pose pose {0.0, 0.0, pi / 2.0};
        const Rectangle size {2.0, 4.0};

        const Obstacle obstacle(pose, size);

        const RectangleCorners obstacle_corners = obstacle.getObstacleCorners();
        const RectangleCorners expected{
            Point{-1.0, 2.0},
            Point{ 1.0, 2.0},
            Point{ 1.0, -2.0},
            Point{-1.0, -2.0}
        };
        expectCornersNear(obstacle_corners, expected, tolerance);
    }
}
