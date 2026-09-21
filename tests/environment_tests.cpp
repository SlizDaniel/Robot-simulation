#include <gtest/gtest.h>
#include <robot_simulation/environment.hpp>
#include <robot_simulation/math_constans.hpp>
#include <robot_simulation/obstacle.hpp>
#include <robot_simulation/robot.hpp>
#include <robot_simulation/types.hpp>
#include <stdexcept>

namespace robot_simulation{
    TEST(EnvironmentTests, StoresConstructorSize){
        const Rectangle size {20.0, 30.0};

        const Environment environment(size);

        const Rectangle actual_size = environment.getEnvironmentSize();

        EXPECT_DOUBLE_EQ(actual_size.width, size.width);
        EXPECT_DOUBLE_EQ(actual_size.length, size.length);
    }

    TEST(EnvironmentTests, RejectsZeroWidth){
        const Rectangle size {0.0, 20.0};

        EXPECT_THROW(Environment{size}, std::invalid_argument);
    }

    TEST(EnvironmentTests, RejectsZeroLength){
        const Rectangle size {20.0, 0.0};

        EXPECT_THROW(Environment{size}, std::invalid_argument);
    }

    TEST(EnvironmentTests, RejectsNegativeWidth){
        const Rectangle size {-20.0, 30.0};

        EXPECT_THROW(Environment{size}, std::invalid_argument);
    }

    TEST(EnvironmentTests, RejectsNegativeLength){
        const Rectangle size {20.0, -30.0};

        EXPECT_THROW(Environment{size}, std::invalid_argument);
    }

    TEST(EnvironmentTests, StartsWithNoObstacles){
        const Rectangle size {20.0, 30.0};

        const Environment environment(size);

        EXPECT_TRUE(environment.getObstacles().empty());
    }

    TEST(EnvironmentTests, AddsObstacle){
        const Rectangle environment_size {20.0, 30.0};
        const Pose obstacle_pose {2.0, 3.0, 0.2};
        const Rectangle obstacle_size {4.0, 6.0};
        Obstacle obstacle(obstacle_pose, obstacle_size);

        Environment environment(environment_size);
        environment.addObstacle(obstacle);

        const std::vector<Obstacle> obstacles = environment.getObstacles();

        ASSERT_EQ(obstacles.size(), 1U);
        const Pose actual_pose = obstacles[0].get_obstacle_pose();
        const Rectangle actual_size = obstacles[0].get_obstacle_size();
        EXPECT_DOUBLE_EQ(actual_pose.x, obstacle_pose.x);
        EXPECT_DOUBLE_EQ(actual_pose.y, obstacle_pose.y);
        EXPECT_DOUBLE_EQ(actual_pose.theta, obstacle_pose.theta);
        EXPECT_DOUBLE_EQ(actual_size.width, obstacle_size.width);
        EXPECT_DOUBLE_EQ(actual_size.length, obstacle_size.length);
    }

    TEST(EnvironmentTests, PreservesObstacleInsertionOrder){
        const Rectangle environment_size {20.0, 30.0};
        Obstacle first_obstacle(Pose{1.0, 2.0, 0.0}, Rectangle{2.0, 4.0});
        Obstacle second_obstacle(Pose{5.0, 6.0, pi / 2.0}, Rectangle{3.0, 5.0});

        Environment environment(environment_size);
        environment.addObstacle(first_obstacle);
        environment.addObstacle(second_obstacle);

        const std::vector<Obstacle> obstacles = environment.getObstacles();

        ASSERT_EQ(obstacles.size(), 2U);
        EXPECT_DOUBLE_EQ(obstacles[0].get_obstacle_pose().x, 1.0);
        EXPECT_DOUBLE_EQ(obstacles[0].get_obstacle_pose().y, 2.0);
        EXPECT_DOUBLE_EQ(obstacles[1].get_obstacle_pose().x, 5.0);
        EXPECT_DOUBLE_EQ(obstacles[1].get_obstacle_pose().y, 6.0);
    }

    TEST(EnvironmentTests, DoesNotCollideWhenThereAreNoObstacles){
        const Rectangle environment_size {20.0, 30.0};
        const Pose robot_pose {0.0, 0.0, 0.0};
        const Velocity velocity {0.0, 0.0};
        const Rectangle robot_size {2.0, 4.0};
        Robot robot(robot_pose, velocity, robot_size);

        const Environment environment(environment_size);

        EXPECT_FALSE(environment.collides(robot));
    }

    TEST(EnvironmentTests, DoesNotCollideWithSeparatedObstacle){
        const Rectangle environment_size {20.0, 30.0};
        const Pose robot_pose {0.0, 0.0, 0.0};
        const Velocity velocity {0.0, 0.0};
        const Rectangle robot_size {2.0, 4.0};
        const Pose obstacle_pose {10.0, 0.0, 0.0};
        const Rectangle obstacle_size {2.0, 4.0};
        Robot robot(robot_pose, velocity, robot_size);
        Obstacle obstacle(obstacle_pose, obstacle_size);

        Environment environment(environment_size);
        environment.addObstacle(obstacle);

        EXPECT_FALSE(environment.collides(robot));
    }

    TEST(EnvironmentTests, CollidesWithOverlappingAxisAlignedObstacle){
        const Rectangle environment_size {20.0, 30.0};
        const Pose robot_pose {0.0, 0.0, 0.0};
        const Velocity velocity {0.0, 0.0};
        const Rectangle robot_size {2.0, 4.0};
        const Pose obstacle_pose {3.0, 0.0, 0.0};
        const Rectangle obstacle_size {2.0, 4.0};
        Robot robot(robot_pose, velocity, robot_size);
        Obstacle obstacle(obstacle_pose, obstacle_size);

        Environment environment(environment_size);
        environment.addObstacle(obstacle);

        EXPECT_TRUE(environment.collides(robot));
    }

    TEST(EnvironmentTests, TreatsTouchingRectanglesAsCollision){
        const Rectangle environment_size {20.0, 30.0};
        const Pose robot_pose {0.0, 0.0, 0.0};
        const Velocity velocity {0.0, 0.0};
        const Rectangle robot_size {2.0, 4.0};
        const Pose obstacle_pose {4.0, 0.0, 0.0};
        const Rectangle obstacle_size {2.0, 4.0};
        Robot robot(robot_pose, velocity, robot_size);
        Obstacle obstacle(obstacle_pose, obstacle_size);

        Environment environment(environment_size);
        environment.addObstacle(obstacle);

        EXPECT_TRUE(environment.collides(robot));
    }

    TEST(EnvironmentTests, CollidesWithOneOfMultipleObstacles){
        const Rectangle environment_size {20.0, 30.0};
        const Pose robot_pose {0.0, 0.0, 0.0};
        const Velocity velocity {0.0, 0.0};
        const Rectangle robot_size {2.0, 4.0};
        Obstacle separated_obstacle(Pose{10.0, 0.0, 0.0}, Rectangle{2.0, 4.0});
        Obstacle colliding_obstacle(Pose{3.0, 0.0, 0.0}, Rectangle{2.0, 4.0});
        Robot robot(robot_pose, velocity, robot_size);

        Environment environment(environment_size);
        environment.addObstacle(separated_obstacle);
        environment.addObstacle(colliding_obstacle);

        EXPECT_TRUE(environment.collides(robot));
    }

    TEST(EnvironmentTests, CollidesWithRotatedRectangles){
        const Rectangle environment_size {20.0, 30.0};
        const Pose robot_pose {0.0, 0.0, pi / 6.0};
        const Velocity velocity {0.0, 0.0};
        const Rectangle robot_size {2.0, 4.0};
        const Pose obstacle_pose {0.5, 0.2, -pi / 4.0};
        const Rectangle obstacle_size {2.0, 4.0};
        Robot robot(robot_pose, velocity, robot_size);
        Obstacle obstacle(obstacle_pose, obstacle_size);

        Environment environment(environment_size);
        environment.addObstacle(obstacle);

        EXPECT_TRUE(environment.collides(robot));
    }

    TEST(EnvironmentTests, DoesNotCollideForRotatedRectanglesWithOverlappingAABBs){
        const Rectangle environment_size {20.0, 30.0};
        const Pose robot_pose {5.0, 5.0, pi / 6.0};
        const Velocity velocity {0.0, 0.0};
        const Rectangle robot_size {0.5, 4.0};
        const Pose obstacle_pose {4.7, 5.5196152422706632, pi / 6.0};
        const Rectangle obstacle_size {0.5, 4.0};
        Robot robot(robot_pose, velocity, robot_size);
        Obstacle obstacle(obstacle_pose, obstacle_size);

        Environment environment(environment_size);
        environment.addObstacle(obstacle);

        EXPECT_FALSE(environment.collides(robot));
    }
}
