#include <gtest/gtest.h>
#include <robot_simulation/distance_sensor.hpp>
#include <robot_simulation/environment.hpp>
#include <robot_simulation/math_constants.hpp>
#include <robot_simulation/obstacle.hpp>
#include <robot_simulation/types.hpp>
#include <cmath>
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

    TEST(DistanceSensorTests, ReportsNoDetectionInEmptyEnvironment){
        const Environment environment(Rectangle{100.0, 100.0});
        const DistanceSensor sensor(0.0, 10.0, Pose{}, pi / 2.0);

        const RaycastResult detection = sensor.nearestObstacleDetected(environment, 3, Pose{});

        EXPECT_FALSE(detection.object_detected);
        EXPECT_TRUE(std::isinf(detection.distance_to_object));
        EXPECT_EQ(detection.detection_type, DetectionType::NOOBJECTDETECTED);
    }

    TEST(DistanceSensorTests, DetectsObstacleWithSingleCentralRay){
        Environment environment(Rectangle{100.0, 100.0});
        environment.addObstacle(Obstacle(Pose{5.0, 0.0, 0.0}, Rectangle{2.0, 2.0}));
        const DistanceSensor sensor(0.0, 10.0, Pose{}, pi / 2.0);

        const RaycastResult detection = sensor.nearestObstacleDetected(environment, 1, Pose{});

        EXPECT_TRUE(detection.object_detected);
        EXPECT_NEAR(detection.distance_to_object, 4.0, tolerance);
        EXPECT_EQ(detection.detection_type, DetectionType::OBJECTDETECTED);
    }

    TEST(DistanceSensorTests, SingleRayRejectsObstacleCloserThanMinDistance){
        Environment environment(Rectangle{100.0, 100.0});
        environment.addObstacle(Obstacle(Pose{5.0, 0.0, 0.0}, Rectangle{2.0, 2.0}));
        const DistanceSensor sensor(5.0, 10.0, Pose{}, pi / 2.0);

        const RaycastResult detection = sensor.nearestObstacleDetected(environment, 1, Pose{});

        EXPECT_FALSE(detection.object_detected);
        EXPECT_NEAR(detection.distance_to_object, 4.0, tolerance);
        EXPECT_EQ(detection.detection_type, DetectionType::OBJECTOUTOFRANGE);
    }

    TEST(DistanceSensorTests, SingleRayRejectsObstacleFartherThanMaxDistance){
        Environment environment(Rectangle{100.0, 100.0});
        environment.addObstacle(Obstacle(Pose{5.0, 0.0, 0.0}, Rectangle{2.0, 2.0}));
        const DistanceSensor sensor(0.0, 3.0, Pose{}, pi / 2.0);

        const RaycastResult detection = sensor.nearestObstacleDetected(environment, 1, Pose{});

        EXPECT_FALSE(detection.object_detected);
        EXPECT_NEAR(detection.distance_to_object, 4.0, tolerance);
        EXPECT_EQ(detection.detection_type, DetectionType::OBJECTOUTOFRANGE);
    }

    TEST(DistanceSensorTests, ReturnsNearestObstacleRegardlessOfInsertionOrder){
        Environment first_environment(Rectangle{100.0, 100.0});
        first_environment.addObstacle(Obstacle(Pose{8.0, 0.0, 0.0}, Rectangle{2.0, 2.0}));
        first_environment.addObstacle(Obstacle(Pose{5.0, 0.0, 0.0}, Rectangle{2.0, 2.0}));
        Environment second_environment(Rectangle{100.0, 100.0});
        second_environment.addObstacle(Obstacle(Pose{5.0, 0.0, 0.0}, Rectangle{2.0, 2.0}));
        second_environment.addObstacle(Obstacle(Pose{8.0, 0.0, 0.0}, Rectangle{2.0, 2.0}));
        const DistanceSensor sensor(0.0, 10.0, Pose{}, pi / 2.0);

        const RaycastResult first_detection = sensor.nearestObstacleDetected(first_environment, 1, Pose{});
        const RaycastResult second_detection = sensor.nearestObstacleDetected(second_environment, 1, Pose{});

        ASSERT_TRUE(first_detection.object_detected);
        ASSERT_TRUE(second_detection.object_detected);
        EXPECT_NEAR(first_detection.distance_to_object, 4.0, tolerance);
        EXPECT_NEAR(second_detection.distance_to_object, 4.0, tolerance);
    }

    TEST(DistanceSensorTests, TooCloseObstacleBlocksFartherObstacleOnSameRay){
        Environment environment(Rectangle{100.0, 100.0});
        environment.addObstacle(Obstacle(Pose{2.0, 0.0, 0.0}, Rectangle{2.0, 2.0}));
        environment.addObstacle(Obstacle(Pose{6.0, 0.0, 0.0}, Rectangle{2.0, 2.0}));
        const DistanceSensor sensor(2.0, 10.0, Pose{}, pi / 2.0);

        const RaycastResult detection = sensor.nearestObstacleDetected(environment, 1, Pose{});

        EXPECT_FALSE(detection.object_detected);
        EXPECT_NEAR(detection.distance_to_object, 1.0, tolerance);
        EXPECT_EQ(detection.detection_type, DetectionType::OBJECTOUTOFRANGE);
    }

    TEST(DistanceSensorTests, ObstacleBehindSensorDoesNotBlockFrontObstacle){
        Environment environment(Rectangle{100.0, 100.0});
        environment.addObstacle(Obstacle(Pose{-3.0, 0.0, 0.0}, Rectangle{2.0, 2.0}));
        environment.addObstacle(Obstacle(Pose{5.0, 0.0, 0.0}, Rectangle{2.0, 2.0}));
        const DistanceSensor sensor(0.0, 10.0, Pose{}, pi / 2.0);

        const RaycastResult detection = sensor.nearestObstacleDetected(environment, 1, Pose{});

        EXPECT_TRUE(detection.object_detected);
        EXPECT_NEAR(detection.distance_to_object, 4.0, tolerance);
    }

    TEST(DistanceSensorTests, RejectsZeroRayCount){
        Environment environment(Rectangle{100.0, 100.0});
        const DistanceSensor sensor(0.0, 10.0, Pose{}, pi / 2.0);

        EXPECT_THROW(sensor.nearestObstacleDetected(environment, 0, Pose{}), std::invalid_argument);
    }

    TEST(DistanceSensorTests, RejectsNegativeRayCount){
        Environment environment(Rectangle{100.0, 100.0});
        const DistanceSensor sensor(0.0, 10.0, Pose{}, pi / 2.0);

        EXPECT_THROW(sensor.nearestObstacleDetected(environment, -1, Pose{}), std::invalid_argument);
    }

    TEST(DistanceSensorTests, RejectsEvenRayCount){
        Environment environment(Rectangle{100.0, 100.0});
        environment.addObstacle(Obstacle(Pose{8.0, 0.0, 0.0}, Rectangle{0.1, 0.1}));
        const DistanceSensor sensor(0.0, 10.0, Pose{}, pi / 2.0);

        EXPECT_THROW(sensor.nearestObstacleDetected(environment, 2, Pose{}), std::invalid_argument);
    }

    TEST(DistanceSensorTests, ThreeRaysIncludeCentralDirection){
        Environment environment(Rectangle{100.0, 100.0});
        environment.addObstacle(Obstacle(Pose{8.0, 0.0, 0.0}, Rectangle{0.1, 0.1}));
        const DistanceSensor sensor(0.0, 10.0, Pose{}, pi / 2.0);

        const RaycastResult detection = sensor.nearestObstacleDetected(environment, 3, Pose{});

        EXPECT_TRUE(detection.object_detected);
        EXPECT_NEAR(detection.distance_to_object, 7.95, tolerance);
    }

    TEST(DistanceSensorTests, DetectsObstacleAtLeftFieldOfViewBoundary){
        Environment environment(Rectangle{100.0, 100.0});
        const double coordinate = 5.0 / std::sqrt(2.0);
        environment.addObstacle(Obstacle(Pose{coordinate, -coordinate, 0.0}, Rectangle{0.2, 0.2}));
        const DistanceSensor sensor(0.0, 10.0, Pose{}, pi / 2.0);

        const RaycastResult detection = sensor.nearestObstacleDetected(environment, 3, Pose{});

        EXPECT_TRUE(detection.object_detected);
    }

    TEST(DistanceSensorTests, DetectsObstacleAtRightFieldOfViewBoundary){
        Environment environment(Rectangle{100.0, 100.0});
        const double coordinate = 5.0 / std::sqrt(2.0);
        environment.addObstacle(Obstacle(Pose{coordinate, coordinate, 0.0}, Rectangle{0.2, 0.2}));
        const DistanceSensor sensor(0.0, 10.0, Pose{}, pi / 2.0);

        const RaycastResult detection = sensor.nearestObstacleDetected(environment, 3, Pose{});

        EXPECT_TRUE(detection.object_detected);
    }

    TEST(DistanceSensorTests, DoesNotDetectSmallObstacleOutsideFieldOfView){
        Environment environment(Rectangle{100.0, 100.0});
        const double angle = pi / 3.0;
        environment.addObstacle(Obstacle(
            Pose{5.0 * std::cos(angle), 5.0 * std::sin(angle), 0.0}, Rectangle{0.1, 0.1}));
        const DistanceSensor sensor(0.0, 10.0, Pose{}, pi / 2.0);

        const RaycastResult detection = sensor.nearestObstacleDetected(environment, 31, Pose{});

        EXPECT_FALSE(detection.object_detected);
        EXPECT_EQ(detection.detection_type, DetectionType::NOOBJECTDETECTED);
    }

    TEST(DistanceSensorTests, ReturnsNearestDetectionFromDifferentRays){
        Environment environment(Rectangle{100.0, 100.0});
        const double coordinate = 4.0 / std::sqrt(2.0);
        environment.addObstacle(Obstacle(Pose{6.0, 0.0, 0.0}, Rectangle{0.2, 0.2}));
        environment.addObstacle(Obstacle(Pose{coordinate, coordinate, 0.0}, Rectangle{0.2, 0.2}));
        const DistanceSensor sensor(0.0, 10.0, Pose{}, pi / 2.0);

        const RaycastResult detection = sensor.nearestObstacleDetected(environment, 3, Pose{});

        EXPECT_TRUE(detection.object_detected);
        EXPECT_NEAR(detection.distance_to_object, 4.0 - 0.1 * std::sqrt(2.0), tolerance);
        EXPECT_EQ(detection.detection_type, DetectionType::OBJECTDETECTED);
    }

    TEST(DistanceSensorTests, TooCloseEchoAnywhereInFieldOfViewSuppressesFartherDetection){
        Environment environment(Rectangle{100.0, 100.0});
        const double close_coordinate = 1.5 / std::sqrt(2.0);
        environment.addObstacle(Obstacle(
            Pose{close_coordinate, -close_coordinate, 0.0}, Rectangle{0.2, 0.2}));
        environment.addObstacle(Obstacle(Pose{5.0, 0.0, 0.0}, Rectangle{0.2, 0.2}));
        const DistanceSensor sensor(2.0, 10.0, Pose{}, pi / 2.0);

        const RaycastResult detection = sensor.nearestObstacleDetected(environment, 3, Pose{});

        EXPECT_FALSE(detection.object_detected);
        EXPECT_NEAR(detection.distance_to_object, 1.5 - 0.1 * std::sqrt(2.0), tolerance);
        EXPECT_EQ(detection.detection_type, DetectionType::OBJECTOUTOFRANGE);
    }

    TEST(DistanceSensorTests, UsesRobotPoseAndRelativeSensorPoseDuringScan){
        Environment environment(Rectangle{100.0, 100.0});
        environment.addObstacle(Obstacle(Pose{10.0, 11.0, 0.0}, Rectangle{2.0, 2.0}));
        const DistanceSensor sensor(0.0, 10.0, Pose{1.0, 0.0, 0.0}, pi / 2.0);
        const Pose robot_pose {10.0, 5.0, pi / 2.0};

        const RaycastResult detection = sensor.nearestObstacleDetected(environment, 1, robot_pose);

        EXPECT_TRUE(detection.object_detected);
        EXPECT_NEAR(detection.distance_to_object, 4.0, tolerance);
    }

    TEST(DistanceSensorTests, DetectsEnvironmentBorderWithSingleCentralRay){
        const Environment environment(Rectangle{20.0, 20.0});
        const DistanceSensor sensor(0.0, 20.0, Pose{}, pi / 2.0);

        const RaycastResult detection = sensor.nearestEnvironmentBorderDetected(environment, 1, Pose{});

        EXPECT_TRUE(detection.object_detected);
        EXPECT_NEAR(detection.distance_to_object, 10.0, tolerance);
        EXPECT_EQ(detection.detection_type, DetectionType::OBJECTDETECTED);
    }

    TEST(DistanceSensorTests, ReturnsNearestEnvironmentBorderFromMultipleRays){
        const Environment environment(Rectangle{20.0, 20.0});
        const DistanceSensor sensor(0.0, 20.0, Pose{}, pi / 2.0);

        const RaycastResult detection = sensor.nearestEnvironmentBorderDetected(
            environment, 3, Pose{7.0, 0.0, 0.0});

        EXPECT_TRUE(detection.object_detected);
        EXPECT_NEAR(detection.distance_to_object, 3.0, tolerance);
        EXPECT_EQ(detection.detection_type, DetectionType::OBJECTDETECTED);
    }

    TEST(DistanceSensorTests, SingleRayRejectsEnvironmentBorderCloserThanMinDistance){
        const Environment environment(Rectangle{20.0, 20.0});
        const DistanceSensor sensor(11.0, 20.0, Pose{}, pi / 2.0);

        const RaycastResult detection = sensor.nearestEnvironmentBorderDetected(environment, 1, Pose{});

        EXPECT_FALSE(detection.object_detected);
        EXPECT_NEAR(detection.distance_to_object, 10.0, tolerance);
        EXPECT_EQ(detection.detection_type, DetectionType::OBJECTOUTOFRANGE);
    }

    TEST(DistanceSensorTests, SingleRayRejectsEnvironmentBorderFartherThanMaxDistance){
        const Environment environment(Rectangle{20.0, 20.0});
        const DistanceSensor sensor(0.0, 9.0, Pose{}, pi / 2.0);

        const RaycastResult detection = sensor.nearestEnvironmentBorderDetected(environment, 1, Pose{});

        EXPECT_FALSE(detection.object_detected);
        EXPECT_NEAR(detection.distance_to_object, 10.0, tolerance);
        EXPECT_EQ(detection.detection_type, DetectionType::OBJECTOUTOFRANGE);
    }

    TEST(DistanceSensorTests, TooCloseEnvironmentBorderInFieldOfViewSuppressesFartherDetection){
        const Environment environment(Rectangle{20.0, 20.0});
        const DistanceSensor sensor(2.0, 20.0, Pose{}, pi);

        const RaycastResult detection = sensor.nearestEnvironmentBorderDetected(
            environment, 3, Pose{9.0, 0.0, pi / 2.0});

        EXPECT_FALSE(detection.object_detected);
        EXPECT_NEAR(detection.distance_to_object, 1.0, tolerance);
        EXPECT_EQ(detection.detection_type, DetectionType::OBJECTOUTOFRANGE);
    }

    TEST(DistanceSensorTests, UsesRobotPoseAndRelativeSensorPoseDuringEnvironmentBorderScan){
        const Environment environment(Rectangle{20.0, 20.0});
        const DistanceSensor sensor(0.0, 20.0, Pose{1.0, 0.0, 0.0}, pi / 2.0);
        const Pose robot_pose {5.0, 0.0, pi / 2.0};

        const RaycastResult detection = sensor.nearestEnvironmentBorderDetected(environment, 1, robot_pose);

        EXPECT_TRUE(detection.object_detected);
        EXPECT_NEAR(detection.distance_to_object, 9.0, tolerance);
        EXPECT_EQ(detection.detection_type, DetectionType::OBJECTDETECTED);
    }

    TEST(DistanceSensorTests, RejectsZeroEnvironmentBorderRayCount){
        const Environment environment(Rectangle{20.0, 20.0});
        const DistanceSensor sensor(0.0, 20.0, Pose{}, pi / 2.0);

        EXPECT_THROW(sensor.nearestEnvironmentBorderDetected(environment, 0, Pose{}), std::invalid_argument);
    }

    TEST(DistanceSensorTests, RejectsNegativeEnvironmentBorderRayCount){
        const Environment environment(Rectangle{20.0, 20.0});
        const DistanceSensor sensor(0.0, 20.0, Pose{}, pi / 2.0);

        EXPECT_THROW(sensor.nearestEnvironmentBorderDetected(environment, -1, Pose{}), std::invalid_argument);
    }

    TEST(DistanceSensorTests, RejectsEvenEnvironmentBorderRayCount){
        const Environment environment(Rectangle{20.0, 20.0});
        const DistanceSensor sensor(0.0, 20.0, Pose{}, pi / 2.0);

        EXPECT_THROW(sensor.nearestEnvironmentBorderDetected(environment, 2, Pose{}), std::invalid_argument);
    }

    TEST(DistanceSensorTests, NearestObjectDetectsCloserObstacleThanEnvironmentBorder){
        Environment environment(Rectangle{20.0, 20.0});
        environment.addObstacle(Obstacle(Pose{5.0, 0.0, 0.0}, Rectangle{2.0, 2.0}));
        const DistanceSensor sensor(0.0, 20.0, Pose{}, pi / 2.0);

        const RaycastResult detection = sensor.nearestObjectDetected(environment, 1, Pose{});

        EXPECT_TRUE(detection.object_detected);
        EXPECT_NEAR(detection.distance_to_object, 4.0, tolerance);
        EXPECT_EQ(detection.detection_type, DetectionType::OBJECTDETECTED);
    }

    TEST(DistanceSensorTests, NearestObjectDetectsEnvironmentBorderWithoutObstacles){
        const Environment environment(Rectangle{20.0, 20.0});
        const DistanceSensor sensor(0.0, 20.0, Pose{}, pi / 2.0);

        const RaycastResult detection = sensor.nearestObjectDetected(environment, 1, Pose{});

        EXPECT_TRUE(detection.object_detected);
        EXPECT_NEAR(detection.distance_to_object, 10.0, tolerance);
        EXPECT_EQ(detection.detection_type, DetectionType::OBJECTDETECTED);
    }

    TEST(DistanceSensorTests, NearestObjectReportsCloserOutOfRangeObstacleBeforeEnvironmentBorder){
        Environment environment(Rectangle{20.0, 20.0});
        environment.addObstacle(Obstacle(Pose{1.0, 0.0, 0.0}, Rectangle{0.5, 0.5}));
        const DistanceSensor sensor(2.0, 20.0, Pose{}, pi / 2.0);

        const RaycastResult detection = sensor.nearestObjectDetected(environment, 1, Pose{});

        EXPECT_FALSE(detection.object_detected);
        EXPECT_NEAR(detection.distance_to_object, 0.75, tolerance);
        EXPECT_EQ(detection.detection_type, DetectionType::OBJECTOUTOFRANGE);
    }

    TEST(DistanceSensorTests, NearestObjectRejectsZeroRayCount){
        const Environment environment(Rectangle{20.0, 20.0});
        const DistanceSensor sensor(0.0, 20.0, Pose{}, pi / 2.0);

        EXPECT_THROW(sensor.nearestObjectDetected(environment, 0, Pose{}), std::invalid_argument);
    }

    TEST(DistanceSensorTests, NearestObjectRejectsNegativeRayCount){
        const Environment environment(Rectangle{20.0, 20.0});
        const DistanceSensor sensor(0.0, 20.0, Pose{}, pi / 2.0);

        EXPECT_THROW(sensor.nearestObjectDetected(environment, -1, Pose{}), std::invalid_argument);
    }

    TEST(DistanceSensorTests, NearestObjectRejectsEvenRayCount){
        const Environment environment(Rectangle{20.0, 20.0});
        const DistanceSensor sensor(0.0, 20.0, Pose{}, pi / 2.0);

        EXPECT_THROW(sensor.nearestObjectDetected(environment, 2, Pose{}), std::invalid_argument);
    }

}//namespace robot_simulation
