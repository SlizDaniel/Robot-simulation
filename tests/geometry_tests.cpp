#include <gtest/gtest.h>
#include <robot_simulation/geometry.hpp>
#include <robot_simulation/distance_sensor.hpp>
#include <robot_simulation/obstacle.hpp>
#include <robot_simulation/types.hpp>
#include "helpers.hpp"
#include <robot_simulation/math_constants.hpp>
#include <array>
#include <cmath>
#include <stdexcept>

namespace robot_simulation{
    TEST (GeometryTest, CalculateCornersWithoutRotation){
        const Pose pose  {0.0, 0.0, 0.0};
        const Rectangle size {2.0, 4.0};

        const RectangleCorners corners = calculateRectangleCorners(pose, size);
        const RectangleCorners expected{
            Point{ 2.0,  1.0},
            Point{ 2.0, -1.0},
            Point{-2.0, -1.0},
            Point{-2.0,  1.0}
            };
        expectCornersNear(corners, expected, tolerance);
    }

    TEST (GeometryTest, CalculateCornersForTranslatedRectangle){
        const Pose pose {10.0, 20.0, 0.0};
        const Rectangle size {2.0, 4.0};

        const RectangleCorners corners = calculateRectangleCorners(pose, size);
        const RectangleCorners expected{
            Point{12.0, 21.0},
            Point{12.0, 19.0},
            Point{ 8.0, 19.0},
            Point{ 8.0, 21.0}
        };
        expectCornersNear(corners, expected, tolerance);
    }

    TEST (GeometryTest, CalculateCornersAfterNinetyDegreeRotation){
        const Pose pose {0.0, 0.0, pi / 2.0};
        const Rectangle size {2.0, 4.0};

        const RectangleCorners corners = calculateRectangleCorners(pose, size);
        const RectangleCorners expected{
            Point{-1.0,  2.0},
            Point{ 1.0,  2.0},
            Point{ 1.0, -2.0},
            Point{-1.0, -2.0}
        };
        expectCornersNear(corners, expected, tolerance);
    }

    TEST (GeometryTest, CalculateCornersAfterNegativeNinetyDegreeRotation){
        const Pose pose {0.0, 0.0, -pi / 2.0};
        const Rectangle size {2.0, 4.0};

        const RectangleCorners corners = calculateRectangleCorners(pose, size);
        const RectangleCorners expected{
            Point{ 1.0, -2.0},
            Point{-1.0, -2.0},
            Point{-1.0,  2.0},
            Point{ 1.0,  2.0}
        };
        expectCornersNear(corners, expected, tolerance);
    }

    TEST (GeometryTest, CalculateCornersForArbitraryRotation){
        const Pose pose {3.0, -2.0, 0.37};
        const Rectangle size {2.0, 4.0};

        const RectangleCorners corners = calculateRectangleCorners(pose, size);
        const RectangleCorners expected{
            Point{4.503039259247, -0.344441790464},
            Point{5.226270123177, -2.209096481676},
            Point{1.496960740753, -3.655558209536},
            Point{0.773729876823, -1.790903518324}
        };
        expectCornersNear(corners, expected, tolerance);
    }

    TEST (GeometryTest, CalculateCornersForSquare){
        const Pose pose {0.0, 0.0, 0.0};
        const Rectangle size {2.0, 2.0};

        const RectangleCorners corners = calculateRectangleCorners(pose, size);
        const RectangleCorners expected{
            Point{ 1.0,  1.0},
            Point{ 1.0, -1.0},
            Point{-1.0, -1.0},
            Point{-1.0,  1.0}
        };
        expectCornersNear(corners, expected, tolerance);
    }

    TEST(GeometryTest, RaycastDetectsObstacleToTheRight){
        const Obstacle obstacle(Pose{5.0, 0.0, 0.0}, Rectangle{2.0, 2.0});
        const std::array<double, 2> range {0.0, 10.0};

        const RaycastResult result = calculateDistanceSensorObstacle(Pose{0.0, 0.0, 0.0}, range, obstacle);

        EXPECT_TRUE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, 4.0, tolerance);
        EXPECT_EQ(result.detection_type, DetectionType::OBJECTDETECTED);
    }

    TEST(GeometryTest, RaycastDetectsObstacleAbove){
        const Obstacle obstacle(Pose{0.0, 5.0, 0.0}, Rectangle{2.0, 2.0});
        const std::array<double, 2> range {0.0, 10.0};

        const RaycastResult result = calculateDistanceSensorObstacle(Pose{0.0, 0.0, pi / 2.0}, range, obstacle);

        EXPECT_TRUE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, 4.0, tolerance);
        EXPECT_EQ(result.detection_type, DetectionType::OBJECTDETECTED);
    }

    TEST(GeometryTest, RaycastDetectsObstacleToTheLeft){
        const Obstacle obstacle(Pose{-5.0, 0.0, 0.0}, Rectangle{2.0, 2.0});
        const std::array<double, 2> range {0.0, 10.0};

        const RaycastResult result = calculateDistanceSensorObstacle(Pose{0.0, 0.0, pi}, range, obstacle);

        EXPECT_TRUE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, 4.0, tolerance);
        EXPECT_EQ(result.detection_type, DetectionType::OBJECTDETECTED);
    }

    TEST(GeometryTest, RaycastDetectsObstacleBelow){
        const Obstacle obstacle(Pose{0.0, -5.0, 0.0}, Rectangle{2.0, 2.0});
        const std::array<double, 2> range {0.0, 10.0};

        const RaycastResult result = calculateDistanceSensorObstacle(Pose{0.0, 0.0, -pi / 2.0}, range, obstacle);

        EXPECT_TRUE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, 4.0, tolerance);
        EXPECT_EQ(result.detection_type, DetectionType::OBJECTDETECTED);
    }

    TEST(GeometryTest, RaycastUsesObstacleLengthAlongLocalXAxis){
        const Obstacle obstacle(Pose{5.0, 0.0, 0.0}, Rectangle{2.0, 4.0});
        const std::array<double, 2> range {0.0, 10.0};

        const RaycastResult result = calculateDistanceSensorObstacle(Pose{0.0, 0.0, 0.0}, range, obstacle);

        EXPECT_TRUE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, 3.0, tolerance);
    }

    TEST(GeometryTest, RaycastDetectsDiagonalObstacle){
        const Obstacle obstacle(Pose{5.0, 5.0, 0.0}, Rectangle{2.0, 2.0});
        const std::array<double, 2> range {0.0, 10.0};

        const RaycastResult result = calculateDistanceSensorObstacle(Pose{0.0, 0.0, pi / 4.0}, range, obstacle);

        EXPECT_TRUE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, 4.0 * std::sqrt(2.0), tolerance);
    }

    TEST(GeometryTest, RaycastDoesNotDetectObstacleNextToRay){
        const Obstacle obstacle(Pose{5.0, 3.0, 0.0}, Rectangle{2.0, 2.0});
        const std::array<double, 2> range {0.0, 10.0};

        const RaycastResult result = calculateDistanceSensorObstacle(Pose{0.0, 0.0, 0.0}, range, obstacle);

        EXPECT_FALSE(result.object_detected);
        EXPECT_TRUE(std::isinf(result.distance_to_object));
        EXPECT_EQ(result.detection_type, DetectionType::NOOBJECTDETECTED);
    }

    TEST(GeometryTest, RaycastDoesNotDetectObstacleBehindHorizontalRay){
        const Obstacle obstacle(Pose{-5.0, 0.0, 0.0}, Rectangle{2.0, 2.0});
        const std::array<double, 2> range {0.0, 10.0};

        const RaycastResult result = calculateDistanceSensorObstacle(Pose{0.0, 0.0, 0.0}, range, obstacle);

        EXPECT_FALSE(result.object_detected);
        EXPECT_TRUE(std::isinf(result.distance_to_object));
    }

    TEST(GeometryTest, RaycastDoesNotDetectObstacleBehindVerticalRay){
        const Obstacle obstacle(Pose{0.0, -5.0, 0.0}, Rectangle{2.0, 2.0});
        const std::array<double, 2> range {0.0, 10.0};

        const RaycastResult result = calculateDistanceSensorObstacle(Pose{0.0, 0.0, pi / 2.0}, range, obstacle);

        EXPECT_FALSE(result.object_detected);
        EXPECT_TRUE(std::isinf(result.distance_to_object));
    }

    TEST(GeometryTest, RaycastDetectsRotatedObstacle){
        const Obstacle obstacle(Pose{5.0, 0.0, pi / 4.0}, Rectangle{2.0, 2.0});
        const std::array<double, 2> range {0.0, 10.0};

        const RaycastResult result = calculateDistanceSensorObstacle(Pose{0.0, 0.0, 0.0}, range, obstacle);

        EXPECT_TRUE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, 5.0 - std::sqrt(2.0), tolerance);
    }

    TEST(GeometryTest, RaycastTreatsTangentialContactAsDetection){
        const Obstacle obstacle(Pose{5.0, 1.0, 0.0}, Rectangle{2.0, 2.0});
        const std::array<double, 2> range {0.0, 10.0};

        const RaycastResult result = calculateDistanceSensorObstacle(Pose{0.0, 0.0, 0.0}, range, obstacle);

        EXPECT_TRUE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, 4.0, tolerance);
    }

    TEST(GeometryTest, RaycastDetectsObstacleExactlyAtMinDistance){
        const Obstacle obstacle(Pose{5.0, 0.0, 0.0}, Rectangle{2.0, 2.0});
        const std::array<double, 2> range {4.0, 10.0};

        const RaycastResult result = calculateDistanceSensorObstacle(Pose{0.0, 0.0, 0.0}, range, obstacle);

        EXPECT_TRUE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, 4.0, tolerance);
    }

    TEST(GeometryTest, RaycastDetectsObstacleExactlyAtMaxDistance){
        const Obstacle obstacle(Pose{5.0, 0.0, 0.0}, Rectangle{2.0, 2.0});
        const std::array<double, 2> range {0.0, 4.0};

        const RaycastResult result = calculateDistanceSensorObstacle(Pose{0.0, 0.0, 0.0}, range, obstacle);

        EXPECT_TRUE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, 4.0, tolerance);
    }

    TEST(GeometryTest, RaycastReportsObstacleCloserThanMinDistance){
        const Obstacle obstacle(Pose{5.0, 0.0, 0.0}, Rectangle{2.0, 2.0});
        const std::array<double, 2> range {5.0, 10.0};

        const RaycastResult result = calculateDistanceSensorObstacle(Pose{0.0, 0.0, 0.0}, range, obstacle);

        EXPECT_FALSE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, 4.0, tolerance);
        EXPECT_EQ(result.detection_type, DetectionType::OBJECTOUTOFRANGE);
    }

    TEST(GeometryTest, RaycastReportsObstacleFartherThanMaxDistance){
        const Obstacle obstacle(Pose{5.0, 0.0, 0.0}, Rectangle{2.0, 2.0});
        const std::array<double, 2> range {0.0, 3.0};

        const RaycastResult result = calculateDistanceSensorObstacle(Pose{0.0, 0.0, 0.0}, range, obstacle);

        EXPECT_FALSE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, 4.0, tolerance);
        EXPECT_EQ(result.detection_type, DetectionType::OBJECTOUTOFRANGE);
    }

    TEST(GeometryTest, RaycastReportsSensorInsideObstacleAsOutOfRange){
        const Obstacle obstacle(Pose{0.0, 0.0, 0.0}, Rectangle{2.0, 2.0});
        const std::array<double, 2> range {0.0, 10.0};

        const RaycastResult result = calculateDistanceSensorObstacle(Pose{0.0, 0.0, 0.0}, range, obstacle);

        EXPECT_FALSE(result.object_detected);
        EXPECT_TRUE(std::isinf(result.distance_to_object));
        EXPECT_EQ(result.detection_type, DetectionType::OBJECTOUTOFRANGE);
    }

    TEST(GeometryTest, RaycastDetectsEnvironmentBorderToTheRight){
        const Rectangle environment_size {10.0, 20.0};
        const std::array<double, 2> range {0.0, 20.0};

        const RaycastResult result = calculateDistanceSensorEnvironment(
            Pose{0.0, 0.0, 0.0}, range, environment_size);

        EXPECT_TRUE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, 10.0, tolerance);
        EXPECT_EQ(result.detection_type, DetectionType::OBJECTDETECTED);
    }

    TEST(GeometryTest, RaycastDetectsEnvironmentBorderToTheLeft){
        const Rectangle environment_size {10.0, 20.0};
        const std::array<double, 2> range {0.0, 20.0};

        const RaycastResult result = calculateDistanceSensorEnvironment(
            Pose{0.0, 0.0, pi}, range, environment_size);

        EXPECT_TRUE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, 10.0, tolerance);
        EXPECT_EQ(result.detection_type, DetectionType::OBJECTDETECTED);
    }

    TEST(GeometryTest, RaycastDetectsEnvironmentBorderAbove){
        const Rectangle environment_size {10.0, 20.0};
        const std::array<double, 2> range {0.0, 20.0};

        const RaycastResult result = calculateDistanceSensorEnvironment(
            Pose{0.0, 0.0, pi / 2.0}, range, environment_size);

        EXPECT_TRUE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, 5.0, tolerance);
        EXPECT_EQ(result.detection_type, DetectionType::OBJECTDETECTED);
    }

    TEST(GeometryTest, RaycastDetectsEnvironmentBorderBelow){
        const Rectangle environment_size {10.0, 20.0};
        const std::array<double, 2> range {0.0, 20.0};

        const RaycastResult result = calculateDistanceSensorEnvironment(
            Pose{0.0, 0.0, -pi / 2.0}, range, environment_size);

        EXPECT_TRUE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, 5.0, tolerance);
        EXPECT_EQ(result.detection_type, DetectionType::OBJECTDETECTED);
    }

    TEST(GeometryTest, RaycastDetectsEnvironmentBorderAlongDiagonal){
        const Rectangle environment_size {10.0, 20.0};
        const std::array<double, 2> range {0.0, 20.0};

        const RaycastResult result = calculateDistanceSensorEnvironment(
            Pose{0.0, 0.0, pi / 4.0}, range, environment_size);

        EXPECT_TRUE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, 5.0 * std::sqrt(2.0), tolerance);
        EXPECT_EQ(result.detection_type, DetectionType::OBJECTDETECTED);
    }

    TEST(GeometryTest, RaycastDetectsNearestEnvironmentBorderFromOffCenterDiagonal){
        const Rectangle environment_size {10.0, 20.0};
        const std::array<double, 2> range {0.0, 20.0};

        const RaycastResult result = calculateDistanceSensorEnvironment(
            Pose{4.0, 4.0, 3.0 * pi / 4.0}, range, environment_size);

        EXPECT_TRUE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, std::sqrt(2.0), tolerance);
        EXPECT_EQ(result.detection_type, DetectionType::OBJECTDETECTED);
    }

    TEST(GeometryTest, RaycastMeasuresEnvironmentBorderFromSensorPosition){
        const Rectangle environment_size {10.0, 20.0};
        const std::array<double, 2> range {0.0, 20.0};

        const RaycastResult result = calculateDistanceSensorEnvironment(
            Pose{8.0, 4.0, 0.0}, range, environment_size);

        EXPECT_TRUE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, 2.0, tolerance);
        EXPECT_EQ(result.detection_type, DetectionType::OBJECTDETECTED);
    }

    TEST(GeometryTest, RaycastDetectsEnvironmentBorderExactlyAtMinDistance){
        const Rectangle environment_size {10.0, 20.0};
        const std::array<double, 2> range {10.0, 20.0};

        const RaycastResult result = calculateDistanceSensorEnvironment(
            Pose{0.0, 0.0, 0.0}, range, environment_size);

        EXPECT_TRUE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, 10.0, tolerance);
        EXPECT_EQ(result.detection_type, DetectionType::OBJECTDETECTED);
    }

    TEST(GeometryTest, RaycastDetectsEnvironmentBorderExactlyAtMaxDistance){
        const Rectangle environment_size {10.0, 20.0};
        const std::array<double, 2> range {0.0, 10.0};

        const RaycastResult result = calculateDistanceSensorEnvironment(
            Pose{0.0, 0.0, 0.0}, range, environment_size);

        EXPECT_TRUE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, 10.0, tolerance);
        EXPECT_EQ(result.detection_type, DetectionType::OBJECTDETECTED);
    }

    TEST(GeometryTest, RaycastReportsEnvironmentBorderCloserThanMinDistance){
        const Rectangle environment_size {10.0, 20.0};
        const std::array<double, 2> range {11.0, 20.0};

        const RaycastResult result = calculateDistanceSensorEnvironment(
            Pose{0.0, 0.0, 0.0}, range, environment_size);

        EXPECT_FALSE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, 10.0, tolerance);
        EXPECT_EQ(result.detection_type, DetectionType::OBJECTOUTOFRANGE);
    }

    TEST(GeometryTest, RaycastReportsEnvironmentBorderFartherThanMaxDistance){
        const Rectangle environment_size {10.0, 20.0};
        const std::array<double, 2> range {0.0, 9.0};

        const RaycastResult result = calculateDistanceSensorEnvironment(
            Pose{0.0, 0.0, 0.0}, range, environment_size);

        EXPECT_FALSE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, 10.0, tolerance);
        EXPECT_EQ(result.detection_type, DetectionType::OBJECTOUTOFRANGE);
    }

    TEST(GeometryTest, RaycastDetectsEnvironmentBorderAtSensorPosition){
        const Rectangle environment_size {10.0, 20.0};
        const std::array<double, 2> range {0.0, 20.0};

        const RaycastResult result = calculateDistanceSensorEnvironment(
            Pose{10.0, 0.0, 0.0}, range, environment_size);

        EXPECT_TRUE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, 0.0, tolerance);
        EXPECT_EQ(result.detection_type, DetectionType::OBJECTDETECTED);
    }

    TEST(GeometryTest, RaycastDetectsOppositeEnvironmentBorderFromBoundary){
        const Rectangle environment_size {10.0, 20.0};
        const std::array<double, 2> range {0.0, 20.0};

        const RaycastResult result = calculateDistanceSensorEnvironment(
            Pose{10.0, 0.0, pi}, range, environment_size);

        EXPECT_TRUE(result.object_detected);
        EXPECT_NEAR(result.distance_to_object, 20.0, tolerance);
        EXPECT_EQ(result.detection_type, DetectionType::OBJECTDETECTED);
    }

    TEST(GeometryTest, RaycastRejectsSensorOutsideEnvironment){
        const Rectangle environment_size {10.0, 20.0};
        const std::array<double, 2> range {0.0, 20.0};

        EXPECT_THROW(calculateDistanceSensorEnvironment(
            Pose{10.1, 0.0, 0.0}, range, environment_size), std::invalid_argument);
    }

    TEST(GeometryTest, RaycastRejectsSensorAboveEnvironment){
        const Rectangle environment_size {10.0, 20.0};
        const std::array<double, 2> range {0.0, 20.0};

        EXPECT_THROW(calculateDistanceSensorEnvironment(
            Pose{0.0, 5.1, 0.0}, range, environment_size), std::invalid_argument);
    }
}//robot_simulation
