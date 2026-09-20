#include <gtest/gtest.h>
#include <robot_simulation/geometry.hpp>
#include <robot_simulation/types.hpp>
#include "helpers.hpp"
#include <robot_simulation/math_constans.hpp>

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
}//robot_simulation
