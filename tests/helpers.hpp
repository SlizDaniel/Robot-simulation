#pragma once
#include <robot_simulation/types.hpp>
#include <gtest/gtest.h>

namespace robot_simulation{
    constexpr double tolerance = 1e-9;

    inline void expectPointNear(const Point& actual, const Point& expected, double tolerance = 1e-9){
        EXPECT_NEAR(actual.x, expected.x, tolerance);
        EXPECT_NEAR(actual.y, expected.y, tolerance);
    }
    inline void expectCornersNear(const RectangleCorners& actual, const RectangleCorners& expected, double tolerance = 1e-9){
        for (std::size_t i = 0; i<actual.size(); i++){
            expectPointNear(actual[i], expected[i], tolerance);
        }
    }
}   
