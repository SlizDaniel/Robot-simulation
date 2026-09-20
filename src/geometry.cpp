#include <robot_simulation/geometry.hpp>
#include <robot_simulation/types.hpp>
#include <cmath>
#include <robot_simulation/math_constans.hpp>

namespace robot_simulation{
    namespace{
    Point calculateDxDy (double length, double width, double alpha){
        double radius = std::sqrt(std::pow(length/2, 2) + std::pow(width/2, 2));
        double dx = radius * std::cos(alpha);
        double dy = radius * std::sin(alpha);
        return(Point{dx, dy});
    }
    }//namespace

    RectangleCorners calculateRectangleCorners(const Pose pose, const Rectangle size) {
        double theta_a = std::atan(size.width/size.length);
        double theta_b = -theta_a;
        double theta_d = pi - theta_a;
        double theta_c = - theta_d;

        Point A; Point B; Point C; Point D;
        A.x = pose.x + calculateDxDy(size.length, size.width, pose.theta + theta_a).x;
        A.y = pose.y + calculateDxDy(size.length, size.width, pose.theta + theta_a).y;
        B.x = pose.x + calculateDxDy(size.length, size.width, pose.theta + theta_b).x;
        B.y = pose.y + calculateDxDy(size.length, size.width, pose.theta + theta_b).y;
        C.x = pose.x + calculateDxDy(size.length, size.width, pose.theta + theta_c).x;
        C.y = pose.y + calculateDxDy(size.length, size.width, pose.theta + theta_c).y;
        D.x = pose.x + calculateDxDy(size.length, size.width, pose.theta + theta_d).x;
        D.y = pose.y + calculateDxDy(size.length, size.width, pose.theta + theta_d).y;

        return RectangleCorners{A, B, C, D};
    }
}//robot_simulation