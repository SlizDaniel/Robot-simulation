#include <robot_simulation/robot.hpp>
#include <cmath>

namespace robot_simulation{

    void Robot::stop(){
        velocity_ = {0.0 , 0.0};        
    }

    void Robot::move(double dt){
        double dx = 0.0;
        double dy = 0.0;
        if (std::abs(velocity_.angularVelocity) * dt < 1e-6){
            dy = velocity_.linearVelocity * dt * std::sin(pose_.theta);
            dx = velocity_.linearVelocity * dt *std::cos(pose_.theta);
            pose_.theta +=velocity_.angularVelocity * dt;
        }
        else{
            double theta_next = pose_.theta + velocity_.angularVelocity * dt;
            dy = (velocity_.linearVelocity/velocity_.angularVelocity) * (std::cos(pose_.theta)-std::cos(theta_next));
            dx = (velocity_.linearVelocity/velocity_.angularVelocity) * (std::sin(theta_next)-std::sin(pose_.theta));
            pose_.theta = theta_next;
        }
        pose_.x += dx;
        pose_.y += dy;
    }

    Point calculateDxDy (double length, double width, double alpha){
        double radius = std::sqrt(std::pow(length/2, 2) + std::pow(width/2, 2));
        double dx = radius * std::cos(alpha);
        double dy = radius * std::sin(alpha);
        return(Point{dx, dy});
    }

    RectangleCorners Robot::getRobotCorners() const {
        double theta_a = std::atan(size_.width/size_.length);
        double theta_b = -theta_a;
        double theta_d = M_PI - theta_a;
        double theta_c = - theta_d;

        Point A; Point B; Point C; Point D;
        A.x = pose_.x + calculateDxDy(size_.length, size_.width, pose_.theta + theta_a).x;
        A.y = pose_.y + calculateDxDy(size_.length, size_.width, pose_.theta + theta_a).y;
        B.x = pose_.x + calculateDxDy(size_.length, size_.width, pose_.theta + theta_b).x;
        B.y = pose_.y + calculateDxDy(size_.length, size_.width, pose_.theta + theta_b).y;
        C.x = pose_.x + calculateDxDy(size_.length, size_.width, pose_.theta + theta_c).x;
        C.y = pose_.y + calculateDxDy(size_.length, size_.width, pose_.theta + theta_c).y;
        D.x = pose_.x + calculateDxDy(size_.length, size_.width, pose_.theta + theta_d).x;
        D.y = pose_.y + calculateDxDy(size_.length, size_.width, pose_.theta + theta_d).y;

        return RectangleCorners{A, B, C, D};
    }
}