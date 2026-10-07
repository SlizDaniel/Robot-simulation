#include <robot_simulation/geometry.hpp>
#include <robot_simulation/types.hpp>
#include <cmath>
#include <array>
#include <stdexcept>
#include <algorithm>
#include <robot_simulation/math_constants.hpp>

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

    RaycastResult calculateDistanceSensorObstacle (const Pose& distance_sensor_pose,
            const std::array<double,2>& min_max_sensor_distance,  const Obstacle& obstacle){

        const Pose obstacle_pose = obstacle.get_obstacle_pose();
        const double relative_to_obstacle_sensor_x = distance_sensor_pose.x - obstacle_pose.x;
        const double relative_to_obstacle_sensor_y = distance_sensor_pose.y - obstacle_pose.y;
        const double sin_theta = std::sin(obstacle_pose.theta);
        const double cos_theta = std::cos(obstacle_pose.theta);
        const double local_sensor_obstacle_x = cos_theta * relative_to_obstacle_sensor_x +
        sin_theta * relative_to_obstacle_sensor_y;
        const double local_sensor_obstacle_y = cos_theta * relative_to_obstacle_sensor_y -
        sin_theta * relative_to_obstacle_sensor_x;
        const double ray_direction_x = std::cos(-obstacle_pose.theta + distance_sensor_pose.theta);
        const double ray_direction_y = std::sin(-obstacle_pose.theta + distance_sensor_pose.theta);
        double t_collision = 0.0;
        if (std::abs(ray_direction_x) < epsilon){
            if(local_sensor_obstacle_x < -obstacle.get_obstacle_size().length/2 ||
                local_sensor_obstacle_x > obstacle.get_obstacle_size().length/2) {
                    return {false, infinity, DetectionType::NOOBJECTDETECTED};
                }
            if (ray_direction_y>= 0){
                t_collision = (-obstacle.get_obstacle_size().width/2) - local_sensor_obstacle_y;
                if (t_collision < 0){
                    return {false, infinity, DetectionType::OBJECTOUTOFRANGE};
                }
                if (t_collision >= min_max_sensor_distance[0] &&
                    t_collision <= min_max_sensor_distance[1]){
                        return{true, t_collision, DetectionType::OBJECTDETECTED};
                    }
                    return{false, t_collision, DetectionType::OBJECTOUTOFRANGE};
            }
            if (ray_direction_y < 0){
                t_collision = local_sensor_obstacle_y - (obstacle.get_obstacle_size().width/2);
                if (t_collision < 0){
                    return {false, infinity, DetectionType::OBJECTOUTOFRANGE};
                }
                if (t_collision >= min_max_sensor_distance[0] &&
                    t_collision <= min_max_sensor_distance[1]){
                        return{true, t_collision, DetectionType::OBJECTDETECTED};
                    }
                    return{false, t_collision, DetectionType::OBJECTOUTOFRANGE};
            }
        }
        if (std::abs(ray_direction_y) < epsilon){
            if(local_sensor_obstacle_y < -obstacle.get_obstacle_size().width/2 ||
                local_sensor_obstacle_y > obstacle.get_obstacle_size().width/2) {
                    return {false, infinity, DetectionType::NOOBJECTDETECTED};
            }
            if (ray_direction_x >= 0){
                t_collision = (-obstacle.get_obstacle_size().length/2) - local_sensor_obstacle_x;
                if (t_collision < 0){
                    return {false, infinity, DetectionType::OBJECTOUTOFRANGE};
                }
                if (t_collision >= min_max_sensor_distance[0] &&
                    t_collision <= min_max_sensor_distance[1]){
                        return{true, t_collision, DetectionType::OBJECTDETECTED};
                    }
                    return{false, t_collision, DetectionType::OBJECTOUTOFRANGE};
            }
            if (ray_direction_x < 0){
                t_collision = local_sensor_obstacle_x - (obstacle.get_obstacle_size().length/2);
                if (t_collision < 0){
                    return {false, infinity, DetectionType::OBJECTOUTOFRANGE};
                }
                if (t_collision >= min_max_sensor_distance[0] &&
                    t_collision <= min_max_sensor_distance[1]){
                        return{true, t_collision, DetectionType::OBJECTDETECTED};
                    }
                    return{false, t_collision, DetectionType::OBJECTOUTOFRANGE};
            }
        }
        const double t1_x = ((-obstacle.get_obstacle_size().length/2) - local_sensor_obstacle_x)
                            / ray_direction_x;
        const double t2_x = ((obstacle.get_obstacle_size().length/2) - local_sensor_obstacle_x)
                            / ray_direction_x;
        const double t1_y = ((-obstacle.get_obstacle_size().width/2) - local_sensor_obstacle_y)
                            / ray_direction_y;
        const double t2_y = ((obstacle.get_obstacle_size().width/2) - local_sensor_obstacle_y)
                            / ray_direction_y;
        const double tx_near = std::min(t1_x, t2_x);
        const double tx_far = std::max(t1_x, t2_x);
        const double ty_near = std::min(t1_y, t2_y);
        const double ty_far = std::max(t1_y, t2_y);
        if (tx_far < ty_near || tx_near > ty_far){
            return {false, infinity, DetectionType::NOOBJECTDETECTED};
        }
        t_collision = std::max(tx_near, ty_near);
        if (t_collision < 0){
            return {false, infinity, DetectionType::OBJECTOUTOFRANGE};
        }
        if (t_collision < min_max_sensor_distance[0] ||
            t_collision > min_max_sensor_distance[1]){
                return{false, t_collision, DetectionType::OBJECTOUTOFRANGE};
            }
        return {true, t_collision, DetectionType::OBJECTDETECTED};
    }
    
    RaycastResult calculateDistanceSensorEnvironment (const Pose& distance_sensor_pose,
        const std::array<double,2>& min_max_sensor_distance, const Rectangle& environment_size){
        if (distance_sensor_pose.x > environment_size.length / 2 || 
            distance_sensor_pose.x < -environment_size.length / 2 ||
            distance_sensor_pose.y > environment_size.width / 2 ||
            distance_sensor_pose.y < -environment_size.width / 2){
                throw std::invalid_argument("Sensor not inside simulation environment");
            }
        double direction_x = std::cos(distance_sensor_pose.theta);
        double direction_y = std::sin(distance_sensor_pose.theta);
        double t_collision = 0.0;
        if (std::abs(direction_x) < epsilon){
            if (direction_y > 0){
                t_collision = ((environment_size.width / 2) - distance_sensor_pose.y) / direction_y;
            }
            else {
                t_collision = (-(environment_size.width / 2) - distance_sensor_pose.y) / direction_y;}
        }
        else if (std::abs(direction_y) < epsilon){
            if (direction_x > 0){
                t_collision = ((environment_size.length / 2) - distance_sensor_pose.x) / direction_x;
            }
            else {
                t_collision = (-(environment_size.length / 2) - distance_sensor_pose.x) / direction_x;}
        }
        else{
            double t_x_right = ((environment_size.length / 2) - distance_sensor_pose.x) / direction_x;
            double t_x_left = (-(environment_size.length / 2) - distance_sensor_pose.x) / direction_x;
            double t_y_top = ((environment_size.width / 2) - distance_sensor_pose.y) / direction_y;
            double t_x = 0.0;
            double t_y = 0.0;
            double t_y_bottom = (-(environment_size.width / 2) - distance_sensor_pose.y) / direction_y;
            if (t_x_right > t_x_left){
                t_x = t_x_right;
            }
            else {t_x = t_x_left;}
            if (t_y_top > t_y_bottom){
                t_y = t_y_top;
            }
            else{t_y = t_y_bottom;}

            t_collision = std::min(t_y, t_x);
        }
        if (t_collision >= min_max_sensor_distance[0] && t_collision <= min_max_sensor_distance[1]){
            return {true, t_collision, DetectionType::OBJECTDETECTED};
        }
        return {false, t_collision, DetectionType::OBJECTOUTOFRANGE};
    };
}//robot_simulation
