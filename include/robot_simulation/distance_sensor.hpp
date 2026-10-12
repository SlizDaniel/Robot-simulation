#pragma once
#include <robot_simulation/types.hpp>
#include <robot_simulation/math_constants.hpp>
#include <cmath>
#include <stdexcept>

namespace robot_simulation{

    class Environment;

    enum class DetectionType {
        OBJECTDETECTED,
        OBJECTOUTOFRANGE,
        NOOBJECTDETECTED
    };

    struct RaycastResult {
        bool object_detected;
        double distance_to_object;
        DetectionType detection_type;
    };

class DistanceSensor{
        private:
        double max_distance_;
        double min_distance_;
        double field_of_view_;
        Pose relativeToRobotPose_;

        public:

        DistanceSensor(double min_distance = 2.0, double max_distance = 20.0,
            Pose to_robot_position = {0.0, 0.0, 0.0}, double field_of_view = pi/2);

        double getSensorMaxDistance () const {return max_distance_;}

        double getSensorMinDistance () const {return min_distance_;}

        const Pose getRelativeToRobotPose () const {return relativeToRobotPose_;}

        Pose getWorldPose (const Pose& robot_pose) const;

        RaycastResult nearestObstacleDetected (const Environment& environment,
            int ray_count, const Pose& robot_pose) const;

        RaycastResult nearestEnvironmentBorderDetected (const Environment& environment, 
            int ray_count, const Pose& robot_pose) const;

        RaycastResult nearestObjectDetected (const Environment& environment, 
            int ray_count, const Pose& robot_pose) const;

        void setToRobotRotation(double theta) {
            if (!std::isfinite(theta)){
                throw std::invalid_argument("Sensor rotation must be finite");
            }
            relativeToRobotPose_.theta = theta;
        }
    };
}
