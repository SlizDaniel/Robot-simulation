#pragma once
#include <robot_simulation/types.hpp>

namespace robot_simulation{

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

        DistanceSensor(double min_distance, double max_distance,
            Pose to_robot_position, double field_of_view);

        double getSensorMaxDistance () const {return max_distance_;}

        double getSensorMinDistance () const {return min_distance_;}

        const Pose getRelativeToRobotPose () const {return relativeToRobotPose_;}

        Pose getWorldPose (const Pose& robot_pose) const;
    };
}
