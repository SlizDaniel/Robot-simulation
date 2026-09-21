#pragma once
#include <robot_simulation/types.hpp>

namespace robot_simulation{

    class Robot{
        private:

        Pose pose_;

        Velocity velocity_;

        Rectangle size_;

        public:

        Robot(Pose pose, Velocity velocity, Rectangle size);

        Pose getPose() const {return pose_;}

        Velocity getVelocity() const {return velocity_;}

        Rectangle getSize() const {return size_;}

        RectangleCorners getRobotCorners() const;

        void move(double dt);

        void stop();
    };
}// namespace robot_simulation