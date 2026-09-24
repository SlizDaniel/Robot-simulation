#pragma once
#include <robot_simulation/types.hpp>
#include <vector>

namespace robot_simulation{

    class Robot{
        private:

        Pose pose_;

        Velocity velocity_;

        Rectangle size_;

        std::vector<Pose> trajectory_;

        void addPointToTrajectory(Pose pose) {trajectory_.push_back(pose);}

        public:

        Robot(Pose pose, Velocity velocity, Rectangle size);

        Pose getPose() const {return pose_;}

        Velocity getVelocity() const {return velocity_;}

        Rectangle getSize() const {return size_;}

        RectangleCorners getRobotCorners() const;

        void move(double dt);

        void stop();

        const std::vector<Pose>& getTrajectory() const {return trajectory_;}

        Pose robotNextPose(double dt) const;

        void setVelocity(double linear_velocity, double angular_velocity);
    };
}// namespace robot_simulation