#pragma once
#include <robot_simulation/types.hpp>
#include <robot_simulation/simulation.hpp>
#include <optional>

namespace robot_simulation{
    struct SimulationSnapshot{
        Pose robot_pose;
        Velocity robot_velocity;
        std::size_t simulation_step;
        double simulation_time;
        bool stopped;
        std::optional<SimulationUpdate> update;
    };

}//robot_simulation