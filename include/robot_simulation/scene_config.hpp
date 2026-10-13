#pragma once
#include <robot_simulation/types.hpp>
#include <vector>
#include <nlohmann/json.hpp>
#include <string>

namespace robot_simulation {

using json = nlohmann::json;

struct RobotConfig {
    Pose pose;
    Rectangle size;
    Velocity velocity;
};

struct ObstacleConfig {
    Pose pose;
    Rectangle size;
};

struct ControllerConfig {
    double safe_distance;
    double critical_distance;
    double velocity_gain_critical;
};

struct SceneConfig {
    Rectangle environment_size;
    RobotConfig robot;
    std::vector<ObstacleConfig> obstacles;

    double dt;
    int ray_count;

    ControllerConfig controller;
};

SceneConfig parseJsonToSimulationConfiguration (const json& json_data);

std::string ConfigurationToJson (const SceneConfig& config);
}//robot_simulation
