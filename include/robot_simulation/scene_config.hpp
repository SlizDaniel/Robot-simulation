#pragma once
#include <robot_simulation/types.hpp>
#include <robot_simulation/simulation_snapshot.hpp>
#include <robot_simulation/sensor_description.hpp>
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

std::string ConfigurationWithSensorsToJson(const SceneConfig& config,
    const std::vector<SensorDescription>& sensors);

std::string SnapshotToJson (const SimulationSnapshot& snapshot);
}//robot_simulation
