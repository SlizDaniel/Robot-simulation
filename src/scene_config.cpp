#include <robot_simulation/scene_config.hpp>
#include <robot_simulation/types.hpp>
#include <vector>
#include <string>

namespace robot_simulation{
    namespace {
        Pose parsePose(const json& json_data_pose) {
            return {
                json_data_pose.at("x").get<double>(),
                json_data_pose.at("y").get<double>(),
                json_data_pose.at("theta").get<double>()
            };
        }

        Rectangle parseRectangle(const json& json_data_size) {
            return {
                json_data_size.at("width").get<double>(),
                json_data_size.at("length").get<double>()
            };
        }

        Velocity parseVelocity(const json& json_data_velocity) {
            return {
                json_data_velocity.at("linearVelocity").get<double>(),
                json_data_velocity.at("angularVelocity").get<double>()
            };
        }

        RobotConfig parseRobotConfig(const json& json_data_robot) {
            return {
                parsePose(json_data_robot.at("pose")),
                parseRectangle(json_data_robot.at("size")),
                parseVelocity(json_data_robot.at("velocity"))
            };
        }

        ObstacleConfig parseObstacleConfig(const json& json_data_obstacle) {
            return {
                parsePose(json_data_obstacle.at("pose")),
                parseRectangle(json_data_obstacle.at("size"))
            };
        }

        std::vector<ObstacleConfig> parseObstacles(const json& json_data_obstacles) {
            std::vector<ObstacleConfig> obstacles;
            obstacles.reserve(json_data_obstacles.size());

            for (const json& json_data_obstacle : json_data_obstacles) {
                obstacles.push_back(parseObstacleConfig(json_data_obstacle));
            }

            return obstacles;
        }

        ControllerConfig parseControllerConfig(const json& json_data_controller) {
            return {
                json_data_controller.at("safe_distance").get<double>(),
                json_data_controller.at("critical_distance").get<double>(),
                json_data_controller.at("velocity_gain_critical").get<double>()
            };
        }

        json poseToJson(const Pose& pose) {
            return {
                {"x", pose.x},
                {"y", pose.y},
                {"theta", pose.theta}
            };
        }

        json rectangleToJson(const Rectangle& rectangle) {
            return {
                {"width", rectangle.width},
                {"length", rectangle.length}
            };
        }
    }

    SceneConfig parseJsonToSimulationConfiguration(const json& json_data) {
        const json& json_data_simulation = json_data.at("simulation");

        return {
            parseRectangle(json_data.at("environment")),
            parseRobotConfig(json_data.at("robot")),
            parseObstacles(json_data.at("obstacles")),
            json_data_simulation.at("dt").get<double>(),
            json_data_simulation.at("ray_count").get<int>(),
            parseControllerConfig(json_data.at("controller"))
        };
    }

    std::string ConfigurationToJson(const SceneConfig& config) {
        json json_data_obstacles = json::array();
        for (const ObstacleConfig& obstacle : config.obstacles) {
            json_data_obstacles.push_back({
                {"pose", poseToJson(obstacle.pose)},
                {"size", rectangleToJson(obstacle.size)}
            });
        }

        const json json_data = {
            {"environment", rectangleToJson(config.environment_size)},
            {"robot", {
                {"pose", poseToJson(config.robot.pose)},
                {"size", rectangleToJson(config.robot.size)},
                {"velocity", {
                    {"linearVelocity", config.robot.velocity.linearVelocity},
                    {"angularVelocity", config.robot.velocity.angularVelocity}
                }}
            }},
            {"obstacles", json_data_obstacles},
            {"simulation", {
                {"dt", config.dt},
                {"ray_count", config.ray_count}
            }},
            {"controller", {
                {"safe_distance", config.controller.safe_distance},
                {"critical_distance", config.controller.critical_distance},
                {"velocity_gain_critical", config.controller.velocity_gain_critical}
            }}
        };

        return json_data.dump(2);
    }
}//robot_simulation
