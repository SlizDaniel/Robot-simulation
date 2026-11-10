#include <robot_simulation/scene_config.hpp>
#include <robot_simulation/types.hpp>
#include <vector>
#include <string>
#include <utility>

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
                json_data_velocity.at("linear_velocity").get<double>(),
                json_data_velocity.at("angular_velocity").get<double>()
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
                    {"linear_velocity", config.robot.velocity.linear_velocity},
                    {"angular_velocity", config.robot.velocity.angular_velocity}
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

    std::string ConfigurationWithSensorsToJson(const SceneConfig& config,
        const std::vector<SensorDescription>& sensors) {
        json json_data = json::parse(ConfigurationToJson(config));
        json json_sensors = json::array();
        for (const SensorDescription& sensor : sensors) {
            json_sensors.push_back({
                {"id", sensor.id},
                {"role", sensor.role},
                {"relative_pose", poseToJson(sensor.relative_pose)},
                {"min_distance", sensor.min_distance},
                {"max_distance", sensor.max_distance},
                {"field_of_view", sensor.field_of_view}
            });
        }
        json_data["sensors"] = std::move(json_sensors);
        return json_data.dump(2);
    }

    std::string SnapshotToJson (const SimulationSnapshot& snapshot){
        const auto stop_reason_to_string = [](StopReason stop_reason) {
            switch (stop_reason) {
                case StopReason::None:
                    return "None";
                case StopReason::OutOfEnvironment:
                    return "OutOfEnvironment";
                case StopReason::ObstacleCollision:
                    return "ObstacleCollision";
            }

            return "Unknown";
        };

        const auto detection_type_to_string = [](DetectionType detection_type) {
            switch (detection_type) {
                case DetectionType::OBJECTDETECTED:
                    return "OBJECTDETECTED";
                case DetectionType::OBJECTOUTOFRANGE:
                    return "OBJECTOUTOFRANGE";
                case DetectionType::NOOBJECTDETECTED:
                    return "NOOBJECTDETECTED";
            }

            return "Unknown";
        };

        json json_update = nullptr;
        if (snapshot.update) {
            json raycast_results = json::array();
            for (std::size_t sensor_id = 0;
                 sensor_id < snapshot.update->raycast_results.size(); ++sensor_id) {
                const RaycastResult& result = snapshot.update->raycast_results[sensor_id];
                raycast_results.push_back({
                    {"sensor_id", sensor_id},
                    {"object_detected", result.object_detected},
                    {"distance_to_object", result.distance_to_object},
                    {"detection_type", detection_type_to_string(result.detection_type)}
                });
            }

            json_update = {
                {"stop_reason", stop_reason_to_string(snapshot.update->stop_reason)},
                {"step_accepted", snapshot.update->step_accepted},
                {"raycast_results", raycast_results}
            };
        }

        const json json_data = {
            {"robot_pose", poseToJson(snapshot.robot_pose)},
            {"robot_velocity", {
                {"linear_velocity", snapshot.robot_velocity.linear_velocity},
                {"angular_velocity", snapshot.robot_velocity.angular_velocity}
            }},
            {"simulation_step", snapshot.simulation_step},
            {"simulation_time", snapshot.simulation_time},
            {"stopped", snapshot.stopped},
            {"update", json_update}
        };

        return json_data.dump(2);
    }
}//robot_simulation
