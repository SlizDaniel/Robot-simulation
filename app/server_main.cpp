#include <robot_simulation/simulation_runner.hpp>
#include <mutex>
#include <httplib.h>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
    struct ServerState {
        std::mutex mutex;
        std::unique_ptr<robot_simulation::SimulationRunner> runner;
    };
}

int main() {
        httplib::Server server;
        ServerState state;

        server.Get("/api/health",
            [](const httplib::Request&,
            httplib::Response& response) {

                response.set_content(
                    R"({"status":"ok","backend":"cpp"})",
                    "application/json"
                );
            }
        );

        std::cout << "Server: http://127.0.0.1:8080\n";

        server.Post("/api/simulation/init", [&state](const httplib::Request& req, httplib::Response& res) {
            try {
                const auto json_data = robot_simulation::json::parse(req.body);
                const auto parsed_config_data = robot_simulation::parseJsonToSimulationConfiguration(json_data);
                auto runner_ = std::make_unique<robot_simulation::SimulationRunner>(parsed_config_data);
                const auto snapshot_json = robot_simulation::SnapshotToJson(
                    runner_->getSimulationSnapshot());

                {
                    std::lock_guard<std::mutex> lock(state.mutex);
                    state.runner = std::move(runner_);
                }
                res.set_content(snapshot_json, "application/json");
            } catch (const robot_simulation::json::exception& error) {
                res.status = 400;
                res.set_content(robot_simulation::json{{"error", error.what()}}.dump(),
                                "application/json");
            } catch (const std::invalid_argument& error) {
                res.status = 400;
                res.set_content(robot_simulation::json{{"error", error.what()}}.dump(),
                                "application/json");
            }
        });

        server.Get("/api/simulation/state", [&state](const httplib::Request&, httplib::Response& res) {
            std::lock_guard<std::mutex> lock(state.mutex);

            if (!state.runner) {
                res.status = 409;
                res.set_content(
                    robot_simulation::json{{"error", "Simulation is not initialized"}}.dump(),
                    "application/json");
                return;
            }

            const auto snapshot_json = robot_simulation::SnapshotToJson(
                state.runner->getSimulationSnapshot());
            res.set_content(snapshot_json, "application/json");
        });

        server.Get("/api/simulation/config", [&state](const httplib::Request&, httplib::Response& res) {
            robot_simulation::SceneConfig config;
            std::vector<robot_simulation::SensorDescription> sensors;

            {
                std::lock_guard<std::mutex> lock(state.mutex);

                if (!state.runner) {
                    res.status = 409;
                    res.set_content(
                        robot_simulation::json{{"error", "Simulation is not initialized"}}.dump(),
                        "application/json");
                    return;
                }

                config = state.runner->getSceneConfig();
                sensors = state.runner->getSensorDescriptions();
            }

            res.set_content(robot_simulation::ConfigurationWithSensorsToJson(config, sensors),
                            "application/json");
        });

        server.Post("/api/simulation/step", [&state](const httplib::Request&, httplib::Response& res) {
            std::string snapshot_json;

            {
                std::lock_guard<std::mutex> lock(state.mutex);

                if (!state.runner) {
                    res.status = 409;
                    res.set_content(
                        robot_simulation::json{{"error", "Simulation is not initialized"}}.dump(),
                        "application/json");
                    return;
                }

                if (state.runner->isStopped()) {
                    res.status = 409;
                    res.set_content(
                        robot_simulation::json{{"error", "Simulation is stopped; reset required"}}.dump(),
                        "application/json");
                    return;
                }

                state.runner->update();
                snapshot_json = robot_simulation::SnapshotToJson(
                    state.runner->getSimulationSnapshot());
            }

            res.set_content(snapshot_json, "application/json");
        });

        server.Post("/api/simulation/reset", [&state](const httplib::Request&, httplib::Response& res) {
            std::string snapshot_json;

            {
                std::lock_guard<std::mutex> lock(state.mutex);

                if (!state.runner) {
                    res.status = 409;
                    res.set_content(
                        robot_simulation::json{{"error", "Simulation is not initialized"}}.dump(),
                        "application/json");
                    return;
                }

                state.runner->reset();
                snapshot_json = robot_simulation::SnapshotToJson(
                    state.runner->getSimulationSnapshot());
            }

            res.set_content(snapshot_json, "application/json");
        });

        if (!server.set_mount_point("/", "./frontend")) {
        std::cerr << "Cannot find frontend directory\n";
        return 1;
        }

        if (!server.listen("127.0.0.1", 8080)) {
            std::cerr << "Failed to start server\n";
            return 1;
        }

        return 0;
}