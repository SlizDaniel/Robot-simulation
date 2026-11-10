#include <robot_simulation/simulation_runner.hpp>
#include <robot_simulation/robot.hpp>
#include <robot_simulation/simulation.hpp>
#include <robot_simulation/distance_sensor.hpp>
#include <robot_simulation/scene_config.hpp>
#include <robot_simulation/types.hpp>
#include <robot_simulation/distance_sensors_front_setup.hpp>
#include <robot_simulation/controller.hpp>
#include <robot_simulation/avoid_obstacles_controller.hpp>
#include <robot_simulation/environment.hpp>
#include <robot_simulation/obstacle.hpp>
#include <robot_simulation/math_constants.hpp>
#include <memory>
#include <stdexcept>
#include <optional>


namespace robot_simulation{
    struct SimulationRunner::Session
    {
        Environment environment;
        Robot robot;
        DistanceSensorsFrontSetup front_sensors;
        std::unique_ptr <Simulation> simulation;
        std::unique_ptr <Controller> controller;
        std::optional<SimulationUpdate> last_update;

        bool stopped = false;

        explicit Session (const SceneConfig& config):
        environment(config.environment_size),
        robot(config.robot.pose, config.robot.velocity, config.robot.size),
        front_sensors(setupDistanceSensorsOnRobot(robot, pi/4.0)),
        controller(std::make_unique<AvoidObstaclesController>(config.controller.safe_distance,
                                                config.controller.critical_distance,
                                                config.controller.velocity_gain_critical))
        {
            for (const auto& obstacle : config.obstacles){
                environment.addObstacle(Obstacle(obstacle.pose, obstacle.size));
            }
            if (!environment.isRobotInside(robot)){
                throw std::invalid_argument("Robot outside environment!");
            }
            if (environment.collides(robot)){
                throw std::invalid_argument("Robot collides!");
            }
            simulation = std::make_unique <Simulation> (config.dt, robot, environment, 
                                                        config.ray_count);
        }
    };

    SimulationRunner::SimulationRunner(const SceneConfig& config):
            session_(std::make_unique<Session>(config)),
            config_(config){}

    SimulationRunner::~SimulationRunner() = default;

    SimulationUpdate SimulationRunner::update(){
        if (session_->stopped){
            throw std::logic_error("Session stopped! Reset required.");
        }

        Velocity initial_velocity = config_.robot.velocity;
        session_->robot.setVelocity(initial_velocity.linear_velocity, initial_velocity.angular_velocity);
        Velocity controller_velocity = session_->controller->calculateVelocity(session_->robot,
            session_->environment, config_.ray_count, session_->front_sensors);
        session_->robot.setVelocity(controller_velocity.linear_velocity,
            controller_velocity.angular_velocity);
        SimulationUpdate update = session_->simulation->update();
        session_->last_update = update;
        if (!update.step_accepted){
            session_->stopped = true;
        }
        return update;
    }

    void SimulationRunner::reset(){
        session_ = std::make_unique<Session>(config_);
    }

    Pose SimulationRunner::getRobotPose() const {return session_->robot.getPose();}

    Velocity SimulationRunner::getRobotVelocity() const {return session_->robot.getVelocity();}

    std::size_t SimulationRunner::getSimulationStep() const {return session_->simulation->getSimulationStep();}

    double SimulationRunner::getSimulationTime() const {return session_->simulation->getSimulationTime();}

    bool SimulationRunner::isStopped() const {return session_->stopped;}
    
    SimulationSnapshot SimulationRunner::getSimulationSnapshot() const {
        return{
            session_->robot.getPose(),
            session_->robot.getVelocity(),
            session_->simulation->getSimulationStep(),
            session_->simulation->getSimulationTime(),
            session_->stopped,
            session_->last_update
        };
    }

    SceneConfig SimulationRunner::getSceneConfig() const {return config_;}

    std::vector<SensorDescription> SimulationRunner::getSensorDescriptions() const {
        const auto& sensors = session_->robot.getDistanceSensors();
        const auto describe = [&sensors](const SensorInfo& info, const char* role) {
            const DistanceSensor& sensor = sensors.at(info.id);
            return SensorDescription{
                info.id,
                role,
                sensor.getRelativeToRobotPose(),
                sensor.getSensorMinDistance(),
                sensor.getSensorMaxDistance(),
                sensor.getSensorFieldOfView()
            };
        };

        return {
            describe(session_->front_sensors.front_lef_sensor, "front_left"),
            describe(session_->front_sensors.front_sensor, "front"),
            describe(session_->front_sensors.front_right_sensor, "front_right")
        };
    }
}//robot_simulation
