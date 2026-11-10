#pragma once
#include <robot_simulation/types.hpp>
#include <robot_simulation/scene_config.hpp>
#include <robot_simulation/simulation.hpp>
#include <robot_simulation/simulation_snapshot.hpp>
#include <robot_simulation/sensor_description.hpp>
#include <memory>
#include <vector>

namespace robot_simulation {
    class SimulationRunner{
        public:
        explicit SimulationRunner(const SceneConfig& config);

        ~SimulationRunner();

        SimulationUpdate update();

        void reset();

        Pose getRobotPose() const;
        
        Velocity getRobotVelocity() const;

        std::size_t getSimulationStep() const;
    
        double getSimulationTime() const;

        bool isStopped() const;

        SimulationSnapshot getSimulationSnapshot() const;

        SceneConfig getSceneConfig() const;

        std::vector<SensorDescription> getSensorDescriptions() const;

        private:
        struct Session;
        std::unique_ptr<Session> session_;
        SceneConfig config_;

    };
}//robot_simulation