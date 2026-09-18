#pragma once

namespace robot_simulation{
    class Simulation{
        private:
        double step_time_;

        public:
        Simulation(double dt){
            step_time_ = dt;
        }

        void update(); // tutaj moze cos w sumie zwracac
    };
}// namespace robot_simulation