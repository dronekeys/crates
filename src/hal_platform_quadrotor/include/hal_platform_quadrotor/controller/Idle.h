#ifndef HAL_QUADROTOR_IDLE_H
#define HAL_QUADROTOR_IDLE_H

// Messages used and produced by this controller
#include <hal_quadrotor/ControlIdle.h>

// Base type
#include "Controller.h"

namespace hal_quadrotor
{
    class Idle : public Controller
    {

    private:

        // Configure data broadcast at a given rate (<= 0.0 means disable)
        bool Receive(ControlIdle::Request &req, ControlIdle::Response &res);

    public:

        // Constructor
        Idle(ros::NodeHandle &node, std::string name);

        // Get new control from current state and time step
        Control Update(const State &state, const double &dt);

        // Reset the current state
        void Reset();
    };
}

#endif