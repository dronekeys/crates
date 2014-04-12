#ifndef HAL_QUADROTOR_HOVER_H
#define HAL_QUADROTOR_HOVER_H

// Messages used and produced by this controller
#include <hal_quadrotor/ControlHover.h>

// Base type
#include "Controller.h"

namespace hal_quadrotor
{
    class Hover : public Controller
    {

    private:

        // Configure data broadcast at a given rate (<= 0.0 means disable)
        bool Receive(ControlHover::Request &req, ControlHover::Response &res);

    public:

        // Constructor
        Hover(ros::NodeHandle &node, std::string name);

        // Get new control from current state and time step
        Control Update(const State &state, const double &dt);

        // Reset the current state
        void Reset();
    };
}

#endif