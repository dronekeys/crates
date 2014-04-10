#ifndef HAL_QUADROTOR_EMERGENCY_H
#define HAL_QUADROTOR_EMERGENCY_H

// Messages used and produced by this controller
#include <hal_quadrotor/ControlEmergency.h>

// Base type
#include "Controller.h"

namespace hal_quadrotor
{
    class Emergency : public Controller
    {

    private:

        // Configure data broadcast at a given rate (<= 0.0 means disable)
        bool Receive(ControlEmergency::Request &req, ControlEmergency::Response &res);

    public:

        // Constructor
        Emergency(ros::NodeHandle &node, std::string name);

        // Get new control from current state and time step
        Control Update(const State &state, const double &dt);

        // Reset the current state
        void Reset();

    };
}

#endif