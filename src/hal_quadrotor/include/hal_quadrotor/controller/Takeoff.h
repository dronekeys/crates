#ifndef HAL_QUADROTOR_TAKEOFF_H
#define HAL_QUADROTOR_TAKEOFF_H

// Messages used and produced by this controller
#include <hal_quadrotor/ControlTakeoff.h>

// Base type
#include "Controller.h"

namespace hal_quadrotor
{
    class Takeoff : public Controller
    {

    private:

        // Configure data broadcast at a given rate (<= 0.0 means disable)
        bool Receive(ControlTakeoff::Request &req, ControlTakeoff::Response &res);

    public:

        // Constructor
        Takeoff(ros::NodeHandle &node, std::string name);
        
        // Get new control from current state and time step
        Control Update(const State &state, const double &dt);

        // Reset the current state
        void Reset();
    };
}

#endif