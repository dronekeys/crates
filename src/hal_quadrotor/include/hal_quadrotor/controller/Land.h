#ifndef HAL_QUADROTOR_LAND_H
#define HAL_QUADROTOR_LAND_H

// Messages used and produced by this controller
#include <hal_quadrotor/ControlLand.h>

// Base class type
#include "Controller.h"

namespace hal_quadrotor
{
    class Land : public Controller
    {

    private:

        // Configure data broadcast at a given rate (<= 0.0 means disable)
        bool Receive(ControlLand::Request &req, ControlLand::Response &res);

    public:

        // Constructor
        Land(ros::NodeHandle &node, std::string name);

        // Get new control from current state and time step
        Control Update(const State &state, const double &dt);

        // Reset the current state
        void Reset();
    };
}

#endif