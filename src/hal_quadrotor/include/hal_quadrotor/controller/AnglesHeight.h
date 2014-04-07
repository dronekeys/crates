#ifndef HAL_QUADROTOR_ANGLESHEIGHT_H
#define HAL_QUADROTOR_ANGLESHEIGHT_H

// Messages used and produced by this controller
#include <hal_quadrotor/ControlAnglesHeight.h>

// Base type
#include "Controller.h"

// Convenience declarations
#define _ROLL   0 
#define _PITCH  1
#define _YAW    2
#define _HEIGHT 3

namespace hal_quadrotor
{
    class AnglesHeight : public Controller
    {

    private:

        // If this is the first iteration since reset
        bool first;

    	// Constant parameters
        static const double _Kv         = 0.09;     // xy velocity proportional constant 
        static const double _maxtilt    = 0.34;     // max pitch/roll angle
        static const double _Kya        = 6.0;      // yaw proportional constant
        static const double _maxyawrate = 4.4;      // max allowed yaw rate
        static const double _Kiz        = 0.0008;   // altitude integrative constant
        static const double _Kpz        = 0.03;     // altitude proportional constant     
        static const double _Kdz        = 0.04;     // altitude derivative constant
        static const double _th_hover   = 0.59;     // throttle hover offset

        // PID parameters
        double iz, ez, sp[4];

        // Configure data broadcast at a given rate (<= 0.0 means disable)
        bool Receive(ControlAnglesHeight::Request &req, ControlAnglesHeight::Response &res);

    public:

        // Constructor
        AnglesHeight(ros::NodeHandle &node, std::string name);

        // Get new control from current state and time step
        Control Update(const State &state, const double &dt);

        // Reset the current state
        void Reset();
    };
}

#endif