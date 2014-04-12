#ifndef HAL_QUADROTOR_WAYPOINT_H
#define HAL_QUADROTOR_WAYPOINT_H

// Messages used and produced by this controller
#include <hal_quadrotor/ControlWaypoint.h>

// Basic HAL : provides Controller class
#include "Controller.h"

namespace hal_quadrotor
{
    class Waypoint : public Controller
    {

    private:
        
        // If this is the first iteration since reset
        bool first;
 
        static const double _Kxy        = 0.9;         // position proportional constant
        static const double _Kv         = 0.09;        // velocity proportional constant   
        static const double _Kiz        = 0.0008;      // altitude integrative constant
        static const double _Kpz        = 0.03;        // altitude proportional constant   
        static const double _Kdz        = 0.04;        // altitude derivative constant        
        static const double _th_hover   = 0.59;        // throttle hover offset
        static const double _maxtilt    = 0.34;        // max pitch/roll angle
        static const double _Kya        = 6.0;         // yaw proportional constant
        static const double _maxyawrate = 4.4;         // max allowed yaw rate
        static const double _maxv       = 5.0;         // max allowed xy velocity

        // PID parameters
        double iz, ez, sp[4];

        // Configure data broadcast at a given rate (<= 0.0 means disable)
        bool Receive(ControlWaypoint::Request &req, ControlWaypoint::Response &res);

    public:

        // Constructor
        Waypoint(ros::NodeHandle &node, std::string name);

        // Get new control from current state and time step
        Control Update(const State &state, const double &dt);

        // Reset the current state
        void Reset();
    };
}

#endif