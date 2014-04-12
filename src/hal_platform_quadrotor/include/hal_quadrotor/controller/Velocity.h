#ifndef HAL_QUADROTOR_VELOCITY_H
#define HAL_QUADROTOR_VELOCITY_H

// Messages used and produced by this controller
#include <hal_quadrotor/ControlVelocity.h>

// Basic HAL : provides Controller class
#include "Controller.h"

namespace hal_quadrotor
{
    class Velocity : public Controller
    {

    private:

        // If this is the first iteration since reset
        bool first;

        static const double _Kvp        =  0.25;     // xy velocity proportional constant 
        static const double _Kvi        =  0.003;    // xy velocity integrative constant 
        static const double _Kvd        =  0.05;     // xy velocity derivative constant
        static const double _Kwp        = -0.2;     // z velocity proportional constant 
        static const double _Kwi        = -0.002;   // z velocity integrative constant 
        static const double _Kwd        = -0.0;     // z velocity derivative constant
        static const double _th_hover   =  0.59;     // throttle hover offset
        static const double _maxtilt    =  0.34;     // max pitch/roll angle
        static const double _Kya        =  6.0;      // yaw proportional constant
        static const double _maxyawrate =  4.4;      // max allowed yaw rate
        static const double _maxv       =  3.0;      // max allowed xy velocity

        // PID parameters
        double ei[3], ep[3], sp[4];

        // Configure data broadcast at a given rate (<= 0.0 means disable)
        bool Receive(ControlVelocity::Request &req, ControlVelocity::Response &res);

    public:

        // Constructor
        Velocity(ros::NodeHandle &node, std::string name);

        // Get new control from current state and time step
        Control Update(const State &state, const double &dt);

        // Reset the current state
        void Reset();
    };
}

#endif