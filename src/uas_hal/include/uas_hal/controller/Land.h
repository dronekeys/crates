#ifndef UAS_HAL_CONTROLLER_LAND_H
#define UAS_HAL_CONTROLLER_LAND_H

// Basic HAL : provides Controller class
#include <uas_hal/controller/Controller.h>

namespace uas_hal
{
    class Land : public Controller
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

    public:

        // Connstructor ta
        Land();

        // Take off to a particular altitude
        void SetGoal(double altitude);

        // Pass the state and discrete time step and receive a control back
        void Update(State *state, double dt, Control *ctl);

        // Reset the controller
        void Reset();
    };
}

#endif