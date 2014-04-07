#ifndef UAS_HAL_CONTROLLER_ANGLESHEIGHT_H
#define UAS_HAL_CONTROLLER_ANGLESHEIGHT_H

// Basic HAL : provides Controller class
#include <uas_hal/controller/ControllerType.h>

namespace uas_hal
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

    public:

        // Connstructor ta
        AnglesHeight();

        // Set the goal
        void SetGoal(double roll, double pitch, double yaw, double height);

        // Pass the state and discrete time step and receive a control back
        void Update(State *state, double dt, Control *ctl);

        // Reset the controller
        void Reset();
    };
}

#endif