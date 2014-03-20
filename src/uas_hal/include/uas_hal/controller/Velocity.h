#ifndef UAS_HAL_CONTROLLER_VELOCITY_H
#define UAS_HAL_CONTROLLER_VELOCITY_H

// Basic HAL : provides Controller class
#include <uas_hal/controller/Controller.h>

namespace uas_hal
{
    class Velocity : public Controller
    {

    private:

        // If this is the first iteration since reset
        bool first;

        static const double _Kvp        = 0.25;     // xy velocity proportional constant 
        static const double _Kvi        = 0.003;    // xy velocity integrative constant 
        static const double _Kvd        = 0.05;     // xy velocity derivative constant
        static const double _Kwp        = -0.2;     // z velocity proportional constant 
        static const double _Kwi        = -0.002;   // z velocity integrative constant 
        static const double _Kwd        = -0.0;     // z velocity derivative constant
        static const double _th_hover   = 0.59;     // throttle hover offset
        static const double _maxtilt    = 0.34;     // max pitch/roll angle
        static const double _Kya        = 6;        // yaw proportional constant
        static const double _maxyawrate = 4.4;      // max allowed yaw rate
        static const double _maxv       = 3;        // max allowed xy velocity

        // PID parameters
        double ei[3], ep[3], sp[4];

    public:

        // Connstructor ta
        Velocity();

        // Set the goal
        void SetGoal(double u, double v, double w, double yaw);

        // Pass the state and discrete time step and receive a control back
        void Update(State *state, double dt, Control *ctl);

        // Reset the controller
        void Reset();
    };
}

#endif