#ifndef UAS_HAL_CONTROLLER_ANGLESHEIGHT_H
#define UAS_HAL_CONTROLLER_ANGLESHEIGHT_H

namespace uas_hal
{
    namespace controller
    {
        // Current goal
        struct Setpoint
        {
            double x;
            double y;
            double z;
        };

        class AnglesHeight : public Controller
        {
        public:

        	// Connstructor
			AnglesHeight(double dt);

        private:

        	// Constant parameters
            static const double _Kv         = 0.09;     // xy velocity proportional constant 
            static const double _maxtilt    = 0.34;     // max pitch/roll angle
            static const double _Kya        = 6.0;      // yaw proportional constant
            static const double _maxyawrate = 4.4;      // max allowed yaw rate
            static const double _Kiz        = 0.0008;   // altitude integrative constant
            static const double _Kpz        = 0.03;     // altitude proportional constant     
            static const double _Kdz        = 0.04;     // altitude derivative constant
            static const double _thoffset   = 0.59;     // throttle hover offset

            // current waypoint and heading
            Goal wp;      

            // Intermediary variables
            double iz;      // altitude PID integrator
            double ez;      // altitude PID error
            double DT;      // control timestep
        };
    }
}

#endif