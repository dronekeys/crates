#ifndef HAL_PLATFORM_QUADROTOR_ANGLESHEIGHT_H
#define HAL_PLATFORM_QUADROTOR_ANGLESHEIGHT_H

// Base controller type
#include <hal/Controller.h>

// Messages used by this controller
#include <hal_platform_quadrotor/State.h>
#include <hal_platform_quadrotor/Control.h>

// Services used by this controller
#include <hal_platform_quadrotor/AnglesHeight.h>

// Convenience declarations
#define _ROLL   0 
#define _PITCH  1
#define _YAW    2
#define _HEIGHT 3

namespace hal
{
    namespace controller
    {
        //! A quadrotor Emergency controller
        /*!
          A more elaborate class description.
        */
        class AnglesHeight : public Controller<hal_platform_quadrotor::State, hal_platform_quadrotor::Control,
            hal_platform_quadrotor::AnglesHeight::Request, hal_platform_quadrotor::AnglesHeight::Response>
        {

        private:

            /// If this is the first iteration since reset
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
            double iz;
            double ez;
            double sp[4];

            //! Callback for goal update
            /*!
              \param req the goal request
              \param res the goal response
              \return whether the control was accepted
            */
            bool Receive(
                hal_platform_quadrotor::AnglesHeight::Request& req, 
                hal_platform_quadrotor::AnglesHeight::Response& res
            );

        public:

            /// Constructor
            AnglesHeight();

            //! Control update implementations
            /*!
              \param state the current platform state
              \param dt the discrete time tick
              \return the control required to move from the current state to the goal 
            */
            hal_platform_quadrotor::Control Update(
                const hal_platform_quadrotor::State &state, 
                const double &dt
            );

            /// Reset the current state
            void Reset();
        };
    }
}

#endif