#ifndef HAL_PLATFORM_QUADROTOR_VELOCITYHEIGHT_H
#define HAL_PLATFORM_QUADROTOR_VELOCITYHEIGHT_H

// Base controller type
#include <hal/Controller.h>

// Messages used by this controller
#include <hal_platform_quadrotor/State.h>
#include <hal_platform_quadrotor/Control.h>

// Services used by this controller
#include <hal_platform_quadrotor/VelocityHeight.h>

namespace hal
{
    namespace controller
    {
        //! A quadrotor Emergency controller
        /*!
          A more elaborate class description.
        */
        class VelocityHeight : public Controller<hal_platform_quadrotor::State, hal_platform_quadrotor::Control,
            hal_platform_quadrotor::VelocityHeight::Request, hal_platform_quadrotor::VelocityHeight::Response>
        {

        private:

            // If this is the first iteration since reset
            bool first;

            static const double _Kvp        = 0.25;     // xy velocity proportional constant 
            static const double _Kvi        = 0.003;    // xy velocity integrative constant 
            static const double _Kvd        = 0.05;     // xy velocity derivative constant          
            static const double _Kiz        = 0.0008;   // altitude integrative constant
            static const double _Kpz        = 0.03;     // altitude proportional constant   
            static const double _Kdz        = 0.04;     // altitude derivative constant
            static const double _th_hover   = 0.59;     // throttle hover offset
            static const double _maxtilt    = 0.34;     // max pitch/roll angle
            static const double _Kya        = 6.0;      // yaw proportional constant
            static const double _maxyawrate = 4.4;      // max allowed yaw rate
            static const double _maxv       = 3.0;      // max allowed xy velocity

            // PID parameters
            double iz;
            double ez;
            double ei[2];
            double ep[2];
            double sp[4];

            //! Callback for goal update
            /*!
              \param req the goal request
              \param res the goal response
              \return whether the control was accepted
            */
            bool Receive(
                hal_platform_quadrotor::VelocityHeight::Request& req, 
                hal_platform_quadrotor::VelocityHeight::Response& res
            );

        public:

            /// Constructor
            VelocityHeight();

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