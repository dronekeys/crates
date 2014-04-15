#ifndef HAL_PLATFORM_QUADROTOR_WAYPOINT_H
#define HAL_PLATFORM_QUADROTOR_WAYPOINT_H

// Base controller type
#include <hal/Controller.h>

// Messages used by this controller
#include <hal_platform_quadrotor/State.h>
#include <hal_platform_quadrotor/Control.h>

// Services used by this controller
#include <hal_platform_quadrotor/Waypoint.h>

namespace hal
{
    namespace controller
    {
        //! A quadrotor Emergency controller
        /*!
          A more elaborate class description.
        */
        class Waypoint : public Controller<hal_platform_quadrotor::State, hal_platform_quadrotor::Control,
            hal_platform_quadrotor::Waypoint::Request, hal_platform_quadrotor::Waypoint::Response>
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
                hal_platform_quadrotor::Waypoint::Request& req, 
                hal_platform_quadrotor::Waypoint::Response& res
            );

        public:

            /// Constructor
            Waypoint();

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