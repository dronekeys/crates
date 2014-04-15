#ifndef HAL_PLATFORM_QUADROTOR_EMERGENCY_H
#define HAL_PLATFORM_QUADROTOR_EMERGENCY_H

// Base controller type
#include <hal/Controller.h>

// Messages used by this controller
#include <hal_platform_quadrotor/State.h>
#include <hal_platform_quadrotor/Control.h>

// Services used by this controller
#include <hal_platform_quadrotor/Emergency.h>

namespace hal
{
    namespace controller
    {
        //! A quadrotor Emergency controller
        /*!
          A more elaborate class description.
        */
        class Emergency : public Controller<hal_platform_quadrotor::State, hal_platform_quadrotor::Control,
            hal_platform_quadrotor::Emergency::Request, hal_platform_quadrotor::Emergency::Response>
        {

        private:

            //! Callback for goal update
            /*!
              \param req the goal request
              \param res the goal response
              \return whether the control was accepted
            */
            bool Receive(hal_platform_quadrotor::Emergency::Request &req, hal_platform_quadrotor::Emergency::Response &res);

        public:

            /// Default constructor takes no arguments
            Emergency();

            //! Control update implementations
            /*!
              \param state the current platform state
              \param dt the discrete time tick
              \return the control required to move from the current state to the goal 
            */
            hal_platform_quadrotor::Control Update(const hal_platform_quadrotor::State &state, const double &dt);

            /// Reset the controller
            void Reset();

        };
    }
}

#endif