#ifndef HAL_QUADROTOR_VELOCITY_H
#define HAL_QUADROTOR_VELOCITY_H

// Base controller type
#include <hal/quadrotor/control/Controller.h>

// Messages used by this controller
#include <hal_quadrotor/Velocity.h>

namespace hal
{
    namespace quadrotor
    {
        //! A quadrotor Emergency controller
        /*!
          A more elaborate class description.
        */
        class Velocity : public Controller
        {

        private:

            /// If this is the first iteration since reset
            bool first; 
            
            // PID parameters
            double ei[3];
            double ep[3];
            double sp[4];

        public:

            //! Callback for goal update
            /*!
              \param req the goal request
              \param res the goal response
              \return whether the control was accepted
            */
            bool SetGoal(
                hal_quadrotor::Velocity::Request& req, 
                hal_quadrotor::Velocity::Response& res
            );

            //! Obtain control from state and timestep
            /*!
              \param state the current platform state
              \param dt the discrete time step
              \param control the output control from the controller
              \return if the state could be updated
            */
            bool Update(const hal_quadrotor::State &state, 
                double dt, hal_quadrotor::Control &control);

            //! Goal reach implementations
            /*!
              \return Whether the goal has been reached
            */
            bool HasGoalBeenReached();
        };
    }
}

#endif