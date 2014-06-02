#ifndef HAL_QUADROTOR_EMERGENCY_H
#define HAL_QUADROTOR_EMERGENCY_H

// Base controller type
#include <hal_quadrotor/control/Controller.h>

// Messages used by this controller
#include <hal_quadrotor/Emergency.h>

namespace hal
{
    namespace quadrotor
    {
        //! A quadrotor Emergency controller
        /*!
          A more elaborate class description.
        */
        class Emergency : public Controller
        {

        public:

            //! Callback for goal update
            /*!
              \param req the goal request
              \param res the goal response
              \return whether the control was accepted
            */
            bool SetGoal(
                hal_quadrotor::Emergency::Request& req, 
                hal_quadrotor::Emergency::Response& res
            );

            //! Obtain control from state and timestep
            /*!
              \param state the current platform state
              \param dt the discrete time step
              \param control the output control from the controller
              \return if the state could be updated
            */
            void Update(const hal_quadrotor::State &state, 
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