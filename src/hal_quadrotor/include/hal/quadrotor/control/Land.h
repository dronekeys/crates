#ifndef HAL_MODEL_QUADROTOR_LAND_H
#define HAL_MODEL_QUADROTOR_LAND_H

// Base controller type
#include <hal/quadrotor/control/Controller.h>

// Messages used by this controller
#include <hal_model_quadrotor/Land.h>

namespace hal
{
    namespace model
    {
        //! A quadrotor Emergency controller
        /*!
          A more elaborate class description.
        */
        class Land : public Controller
        {

        private:

            /// If this is the first iteration since reset
            bool first; 
            
            /// If we have reached the goal
            bool reach;

            // PID parameters
            double iz;
            double ez;
            double sp[4];

        public:

            //! Callback for goal update
            /*!
              \param req the goal request
              \param res the goal response
              \return whether the control was accepted
            */
            bool SetGoal(
                hal_model_quadrotor::Land::Request& req, 
                hal_model_quadrotor::Land::Response& res
            );

            //! Obtain control from state and timestep
            /*!
              \param state the current platform state
              \param dt the discrete time step
              \param control the output control from the controller
              \return if the state could be updated
            */
            bool Update(const hal_model_quadrotor::State &state, 
                double dt, hal_model_quadrotor::Control &control);

            //! Goal reach implementations
            /*!
              \return Whether the goal has been reached
            */
            bool HasGoalBeenReached();
        };
    }
}

#endif