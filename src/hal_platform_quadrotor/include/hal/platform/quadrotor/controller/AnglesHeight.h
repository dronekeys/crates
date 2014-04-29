#ifndef HAL_PLATFORM_QUADROTOR_ANGLESHEIGHT_H
#define HAL_PLATFORM_QUADROTOR_ANGLESHEIGHT_H

// Base controller type
#include <hal/controller/Controller.h>

// Messages used by this controller
#include <hal_platform_quadrotor/State.h>
#include <hal_platform_quadrotor/Control.h>
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
            
            /// If we have reached the goal
            bool reach;

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
            AnglesHeight(ros::NodeHandle& node, const char* name);

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

            //! Goal reach implementations
            /*!
              \return Whether the goal has been reached
            */
            bool HasGoalBeenReached();

            /// Reset the current state
            void Reset();
        };
    }
}

#endif