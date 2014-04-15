#ifndef HAL_PLATFORM_QUADROTOR_H
#define HAL_PLATFORM_QUADROTOR_H

// System libraries
#include <string>

// Basic ROS stuff
#include <hal/Platform.h>

// State and control messages
#include <hal_platform_quadrotor/State.h>
#include <hal_platform_quadrotor/Control.h>

// Quadrotor-specific controllers
#include <hal_platform_quadrotor/controller/Emergency.h>

namespace hal
{
    namespace platform
    {
        class Quadrotor : public hal::platform::Platform
        {       

        private:

            /// Current state of the quadrotor
            hal_platform_quadrotor::State   state;

            /// Current control
            hal_platform_quadrotor::Control control;

            /// Last time at which the update clock was called
            double tick;

            /// Publishers
            ros::Publisher pubControl;  /*!< Contro; spublisher      */
            ros::Publisher pubState;    /*!< State publisher         */

            /// Timers
            ros::Timer timerUpdate;     /*!< Update loop timer       */
            ros::Timer timerState;      /*!< State broadcast timer   */
            ros::Timer timerControl;    /*!< Control broadcast timer */

            //! Timer callback for internal update loop
            /*!
              \param event Timer event
            */
            void Update(const ros::TimerEvent& event);

            //! Timer callback for control broadcast loop
            /*!
              \param event Timer event
            */
            void BroadcastControl(const ros::TimerEvent& event);

            //! Timer callback for state broadcast loop
            /*!
              \param event Timer event
            */
            void BroadcastState(const ros::TimerEvent& event);

        protected:

            //! Accept a control message from the HAL
            /*!
              \param ctl
            */
            virtual void OnControl(const hal_platform_quadrotor::Control &ctl) = 0;

        public:

            //! Create a new Quadrotor object
            /*!
              \param name name of the quadrotor
            */
            Quadrotor(const char *name);

            //! Update the state of the platform
            /*!
              \param state the new quadrotor state
              \return Whether the state was accepted
            */
            bool SetState(const hal_platform_quadrotor::State &sta);


        };
    }
}

#endif