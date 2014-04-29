#ifndef HAL_PLATFORM_QUADROTOR_H
#define HAL_PLATFORM_QUADROTOR_H

// System libraries
#include <string>
#include <map>

// Basic ROS stuff
#include <hal/platform/Platform.h>

// Quadrotor-specific controllers
#include <hal/platform/quadrotor/controller/Emergency.h>
#include <hal/platform/quadrotor/controller/Hover.h>
#include <hal/platform/quadrotor/controller/Idle.h>
#include <hal/platform/quadrotor/controller/Land.h>
#include <hal/platform/quadrotor/controller/Takeoff.h>
#include <hal/platform/quadrotor/controller/AnglesHeight.h>
#include <hal/platform/quadrotor/controller/Velocity.h>
#include <hal/platform/quadrotor/controller/VelocityHeight.h>
#include <hal/platform/quadrotor/controller/Waypoint.h>

// State and control messages
#include <hal_platform_quadrotor/State.h>
#include <hal_platform_quadrotor/Control.h>

namespace hal
{
    namespace platform
    {

        class Quadrotor : 
            public hal::platform::Platform,
            public hal::controller::Emergency,
            public hal::controller::Hover,
            public hal::controller::Idle,
            public hal::controller::Land,
            public hal::controller::Takeoff,
            public hal::controller::AnglesHeight,
            public hal::controller::Velocity,
            public hal::controller::VelocityHeight,
            public hal::controller::Waypoint
        {       

        private:

            /// Current state of the quadrotor
            hal_platform_quadrotor::State state;

            /// Current control vector
            hal_platform_quadrotor::Control control;

            /// Last time at which the update clock was called
            double tick;

            /// Publishers
            ros::Publisher pubControl;  /*!< Control publisher      */
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
            virtual void ReceiveControl(const hal_platform_quadrotor::Control &ctl) = 0;

        public:

            //! Create a new Quadrotor object
            /*!
              \param node ROS node tow hich the HAL will bind
            */
            Quadrotor(ros::NodeHandle& node);

            //! Update the state of the platform
            /*!
              \param state the new quadrotor state
              \return Whether the state was accepted
            */
            bool SetState(const hal_platform_quadrotor::State &sta);

            //! Switch flight controller
            /*!
              \param next the next controller type
              \return whether the transition was allowed
            */
            bool SwitchController(const std::string& next);

        };
    }
}

#endif