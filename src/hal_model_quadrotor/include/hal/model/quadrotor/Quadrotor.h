#ifndef HAL_MODEL_QUADROTOR_H
#define HAL_MODEL_QUADROTOR_H

// System libraries
#include <string>
#include <map>

// Basic ROS stuff
#include <hal/model/Model.h>

// Quadrotor-specific controllers
#include <hal/model/quadrotor/controller/Emergency.h>
#include <hal/model/quadrotor/controller/Hover.h>
#include <hal/model/quadrotor/controller/Idle.h>
#include <hal/model/quadrotor/controller/Land.h>
#include <hal/model/quadrotor/controller/Takeoff.h>
#include <hal/model/quadrotor/controller/AnglesHeight.h>
#include <hal/model/quadrotor/controller/Velocity.h>
#include <hal/model/quadrotor/controller/VelocityHeight.h>
#include <hal/model/quadrotor/controller/Waypoint.h>

// State and control messages
#include <hal_model_quadrotor/State.h>
#include <hal_model_quadrotor/Control.h>

namespace hal
{
    namespace model
    {

        class Quadrotor : public hal::model::Model
        {       

        private:

            /// Current state of the quadrotor
            hal_model_quadrotor::State state;

            /// Current control vector
            hal_model_quadrotor::Control control;

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

            // Called by HAL when initialised
            void OnLoad();

        protected:

            //! Accept a control message from the HAL
            /*!
              \param ctl
            */
            virtual void ReceiveControl(const hal_model_quadrotor::Control &ctl) = 0;

        public:

            //! Create a new Quadrotor object
            /*!
              \param node ROS node tow hich the HAL will bind
            */
            Quadrotor();

            //! Update the state of the platform
            /*!
              \param state the new quadrotor state
              \return Whether the state was accepted
            */
            bool SetState(const hal_model_quadrotor::State &sta);

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