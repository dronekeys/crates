#ifndef HAL_MODEL_QUADROTOR_H
#define HAL_MODEL_QUADROTOR_H

// System libraries
#include <string>
#include <map>

// Basic ROS stuff
#include <hal/HAL.h>

// Controller stuff
#include <hal/model/FlightLogic.h>

// Services
#include <hal_model_quadrotor/SetState.h>
#include <hal_model_quadrotor/SetControl.h>

namespace hal
{
    namespace model
    {

        class Quadrotor : public hal::HAL
        {       

        private:

            /// Current state of the quadrotor
            hal_model_quadrotor::State state;

            /// Current control vector
            hal_model_quadrotor::Control control;

            /// Last time at which the update clock was called
            double tick;

            /// Publishers
            ros::Publisher pubControl;              /*!< Control publisher      */
            ros::Publisher pubState;                /*!< State publisher         */
            
            /// Timers
            ros::ServiceServer srvSetState;         /*!< Update loop timer       */
            ros::ServiceServer srvSetControl;       /*!< State broadcast timer   */

            /// Timers
            ros::Timer timerUpdate;                 /*!< Update loop timer       */
            ros::Timer timerState;                  /*!< State broadcast timer   */
            ros::Timer timerControl;                /*!< Control broadcast timer */

            /// Manages all the controllers
            FlightLogic flightLogic;

            // Called by HAL when ROS is ready!            
            void OnInit();

            //! Timer callback for internal update loop
            /*!
              \param event Timer event
            */
            void Update(const ros::TimerEvent& event);

            // TOPIC CALLBACKS //////////////////////////////////////////////////

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

            // SERVICE CALLBACKS ////////////////////////////////////////////////

            //! Service callback for setting the state
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvState(
                hal_model_quadrotor::SetState::Request  &req, 
                hal_model_quadrotor::SetState::Response &res
            );

            //! Service callback for setting the control
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvControl(
                hal_model_quadrotor::SetControl::Request  &req, 
                hal_model_quadrotor::SetControl::Response &res
            );

        protected:

            //! Get the state of the quadrotor from the FCS
            /*!
              \param state the state of the quadrotor
            */
            virtual void GetState(hal_model_quadrotor::State &state) = 0;

            //! Accept a state update message from the HAL
            /*!
              \param state the state of the quadrotor
            */
            virtual void SetState(const hal_model_quadrotor::State &state) = 0;

            //! Accept a control message from the HAL
            /*!
              \param control the control to apply
            */
            virtual void SetControl(const hal_model_quadrotor::Control &control) = 0;

        public:

            //! Create a new Quadrotor object
            /*!
              \param node ROS node tow hich the HAL will bind
            */
            Quadrotor();
        };
    }
}

#endif