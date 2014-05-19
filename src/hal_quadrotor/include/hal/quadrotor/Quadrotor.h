#ifndef HAL_MODEL_QUADROTOR_H
#define HAL_MODEL_QUADROTOR_H

// System libraries
#include <string>
#include <map>

// Basic ROS stuff
#include <hal/HAL.h>

// Provides basic navigation and actuation functionality
#include <hal/quadrotor/Navigation.h>
#include <hal/quadrotor/Actuation.h>

// Services
#include <hal_model_quadrotor/SetTruth.h>
#include <hal_model_quadrotor/SetEstimate.h>
#include <hal_model_quadrotor/SetControl.h>
#include <hal_model_quadrotor/GetTruth.h>
#include <hal_model_quadrotor/GetEstimate.h>
#include <hal_model_quadrotor/GetControl.h>

namespace hal
{
    namespace quadrotor
    {
        class Quadrotor : public hal::HAL
        {       

        private:

            /// Current state of the quadrotor
            hal_model_quadrotor::State      estimate, truth;

            /// Current control vector
            hal_model_quadrotor::Control    control;

            /// Last time at which the update clock was called
            double tick;

            /// Publishers
            ros::Publisher      pubControl;          /*!< Control publisher      */
            ros::Publisher      pubEstimate;         /*!< State publisher        */
            ros::Publisher      pubTruth;            /*!< State publisher        */
            
            /// Timers
            ros::ServiceServer  srvSetTruth;         /*!< Update loop timer       */
            ros::ServiceServer  srvSetEstimate;      /*!< Update loop timer       */
            ros::ServiceServer  srvSetControl;       /*!< State broadcast timer   */
            ros::ServiceServer  srvGetTruth;         /*!< Update loop timer       */
            ros::ServiceServer  srvGetEstimate;      /*!< Update loop timer       */
            ros::ServiceServer  srvGetControl;       /*!< State broadcast timer   */

            /// Timers
            ros::Timer          timerUpdate;          /*!< Update loop timer       */
            ros::Timer          timerTruth;           /*!< State broadcast timer   */
            ros::Timer          timerEstimate;        /*!< State broadcast timer   */
            ros::Timer          timerControl;         /*!< Control broadcast timer */

            /// Converts HL-control instructions to LL-control commands
            Actuation           actuation;

            /// Converts sensor measurements to a state estimate
            Navigation          navigation;

            // CORE CONTROLLER FUNCRIONS ///////////////////////////////////////

            // Called by HAL when ROS is ready!            
            void OnInit();

            //! Timer callback for internal update loop
            /*!
              \param event Timer event
            */
            void Update(const ros::TimerEvent& event);

            // TOPIC CALLBACKS //////////////////////////////////////////////////

            //! Timer callback for state broadcast loop
            /*!
              \param event Timer event
            */
            void BroadcastTruth(const ros::TimerEvent& event);

            //! Timer callback for state broadcast loop
            /*!
              \param event Timer event
            */
            void BroadcastEstimate(const ros::TimerEvent& event);

            //! Timer callback for control broadcast loop
            /*!
              \param event Timer event
            */
            void BroadcastControl(const ros::TimerEvent& event);

            // SERVICE CALLBACKS ////////////////////////////////////////////////

            //! Service callback for getting the state
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvGetTruth(
                hal_model_quadrotor::GetTruth::Request  &req, 
                hal_model_quadrotor::GetTruth::Response &res
            );

            //! Service callback for setting the state
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvSetTruth(
                hal_model_quadrotor::SetTruth::Request  &req, 
                hal_model_quadrotor::SetTruth::Response &res
            );

            //! Service callback for getting the state
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvGetEstimate(
                hal_model_quadrotor::GetEstimate::Request  &req, 
                hal_model_quadrotor::GetEstimate::Response &res
            );

            //! Service callback for setting the state
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvSetEstimate(
                hal_model_quadrotor::SetEstimate::Request  &req, 
                hal_model_quadrotor::SetEstimate::Response &res
            );

            //! Service callback for getting the control
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvGetControl(
                hal_model_quadrotor::GetControl::Request  &req, 
                hal_model_quadrotor::GetControl::Response &res
            );

            //! Service callback for setting the control
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvSetControl(
                hal_model_quadrotor::SetControl::Request  &req, 
                hal_model_quadrotor::SetControl::Response &res
            );

        protected:

            //! Get the true quadrotor state
            /*!
              \param state the state of the quadrotor
            */
            virtual void GetTruth(hal_model_quadrotor::State &state) = 0;

            //! Get the estimated quadrotor state
            /*!
              \param state the state of the quadrotor
            */
            virtual void GetEstimate(hal_model_quadrotor::State &state) = 0;

            //! Set the true quadrotor state
            /*!
              \param state the state of the quadrotor
            */
            virtual void SetTruth(const hal_model_quadrotor::State &state) = 0;

            //! Set the quadrotor control
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