#ifndef HAL_MODEL_QUADROTOR_H
#define HAL_MODEL_QUADROTOR_H

// System libraries
#include <string>
#include <map>

// Basic ROS stuff
#include <hal/HAL.h>

// Quadrotor-specific controllers
#include <hal_model_quadrotor/Emergency.h>
#include <hal_model_quadrotor/Hover.h>
#include <hal_model_quadrotor/Idle.h>
#include <hal_model_quadrotor/Land.h>
#include <hal_model_quadrotor/Takeoff.h>
#include <hal_model_quadrotor/AnglesHeight.h>
#include <hal_model_quadrotor/Velocity.h>
#include <hal_model_quadrotor/VelocityHeight.h>
#include <hal_model_quadrotor/Waypoint.h>

// State and control messages
#include <hal_model_quadrotor/State.h>
#include <hal_model_quadrotor/Control.h>

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
            ros::Publisher      pubControl;         /*!< Control publisher      */
            ros::Publisher      pubState;           /*!< State publisher         */

            // Services
            ros::ServiceServer  srvAnglesHeight;
            ros::ServiceServer  srvEmergency;
            ros::ServiceServer  srvHover;
            ros::ServiceServer  srvIdle;
            ros::ServiceServer  srvLand;
            ros::ServiceServer  srvTakeoff;
            ros::ServiceServer  srvVelocity;
            ros::ServiceServer  srvVelocityHeight;
            ros::ServiceServer  srvWaypoint;

            /// Timers
            ros::Timer timerUpdate;                 /*!< Update loop timer       */
            ros::Timer timerState;                  /*!< State broadcast timer   */
            ros::Timer timerControl;                /*!< Control broadcast timer */

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

            //! Timer callback for state broadcast loop
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvAnglesHeight(
                hal_model_quadrotor::AnglesHeight::Request  &req, 
                hal_model_quadrotor::AnglesHeight::Response &res
            );

            //! Timer callback for state broadcast loop
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvEmergency(
                hal_model_quadrotor::Emergency::Request  &req, 
                hal_model_quadrotor::Emergency::Response &res
            );
            //! Timer callback for state broadcast loop
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvHover(
                hal_model_quadrotor::Hover::Request  &req, 
                hal_model_quadrotor::Hover::Response &res
            );
                        
            //! Timer callback for state broadcast loop
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvIdle(
                hal_model_quadrotor::Idle::Request  &req, 
                hal_model_quadrotor::Idle::Response &res
            );
                        
            //! Timer callback for state broadcast loop
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvLand(
                hal_model_quadrotor::Land::Request  &req, 
                hal_model_quadrotor::Land::Response &res
            );
            
            //! Timer callback for state broadcast loop
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvTakeoff(
                hal_model_quadrotor::Takeoff::Request  &req, 
                hal_model_quadrotor::Takeoff::Response &res
            );
                        //! Timer callback for state broadcast loop
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvVelocity(
                hal_model_quadrotor::Velocity::Request  &req, 
                hal_model_quadrotor::Velocity::Response &res
            );
                        //! Timer callback for state broadcast loop
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvVelocityHeight(
                hal_model_quadrotor::VelocityHeight::Request  &req, 
                hal_model_quadrotor::VelocityHeight::Response &res
            );

            //! Timer callback for state broadcast loop
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvWaypoint(
                hal_model_quadrotor::Waypoint::Request  &req, 
                hal_model_quadrotor::Waypoint::Response &res
            );

        protected:

            //! Accept a control message from the HAL
            /*!
              \param ctl
            */
            virtual void Control(const hal_model_quadrotor::Control &ctl) = 0;

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