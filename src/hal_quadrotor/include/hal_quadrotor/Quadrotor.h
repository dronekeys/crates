#ifndef HAL_QUADROTOR_H
#define HAL_QUADROTOR_H

// System libraries
#include <string>
#include <map>

// Basic ROS stuff
#include <hal/HAL.h>

// Core quadrotor functionality
#include <hal_quadrotor/Navigation.h>
#include <hal_quadrotor/Actuation.h>

// ROS Services
#include <hal_quadrotor/SetTruth.h>
#include <hal_quadrotor/SetEstimate.h>
#include <hal_quadrotor/SetControl.h>
#include <hal_quadrotor/GetTruth.h>
#include <hal_quadrotor/GetEstimate.h>
#include <hal_quadrotor/GetControl.h>

namespace hal
{
    namespace quadrotor
    {
        class Quadrotor : public hal::HAL
        {       

        private:

            /// Whether the motors are currently armed
            bool armed;

            /// Current state of the quadrotor
            hal_quadrotor::State estimate, truth;

            /// Current control vector
            hal_quadrotor::Control control;

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

            /// RECEIVE CALLS FOR ALL SENSOR DATA ////////////////////////////

            //! Service callback for getting the state
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvGetTruth(
                hal_quadrotor::GetTruth::Request  &req, 
                hal_quadrotor::GetTruth::Response &res
            );

            //! Service callback for setting the state
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvSetTruth(
                hal_quadrotor::SetTruth::Request  &req, 
                hal_quadrotor::SetTruth::Response &res
            );

            //! Service callback for getting the state
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvGetEstimate(
                hal_quadrotor::GetEstimate::Request  &req, 
                hal_quadrotor::GetEstimate::Response &res
            );

            //! Service callback for setting the state
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvSetEstimate(
                hal_quadrotor::SetEstimate::Request  &req, 
                hal_quadrotor::SetEstimate::Response &res
            );

            //! Service callback for getting the control
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvGetControl(
                hal_quadrotor::GetControl::Request  &req, 
                hal_quadrotor::GetControl::Response &res
            );

            //! Service callback for setting the control
            /*!
              \param req service request
              \param res service response
              \return whether the packet was process successfully
            */
            bool RcvSetControl(
                hal_quadrotor::SetControl::Request  &req, 
                hal_quadrotor::SetControl::Response &res
            );

        protected:

            /// RECEIVE CALLS FOR ALL SENSOR DATA ////////////////////////////

            //! Called when new altimeter data arrives
            /*!
            \param msg the sensor data
            */
            void Feed(const hal_sensor_altimeter::Data& msg);

            //! Called when new compass data arrives
            /*!
            \param msg the sensor data
            */
            void Feed(const hal_sensor_compass::Data& msg);

            //! Called when new IMU data arrives
            /*!
            \param msg the sensor data
            */
            void Feed(const hal_sensor_imu::Data& msg);

            //! Called when new GNSS data arrives
            /*!
            \param msg the sensor data
            */
            void Feed(const hal_sensor_gnss::Data& msg);

            //! Called when new orientation data arrives
            /*!
            \param msg the sensor data
            */
            void Feed(const hal_sensor_orientation::Data& msg);

            //! Called to set the quadrotor state estimate manually
            /*!
            \param msg the quadrotor state
            */
            void Feed(const hal_quadrotor::State &state);

            /// INERACTION WITH THE FLIGHT CONTROL SYSTEM ///////////////////////

            //! Arm or disarm the motors
            /*!
              \param arm whether the motors should be armed
            */
            virtual void ArmMotors(bool arm) = 0;

            //! Get the true quadrotor state
            /*!
              \param state the state of the quadrotor
            */
            virtual void GetTruth(hal_quadrotor::State &state) = 0;

            //! Set the true quadrotor state
            /*!
              \param state the state of the quadrotor
            */
            virtual void SetTruth(const hal_quadrotor::State &state) = 0;

            //! Set the quadrotor control
            /*!
              \param control the control to apply
              \return time at which the control was set 
            */
            virtual double SetControl(const hal_quadrotor::Control &control) = 0;

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