#ifndef HAL_MODEL_QUADROTOR_FLIGHTLOGIC_H
#define HAL_MODEL_QUADROTOR_FLIGHTLOGIC_H

// Project includes
#include <hal/model/controller/Emergency.h>
#include <hal/model/controller/Hover.h>
#include <hal/model/controller/Idle.h>
#include <hal/model/controller/Land.h>
#include <hal/model/controller/Takeoff.h>
#include <hal/model/controller/AnglesHeight.h>
#include <hal/model/controller/Velocity.h>
#include <hal/model/controller/VelocityHeight.h>
#include <hal/model/controller/Waypoint.h>

// ROS
#include <ros/ros.h>

namespace hal
{
  namespace model
  {
    // Define the gaussian types
    typedef enum 
    {
    	ANGLES_HEIGHT,        
    	EMERGENCY,            
    	HOVER,                
    	IDLE,                 
      LAND,                  
      TAKEOFF,
      VELOCITY_HEIGHT,
      VELOCITY,
      WAYPOINT,
      DISABLED
    } 
    ControllerType;

    // An abstract class for modelling noise
    class FlightLogic
    {     

    private:

      // Create services
      ros::ServiceServer srvAnglesHeight;
      ros::ServiceServer srvEmergency;
      ros::ServiceServer srvHover;
      ros::ServiceServer srvLand;
      ros::ServiceServer srvTakeoff;
      ros::ServiceServer srvVelocity;
      ros::ServiceServer srvVelocityHeight;
      ros::ServiceServer srvWaypoint;

      // One of each controller
      AnglesHeight    cAnglesHeight;
      Emergency       cEmergency;
      Hover           cHover;
      Idle            cIdle;
      Land            cLand;
      Takeoff         cTakeoff;
      Velocity        cVelocity;
      VelocityHeight  cVelocityHeight;
      Waypoint        cWaypoint;

      /// The current controller
      ControllerType  current;

    public:    

      //! Initialise the controller factory
      /*!
          \param nh the ROS node handle
          \param controller the default controller
      */
      void Init(ros::NodeHandle* nh, ControllerType controller);

      //! Switch to a new controller
      /*!
          \param controller the new controller
      */
      void Switch(ControllerType controller);

      //! Obtain control from state and timestep
      /*!
          \param state the current platform state
          \param dt the discrete time step
          \param control the output control from the controller
          \return if the state could be updated
      */
      bool Update(const hal_model_quadrotor::State &state, 
          double dt, hal_model_quadrotor::Control &control);

      // CONTROLLER CALLBACKS //////////////////////////////////////

      //! Callback for new AnglesHeight request
      /*!
        \param req service request
        \param res service response
        \return whether the service was process successfully
      */
      bool RcvAnglesHeight(
          hal_model_quadrotor::AnglesHeight::Request  &req, 
          hal_model_quadrotor::AnglesHeight::Response &res
      );

      //! Callback for new Emergency request
      /*!
        \param req service request
        \param res service response
        \return whether the packet was process successfully
      */
      bool RcvEmergency(
          hal_model_quadrotor::Emergency::Request  &req, 
          hal_model_quadrotor::Emergency::Response &res
      );
      
      //! Callback for new Hover request
      /*!
        \param req service request
        \param res service response
        \return whether the packet was process successfully
      */
      bool RcvHover(
          hal_model_quadrotor::Hover::Request  &req, 
          hal_model_quadrotor::Hover::Response &res
      );
                  
      //! Callback for new Land request
      /*!
        \param req service request
        \param res service response
        \return whether the packet was process successfully
      */
      bool RcvLand(
          hal_model_quadrotor::Land::Request  &req, 
          hal_model_quadrotor::Land::Response &res
      );

      //! Callback for new Takeoff request
      /*!
        \param req service request
        \param res service response
        \return whether the packet was process successfully
      */
      bool RcvTakeoff(
          hal_model_quadrotor::Takeoff::Request  &req, 
          hal_model_quadrotor::Takeoff::Response &res
      );
      
      //! Callback for new Velocity request
      /*!
        \param req service request
        \param res service response
        \return whether the packet was process successfully
      */
      bool RcvVelocity(
          hal_model_quadrotor::Velocity::Request  &req, 
          hal_model_quadrotor::Velocity::Response &res
      );

      //! Callback for new VelocityHeight request
      /*!
        \param req service request
        \param res service response
        \return whether the packet was process successfully
      */
      bool RcvVelocityHeight(
          hal_model_quadrotor::VelocityHeight::Request  &req, 
          hal_model_quadrotor::VelocityHeight::Response &res
      );

      //! Callback for new Waypoint request
      /*!
        \param req service request
        \param res service response
        \return whether the packet was process successfully
      */
      bool RcvWaypoint(
          hal_model_quadrotor::Waypoint::Request  &req, 
          hal_model_quadrotor::Waypoint::Response &res
      );

    };
  }
}

#endif