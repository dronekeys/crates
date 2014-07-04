#ifndef HAL_QUADROTOR_CONTROL_H
#define HAL_QUADROTOR_CONTROL_H

// Project includes
#include <hal_quadrotor/control/Emergency.h>
#include <hal_quadrotor/control/Hover.h>
#include <hal_quadrotor/control/Idle.h>
#include <hal_quadrotor/control/Land.h>
#include <hal_quadrotor/control/Takeoff.h>
#include <hal_quadrotor/control/AnglesHeight.h>
#include <hal_quadrotor/control/Velocity.h>
#include <hal_quadrotor/control/VelocityHeight.h>
#include <hal_quadrotor/control/Waypoint.h>

// ROS
#include <ros/ros.h>

namespace hal
{
  namespace quadrotor
  {
    // Define the gaussian types
    typedef enum
    {
    	CONTROLLER_ANGLESHEIGHT,
    	CONTROLLER_EMERGENCY,
    	CONTROLLER_HOVER,
    	CONTROLLER_IDLE,
      CONTROLLER_LAND,
      CONTROLLER_TAKEOFF,
      CONTROLLER_VELOCITYHEIGHT,
      CONTROLLER_VELOCITY,
      CONTROLLER_WAYPOINT,
      CONTROLLER_DISABLED
    }
    ControllerType;

    // An abstract class for modelling noise
    class Actuation
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
      void Init(ros::NodeHandle nh, ControllerType controller = CONTROLLER_IDLE);

      //! Switch to a new controller
      /*!
          \param controller the new controller
      */
      void Switch(ControllerType controller);

      //! Obtain current controller
      /*!
          \return current Controller or NULL
      */
      Controller *GetControler(void);
      //! Obtain control from state and timestep
      /*!
          \param state the current platform state
          \param dt the discrete time step
          \param control the output control from the controller
          \return if the state could be updated
      */
      bool GetControl(const hal_quadrotor::State &state,
          double dt, hal_quadrotor::Control &control);

      // CONTROLLER CALLBACKS //////////////////////////////////////

      //! Callback for new AnglesHeight request
      /*!
        \param req service request
        \param res service response
        \return whether the service was process successfully
      */
      bool RcvAnglesHeight(
          hal_quadrotor::AnglesHeight::Request  &req,
          hal_quadrotor::AnglesHeight::Response &res
      );

      //! Callback for new Emergency request
      /*!
        \param req service request
        \param res service response
        \return whether the packet was process successfully
      */
      bool RcvEmergency(
          hal_quadrotor::Emergency::Request  &req,
          hal_quadrotor::Emergency::Response &res
      );

      //! Callback for new Hover request
      /*!
        \param req service request
        \param res service response
        \return whether the packet was process successfully
      */
      bool RcvHover(
          hal_quadrotor::Hover::Request  &req,
          hal_quadrotor::Hover::Response &res
      );

      //! Callback for new Land request
      /*!
        \param req service request
        \param res service response
        \return whether the packet was process successfully
      */
      bool RcvLand(
          hal_quadrotor::Land::Request  &req,
          hal_quadrotor::Land::Response &res
      );

      //! Callback for new Takeoff request
      /*!
        \param req service request
        \param res service response
        \return whether the packet was process successfully
      */
      bool RcvTakeoff(
          hal_quadrotor::Takeoff::Request  &req,
          hal_quadrotor::Takeoff::Response &res
      );

      //! Callback for new Velocity request
      /*!
        \param req service request
        \param res service response
        \return whether the packet was process successfully
      */
      bool RcvVelocity(
          hal_quadrotor::Velocity::Request  &req,
          hal_quadrotor::Velocity::Response &res
      );

      //! Callback for new VelocityHeight request
      /*!
        \param req service request
        \param res service response
        \return whether the packet was process successfully
      */
      bool RcvVelocityHeight(
          hal_quadrotor::VelocityHeight::Request  &req,
          hal_quadrotor::VelocityHeight::Response &res
      );

      //! Callback for new Waypoint request
      /*!
        \param req service request
        \param res service response
        \return whether the packet was process successfully
      */
      bool RcvWaypoint(
          hal_quadrotor::Waypoint::Request  &req,
          hal_quadrotor::Waypoint::Response &res
      );

    };
  }
}

#endif
