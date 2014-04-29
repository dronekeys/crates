/* This gazebo model plugin implements a second order model for quadrotor dynamics */ 

//  Boost includes
#include <boost/bind.hpp>

// ROS includes
#include <ros/ros.h>

// Gazebo includes
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// For animation
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Animation.hh>
#include <gazebo/common/KeyFrame.hh>

// Include the quadrotor HAL
#include <hal_quadrotor/HAL.h>

// Components used to update the model dynamics
#include "model/Propulsion.h"
#include "model/Shear.h"
#include "model/Turbulence.h"
#include "model/Energy.h"
#include "model/Altimeter.h"
#include "model/Compass.h"
#include "model/GNSS.h"
#include "model/IMU.h"

namespace controller
{

  class Quadrotor : public hal_quadrotor::HAL, public gazebo::ModelPlugin
  {

  private:

    // BASIC SIMULATION VARIABLES ////////////////////////////////////////////////////////////

    // Pointer to the model object
    gazebo::physics::ModelPtr         modPtr;

    // Pointer to the update event connection
    gazebo::event::ConnectionPtr      conBegPtr;
    gazebo::event::ConnectionPtr      conEndPtr;

    // Required for receiving atmospheric updates
    gazebo::transport::NodePtr        nodePtr;
    gazebo::transport::SubscriberPtr  subPtrEnv, subPtrMet, subPtrSat;

    // Stores the current simulation time (seconds since start)
    double                            tim;

    // DYNAMICS CLASSES TO HELP WITH PLATFORM DYNAMICS ////////////////////////////////////////

    Shear         shear;        // Calculates wind shear and applies force to platform
    Turbulence    turbulence;   // Calculates wind turbulence and applies force to platform
    Propulsion    propulsion;   // Uses control to select motor speeds, applying force to platform 
    Energy        energy;       // Simulates a battery

    // CLASSES TO HELP WITH SENSOR DATA PRODUCTION /////////////////////////////////////////////

    Altimeter     altimeter;    // Simulates barometric altitude sensing
    Compass       compass;      // Simulates a magnetic field strength sensor
    GNSS          gnss;         // Simulates a GNSS receiver using GPStk
    IMU           imu;          // Simulates an inertial measurement unit

    // CALLBACKS ///////////////////////////////////////////////////////////////////////////////

    // When new control arrives from the HAL, immedaitely save it to the control class, so that
    // it can be used by Propulsion::Update() to update the platform dynamics.
    void Receive(const hal_quadrotor::Control &ctl)
    {
      // The energy module needs to know how much juice we are sending to the rotors
      energy.SetThrottle(ctl.throttle);

      // Pass the control to the propulsion engine
      propulsion.SetControl(
        ctl.roll, ctl.pitch, ctl.yaw, ctl.throttle,   // From the HAL
        energy.GetVoltage()                           // From the energy model
      );
    }

    // This is the grand simulation abritrator. In a nutshell, it is called on every physics time
    // tick, which should be 20ms (50Hz). The Update() method is called on each of the dynamics 
    // classes, which apply a force and torque to the platform. The actual physics is then handled
    // by the gazebo physics engine, in order to be interoperable with other robots.
    void PrePhysics(const gazebo::common::UpdateInfo &_info)
    {
      // Time over which dynamics must be updated (needed for thrust update)
      double dt = _info.simTime.Double() - tim;

      // If simulation is paused, dont waste CPU cycles calculating a physics update...
      if (dt > 0) 
      {
        // Set the state of the platform directly from the simulation
        propulsion.Update(dt);

        // Add any shear forces
        shear.Update(dt);

        // Add any turbulent forces
        turbulence.Update(dt);

        // Update energy
        energy.Update(dt);
      }

      // Update timer
      tim = _info.simTime.Double();
    }

    // Once the simulated world has been updated, we're going to want to update our noisy perception
    // of it. We will post the result up to the UAV hardware abstraction layer, as it will be needed
    // to decide the next control action to send back to the simulated device.
    void PostPhysics()
    {
      /*
      // Update the HAL state
      hal_quadrotor::Topic<hal_quadrotor::State>::Set();

      // Position and velocity from GNSS (in gazebo local reference frame)
      gazebo::math::Vector3 pos = gnss.GetPosition();
      gazebo::math::Vector3 vel = gnss.GetVelocity();

      // Orientation and drift-corrected angular velocity from AHRS
      gazebo::math::Vector3 rot = ahrs.GetOrientation();
      gazebo::math::Vector3 ang = ahrs.GetAngularVelocity();

      // Altitude from barometric altimeter
      double alt = altimeter.GetAltitude();
      
      // Current thrust force
      double thrust = propulsion.GetThrust();

      // Current battery voltage
      double voltage = energy.GetVoltage();

      // Extract the state
      // gazebo::math::Vector3 pos = modPtr->GetLink("body")->GetWorldPose().pos;
      // gazebo::math::Vector3 rot = modPtr->GetLink("body")->GetWorldPose().rot.GetAsEuler();
      // gazebo::math::Vector3 vel = modPtr->GetLink("body")->GetRelativeLinearVel();
      // gazebo::math::Vector3 ang = modPtr->GetLink("body")->GetRelativeAngularVel();
      // alt = pos.z;

      // Push the state
      Quadrotor::SetState(
        pos.x,  pos.y,  alt,        // Position (GNSS, GNSS, altimeter)
        rot.x,  rot.y,  rot.z,      // Orientation (AHRS, AHRS, AHRS)
        vel.x,  vel.y,  vel.z,      // Velocity (GNSS, GNSS, GNSS)
        ang.x,  ang.y,  ang.z,      // Angular velocity (IMU)
        thrust,                     // Current thrust force (dynamics)
        voltage                     // Battery voltage (energy)
      );
      */
    }

  public: 

    // Constructor
    Quadrotor() : hal_quadrotor::HAL("quadrotor"), tim(0.0) {}

    // On initial load
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr root) 
    {
      // Save the model
      modPtr = model;

      // CONFIGURE THE HELPER CLASSES ////////////////////////////////

      // Dynamics and control
      turbulence.Configure(root->GetElement("turbulence"),model);
      propulsion.Configure(root->GetElement("propulsion"),model);
      shear.Configure(root->GetElement("shear"),model);
      
      // Sensors
      altimeter.Configure(root->GetElement("altimeter"),model);
      compass.Configure(root->GetElement("compass"),model);
      energy.Configure(root->GetElement("energy"),model);
      gnss.Configure(root->GetElement("gnss"),model);
      imu.Configure(root->GetElement("imu"),model);

      // LISTEN FOR MESSAGES CONTAINING METEOROLOGICAL INFO //////////

      // Setup the gazebo node
      nodePtr = gazebo::transport::NodePtr(new gazebo::transport::Node());

      // Initialize the node with the world name
      nodePtr->Init(modPtr->GetWorld()->GetName());

      // LISTEN FOR PRE AND POST PHYSICS SIM UPDATES /////////////////

      // Pre physics - update quadrotor dynamics and wind
      conBegPtr = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&Quadrotor::PrePhysics, this, _1));

      // Post physics - broadcast current state to HAL
      conEndPtr = gazebo::event::Events::ConnectWorldUpdateEnd(
        boost::bind(&Quadrotor::PostPhysics, this));

      /////////////////////////////////////////////////////////////////
    }

    // Return to the initial configuration
    void Reset()
    {
      // Set time back to zero
      tim = 0.0;

      // Reset all dynamics and control
      turbulence.Reset();
      propulsion.Reset();
      shear.Reset();
      
      // Reset all sensors
      altimeter.Reset();
      compass.Reset();
      energy.Reset();
      gnss.Reset();
      imu.Reset();
    } 
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Quadrotor)
}

