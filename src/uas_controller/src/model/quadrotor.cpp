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

// HAL includes
#include <uas_hal/platform/UAV.h>

// HAL includes
#include "environment.pb.h"

// Components used to update the model dynamics
#include "dynamics/propulsion.h"
#include "dynamics/shear.h"
#include "dynamics/turbulence.h"
#include "dynamics/energy.h"

// Sensors provided by a stock quadrotor
#include "sensors/altimeter.h"
#include "sensors/compass.h"
#include "sensors/gnss.h"
#include "sensors/imu.h"
#include "sensors/ahrs.h"

namespace uas_controller
{

  // Save some pain
  typedef const boost::shared_ptr<const uas_controller::msgs::Environment> EnvironmentPtr;

  class Quadrotor : public uas_hal::UAV, public gazebo::ModelPlugin
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
    gazebo::transport::SubscriberPtr  subPtr;

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
    AHRS          ahrs;         // Simulates an AHRS system (orientation)

    // CALLBACKS ///////////////////////////////////////////////////////////////////////////////

    // When new control arrives from the HAL, immedaitely save it to the control class, so that
    // it can be used by Dynamics::Update() to change the platform propulsion.
    void ReceiveControl(
            const double &roll,
            const double &pitch,
            const double &yaw,
            const double &throttle)
    {
      // The energy module needs to know how much juice we are sending to the rotors
      energy.SetThrottle(throttle);

      // Pass the control to the propulsion engine
      propulsion.SetControl(
        roll, pitch, yaw, throttle,   // From the HAL
        energy.GetVoltage()           // From the energy model
      );

    }

    // Periodically the simulation produces a environment message, which contains a global UTC time
    // as well as meteorological information and GPS ephemerides. This information is used to 
    // update all of the simulated sensors, so that they can produce a meaningful measurement.
    void ReceiveEnvironment(EnvironmentPtr &msg)
    {
      ROS_INFO("RECEIVED ENVIRONMENT");

      // Configure the wind shear
      shear.SetWind(
        msg->wind().speed(),      // Speed (meter per second at 6m)
        msg->wind().direction()   // Direction (degrees at 6m)
      );

      // Set the ground temperature and pressure
      altimeter.SetMeteorological(
        msg->temperature(),       // Environment dry temp (kelvin)
        msg->pressure(),          // Environment pressure (hPa)
        msg->humidity()           // Environment rel humidity (%)
      );

      // Set the temperature of the IMU
      imu.SetMeteorological(
        msg->temperature(),       // Environment dry temp (kelvin)
        msg->gravity().x(),         // Grav field strength X
        msg->gravity().y(),         // Grav field strength Y
        msg->gravity().z()          // Grav field strength Z
      );

      // Set the temperature of the compass
      compass.SetMeteorological(
        msg->temperature(),       // Environment dry temp (kelvin)
        msg->magnetic().x(),        // Mag field strength X
        msg->magnetic().y(),        // Mag field strength Y
        msg->magnetic().z()         // Mag field strength Z
      );
      
      // Gte a navigation solution from the message
      gnss.SetNavigationSolution(msg);
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
      // Extract the state
      gazebo::math::Vector3 pos = modPtr->GetLink("body")->GetWorldPose().pos;
      gazebo::math::Vector3 rot = modPtr->GetLink("body")->GetWorldPose().rot.GetAsEuler();
      gazebo::math::Vector3 vel = modPtr->GetLink("body")->GetRelativeLinearVel();
      gazebo::math::Vector3 ang = modPtr->GetLink("body")->GetRelativeAngularVel();
      
      // Push the state
      UAV::SetState(
        pos.x,  pos.y,  pos.z,      // Position
        rot.x,  rot.y,  rot.z,      // Orientation
        vel.x,  vel.y,  vel.z,      // Velocity
        ang.x,  ang.y,  ang.z,      // Angular velocity
        propulsion.GetThrust(),     // Current thrust force
        energy.GetVoltage()         // Battery voltage
      );
    }

  public: 

    // Constructor
    Quadrotor() : uas_hal::UAV("quadrotor"), tim(0.0) {}

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
      ahrs.Configure(root->GetElement("ahrs"),model);
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

      // Subscribe to messages about atmospheric conditions
      subPtr  = nodePtr->Subscribe("~/environment", &Quadrotor::ReceiveEnvironment, this);

      ROS_INFO("Subscribed to environment");

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
      ahrs.Reset();
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

