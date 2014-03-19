/* This gazebo model plugin implements a second order model for quadrotor dynamics */ 

//  Boost includes
#include <boost/bind.hpp>

// ROS includes
#include <ros/ros.h>

// Gazebo includes
#include <gazebo/physics/Model.hh>
#include <gazebo/gazebo.hh>

// HAL includes
#include <uas_hal/platform/UAV.h>

// Components used int the model
#include "components/energy.h"
#include "components/dynamics.h"
#include "components/control.h"
#include "components/shear.h"
#include "components/turbulence.h"

namespace uas_controller
{
  class Quadrotor : public uas_hal::UAV, public gazebo::ModelPlugin
  {

  private:

    // Pointer to the current model
    gazebo::physics::ModelPtr         modPtr;

    // Pointer to the update event connection
    gazebo::event::ConnectionPtr      conPtr;

    // Required for receiving atmospheric updates
    gazebo::transport::NodePtr        nodePtr;
    gazebo::transport::SubscriberPtr  subPtr;

    // Timers for state and information broadcasting
    ros::Timer                        timState;
    ros::Timer                        timInformation;

    // COMPONENTS
    Shear         shear;      // Models wind shear, based on global field
    Turbulence    turbulence; // Models wind gusts
    Energy        energy;     // Models energy consumption
    Dynamics      dynamics;   // Models UAV dynamics
    Control       control;    // Converts SI control <-> RX values

    // When new control arrives from the HAL, save it...
    void HalProcessControl(const uas_hal::Control & _ctl)
    {
      // If we are not in a critical energy mode, then set control
      if (!energy.IsCriticallyLow())
        control.SetControlSI(_ctl.pitch,_ctl.roll,_ctl.thrust,_ctl.yaw);
    }

    // Receive the global wind speed and direction
    void ReceiveAtmosphere(AtmospherePtr &msg)
    {
      // Configure the wind shear accoridngly
      shear.SetGlobalField
      (
        msg->wind_speed(),
        msg->wind_direction()
      )
    }

    // Broadcast the current state
    void BroadcastState(const ros::TimerEvent& event)
    {
      // Get the current state of the vehicle
      state = dynamics.SetState(modPtr);

      // Immediately publish the state
      HalBroadcastState(
        state.pos.x,  state.pos.y,  state.pos.z,  // Position
        state.rot.x,  state.rot.y,  state.rot.z,  // Orientation
        state.vel.x,  state.vel.y,  state.vel.z,  // Velocity
        state.ang.x,  state.ang.y,  state.ang.z,  // Angular velocity
        state.thrust, state.battery               // Thrust force
      );
    }

    // Broadcast the current state
    void BroadcastInformation(const ros::TimerEvent& event)
    {
      // Immediately publish the state
      HalBroadcastInformation(
        modPtr->GetName().c_str()   // UAV name
        "simulated",                // Decription
        tim                         // Uptime (respects siulation)
      );
    }

    // Second-order Runge-Kutta dynamics
    void Update(const gazebo::common::UpdateInfo & _info)
    {
      // Time over which dynamics must be updated (needed for thrust update)
      dt = _info.simTime.Double() - tim;

      // If simulation is paused, dont waste CPU cycles calculating a physics update...
      if (dt > 0) 
      {
        // Set the state of the platform directly from the simulation
        dynamics.SetState(linkPtr);

        ///////////////////
        // ENERGY UPDATE //
        ///////////////////

        // Update the energy consumption based on the current thrust force
        energy.Update(control.GetThrust(),dt);
        
        // If we have hit a critically low energy value, 
        if (energy.IsCriticallyLow())
        {
          // Update the control to level platform, then gently reduce thrust
          control.SetControlSI(
            0.0,
            0.0,
            control.GetThrustSI() - energy.GetThrustReduction(),
            0.0
          );
        }
      
        // Update the control to include the remaining voltage 
        control.SetVoltage(energy.GetVoltage());

        /////////////////
        // WIND UPDATE //
        /////////////////

        // Update shear
        shear.Update(dynamics.GetAltitude());

        // Update turbulence
        turbulence.Update(dynamics.GetAltitude(),dynamics.GetAirspeed(),dt);

        ///////////////////
        // DYNAMIC MODEL //
        ///////////////////

        // First, set the state based on the simulated link
        dynamics.Update(
          control.GetControlRC(),                             // control
          shear->GetVelocity() + turbulence->GetVelocity(),   // wind
          dt                                                  // discrete time
        );

        ///////////////////////
        // FORCE APPLICATION //
        ///////////////////////

        // Add a force to the link (noise is added by dynamic model)
        linkPtr->AddForce(dynamics.GetForce());
        
        // Add a torque to the link (noise is added by dynamic model)
        linkPtr->AddTorque(dynamics.GetTorque());

      }

      // Update timer
      tim = _info.simTime.Double();
    }

  public: 

    // On initial load
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) 
    {
      // Save pointer to the model
      modPtr = _model;

      ////////////////////////
      // Configure from SDF //
      ////////////////////////

      // Configure the energy model from the SDF
      energy.Configure(_sdf);

      // Configure the shear model from SDF
      shear.Configure(_sdf);

      // Configure the turbulence model from SDF
      turbulence.Configure(_sdf);
      
      // Configure the dynamic model from SDF
      dynamics.Configure(_sdf);

      // Configure the control model from SDF
      control.Configure(_sdf);

      /////////////////////////////////////
      // Initialise gazbeo communication //
      /////////////////////////////////////

      // Setup the gazebo node
      nodePtr = transport::NodePtr(new transport::Node());

      // Subscribe to messages about atmospheric conditions
      subPtr = nodePtr->Subscribe("~/atmosphere", &Quadrotor::ReceiveAtmosphere, this);

      // Set up callback for updating the model dynamics (at physics rate)
      conPtr = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&Quadrotor::Update, this, _1));

      //////////////////////////////////////////
      // Initialise HAL and ROS communication //
      //////////////////////////////////////////

      // Initialise the HAL to the model name
      HalInit(((std::string)"/hal/"+(std::string)modPtr->GetName()).c_str()); 

      // Call method periodically to broadcast state (respects simulation time)
      timState = node.createTimer(
          ros::Duration(1.0),                                             // duration
          boost::bind(&Quadrotor::BroadcastState, this, _1),           // callback
          false                                                           // oneshot?
      );

      // Call method periodically to broadcast information (respects simulation time)
      timInformation = node.createTimer(
          ros::Duration(1.0),                                             // duration
          boost::bind(&Quadrotor::BroadcastInformation, this, _1),     // callback
          false                                                           // oneshot?
      );
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Quadrotor)
}

