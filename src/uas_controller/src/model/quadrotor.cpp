/* This gazebo model plugin implements a second order model for quadrotor dynamics */ 

//  Boost includes
#include <boost/bind.hpp>

// ROS includes
#include <ros/ros.h>

// Gazebo includes
#include <gazebo/physics/Model.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Animation.hh>
#include <gazebo/common/KeyFrame.hh>
#include <gazebo/physics/Model.hh>

// HAL includes
#include <uas_hal/platform/UAV.h>

// HAL includes
#include "atmosphere.pb.h"

// Components used int the model
#include "components/energy.h"
#include "components/dynamics.h"
#include "components/control.h"
#include "components/shear.h"
#include "components/turbulence.h"

namespace uas_controller
{

  // Save some pain
  typedef const boost::shared_ptr<const uas_controller::msgs::Atmosphere> AtmospherePtr;

  class Quadrotor : public uas_hal::UAV, public gazebo::ModelPlugin
  {

  private:

    // Current time
    double tim;

    // Pointer to the current model
    gazebo::physics::ModelPtr         modPtr;
    gazebo::physics::LinkPtr          lnkPtr;

    // Pointer to the update event connection
    gazebo::event::ConnectionPtr      conPtr;

    // Required for receiving atmospheric updates
    gazebo::transport::NodePtr        nodePtr;
    gazebo::transport::SubscriberPtr  subPtr;

    // Timers for state and information broadcasting
    ros::Timer                        timState;
    ros::Timer                        timInformation;

    // Optimizaiton
    gazebo::math::Vector3             wind;

    // COMPONENTS
    Shear         shear;        // Models wind shear, based on global field
    Turbulence    turbulence;   // Models wind gusts
    Energy        energy;       // Models energy consumption
    Dynamics      dynamics;     // Models UAV dynamics
    Control       control;      // Converts SI control <-> RX values
  

    // When new control arrives from the HAL, save it...
    void ReceiveControl(
            const double &pitch,
            const double &roll,
            const double &throttle,
            const double &yaw)
    {
        control.pitch    = pitch;
        control.roll     = roll;
        control.throttle = throttle;
        control.yaw      = yaw;
    }

    // Receive the global wind speed and direction
    void ReceiveAtmosphere(AtmospherePtr &msg)
    {
      /*
      // Configure the wind shear accoridngly
      shear.SetWind(
        msg->wind_speed(),      // Spped
        msg->wind_direction()   // Direction
      );
      */
    }

    // Broadcast the current state
    void BroadcastState(const ros::TimerEvent& event)
    {
      /*
      pose = lnkPtr->GetWorldPose();

      // Immediately publish the state
      HalBroadcastState(
        state.pos.x,  state.pos.y,  state.pos.z,  // Position
        state.rot.x,  state.rot.y,  state.rot.z,  // Orientation
        state.vel.x,  state.vel.y,  state.vel.z,  // Velocity
        state.ang.x,  state.ang.y,  state.ang.z,  // Angular velocity
        state.thrust, state.battery               // Thrust force
      );

      */
    }

    // Broadcast the current state
    void BroadcastInformation(const ros::TimerEvent& event)
    {
      // Immediately publish the state
      PostInformation(
        "uav",              // UAV name
        "simulated",        // Decription
        tim                 // Uptime (respects siulation)
      );
    }

    // Second-order Runge-Kutta dynamics
    void Update(const gazebo::common::UpdateInfo &_info)
    {
      // Time over which dynamics must be updated (needed for thrust update)
      double dt = _info.simTime.Double() - tim;

      // If simulation is paused, dont waste CPU cycles calculating a physics update...
      if (dt > 0) 
      {
        ///////////////////
        // ENERGY UPDATE //
        ///////////////////

        // Update the energy consumption, and get the current voltage
        control.voltage = energy.Update(control.throttle, dt);

        if (tim < 5)
          control.pitch = 0.05;
        else
          control.pitch = 0.0;
        
        // Get navigation-frame wind
        wind = shear.Update(lnkPtr, dt) + turbulence.Update(lnkPtr, dt);

        // Set the state of the platform directly from the simulation
        dynamics.Update(
          lnkPtr,                                             // Model
          wind,                                               // Wind
          control.GetScaledPitch(),                           // Pitch
          control.GetScaledRoll(),                            // Roll
          control.GetScaledThrottle(),                        // Throttle
          control.GetScaledYaw(),                             // Yaw
          control.GetScaledVoltage(),                         // Yaw
          dt                                                  // Time
        );

      }

      // Update timer
      tim = _info.simTime.Double();
    }

  public: 

    Quadrotor() : uas_hal::UAV("quadrotor"), tim(0.0)
    {
      ROS_INFO("Loaded quadrotor plugin");
    }

    // On initial load
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr root) 
    {
      // Save pointer to the model
      modPtr = model;
      lnkPtr = model->GetLink("body");

      ////////////////////////
      // Configure from SDF //
      ////////////////////////

      // Configure the energy model from the SDF
      energy.Configure(root);

      // Configure the shear model from SDF
      shear.Configure(root);

      // Configure the turbulence model from SDF
      turbulence.Configure(root, lnkPtr);
      
      // Configure the dynamic model from SDF
      dynamics.Configure(root);

      // Configure the control model from SDF
      control.Configure(root);

      /////////////////////////////////////
      // Initialise gazbeo communication //
      /////////////////////////////////////

      // Setup the gazebo node
      nodePtr = gazebo::transport::NodePtr(new gazebo::transport::Node());

      // Subscribe to messages about atmospheric conditions
      subPtr = nodePtr->Subscribe("~/atmosphere", &Quadrotor::ReceiveAtmosphere, this);

      // Set up callback for updating the model dynamics (at physics rate)
      conPtr = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&Quadrotor::Update, this, _1));

      //////////////////////////////////////////
      // Initialise HAL and ROS communication //
      //////////////////////////////////////////

      // Call method periodically to broadcast state (respects simulation time)
      timState = GetROSNode().createTimer(
          ros::Duration(1.0),                                             // duration
          boost::bind(&Quadrotor::BroadcastState, this, _1),           // callback
          false                                                           // oneshot?
      );

      // Call method periodically to broadcast information (respects simulation time)
      timInformation = GetROSNode().createTimer(
          ros::Duration(1.0),                                             // duration
          boost::bind(&Quadrotor::BroadcastInformation, this, _1),     // callback
          false                                                           // oneshot?
      );


      // Animate the motors
      /*
      gazebo::common::PoseAnimationPtr anim(new gazebo::common::PoseAnimation("test", 1000.0, true));
      gazebo::common::PoseKeyFrame *key;
      key = anim->CreateKeyFrame(0);
      key->SetRotation(math::Quaternion(0, 0, 0));
      key->SetRotation(math::Quaternion(0, 0, 1.5707));
      _parent->SetAnimation(anim);
      */

    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Quadrotor)
}

