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
    gazebo::physics::ModelPtr     modPtr;

    // Pointer to the update event connection
    gazebo::event::ConnectionPtr  conPtr;

    // Updating in each loop iteration
    double dFth, tau, tim;

    // The state of the UAV
    gazebo::math::Vector3 n_pos, b_vel, b_for;  // n-Position, b-velocity, b-force
    gazebo::math::Vector3 n_rot, b_ang, b_tor;  // n-Rotation, b-angvel, b-torque
    gazebo::math::Vector3 b_wnd;                // b-wind
    double thrust, battery;       // b-Thrust, n-windspd, n-wnddir

    // Timers for state and information broadcasting
    ros::Timer timState;
    ros::Timer timInformation;

    // Required for receiving body-frame wind updates
    gazebo::transport::NodePtr        nodePtr;
    gazebo::transport::SubscriberPtr  subPtr;

    // Stores current local wind information
    Shear       shear;
    Turbulence  turbulence;

    // Pointer to the current control
    uas_hal::Control ctl;

    // When new control arrives from the HAL, save it...
    void HalProcessControl(const uas_hal::Control & _ctl)
    {
      ctl = _ctl;
    }

    // Utility function for parsing SDF
    double GetParameter(sdf::ElementPtr _sdf, const char* name, double val)
    {
      double ret = val;
      if (_sdf->HasElement(name))
      {
        _sdf->GetElement(name)->GetValue()->Get(ret);
        std::cout << "Found parameter " << name << " with value " <<  ret << std::endl;
      }
      return val;
    }

  public: 

    // On initial load
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) 
    {
      // Save pointer to the model
      modPtr = _model;

      ////////////////////////
      // Get the parameters //
      ////////////////////////

      _LOW_THROTT = GetParameter(_sdf, "low_throttle",  300.0);
      _MAX_ANGVEL = GetParameter(_sdf, "max_angvel",    2.617993877991494);
      _pq0        = GetParameter(_sdf, "pq0",          -3.25060e-04);
      _pq1        = GetParameter(_sdf, "pq1",           1.79797e+02);
      _pq2        = GetParameter(_sdf, "pq2",          -24.3536);
      _r0         = GetParameter(_sdf, "r0",           -4.81783e-03);
      _r1         = GetParameter(_sdf, "r1",           -5.08944);
      _Cth0       = GetParameter(_sdf, "Cth0",          6.63881e-01);
      _Cth1       = GetParameter(_sdf, "Cth1",          7.44649e-04);
      _Cth2       = GetParameter(_sdf, "Cth2",          2.39855e-06);
      _Cvb0       = GetParameter(_sdf, "Cvb0",         -18.0007);
      _Cvb1       = GetParameter(_sdf, "Cvb1",          4.23754);
      _tau0       = GetParameter(_sdf, "tau0",          3.07321);
      _tau1       = GetParameter(_sdf, "tau1",          46.8004);
      _kuv        = GetParameter(_sdf, "kuv",          -4.97391e-01);
      _kw         = GetParameter(_sdf, "kw",           -1.35341);

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
      // Immediately publish the state
      HalBroadcastState(
        n_pos.x, n_pos.y, n_pos.z,  // Position
        n_rot.x, n_rot.y, n_rot.z,  // Orientation
        b_vel.x, b_vel.y, b_vel.z,  // Velocity
        b_ang.x, b_ang.y, b_ang.z,  // Angular velocity
        thrust, battery             // Thrust force
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
      /////////////////////////
      // Calculate time step //
      /////////////////////////

      // Time over which dynamics must be updated (needed for thrust update)
      dt = _info.simTime.Double() - tim;

      // If simulation is paused, dont waste CPU cycles calculating a physics update...
      if (dt == 0.0) 
        return;

      //////////////////////////
      // Extract state values //
      //////////////////////////

      // Get the b-frame linear velocity
      b_vel = modPtr->GetWorldPose().rot.GetInverse().RotateVector(
        model->GetWorldLinearVel()
      );
      
      // Get the b-frame angular velocity
      b_ang = modPtr->GetWorldPose().rot.GetInverse().RotateVector(
        model->GetWorldAngularVel()
      );

      //////////////////////////
      // Get the wind values  //
      //////////////////////////

      // Wind = shear(altitude) + turbulence(altitude,speed,dt)
      b_wnd = shear.GetGlobalVelocity(n_pos.z)
            + turbulence.GetGlobalVelocity(n_pos.z,b_vel.GetLength(),dt);

      // Navigation frame -> Body frame
      b_wnd = modPtr->GetWorldPose().rot.RotateVector(b_wnd);

      ////////////////////////////////////////////
      // Calculate b-frame angular acceleration //
      ////////////////////////////////////////////

      // X torque
      b_tor.x = _pq1*(_pq0*ctl.r - n_rot.x) + _pq2*b_ang.x;    
      if ((b_ang.x > _MAX_ANGVEL && (b_tor.x > 0)) || (b_ang.x < -_MAX_ANGVEL && (b_tor.x < 0)))
          b_tor.x = 0;

      // Y torque
      b_tor.y = _pq1*(_pq0*ctl.p - n_rot.y) + _pq2*b_ang.y;    
      if ((b_ang.y > _MAX_ANGVEL && (b_tor.y > 0)) || (b_ang.y < -_MAX_ANGVEL && (b_tor.y < 0)))
          b_tor.y = 0;

      // Z torque
      b_tor.z = _r0*ctl.y + _r1*b_ang.z;
     
      ////////////////////////////
      // Calculate thrust force //
      ////////////////////////////

      dFth = ((_Cth0 + _Cth1*ctl.t + _Cth2*ctl.t*ctl.t) - thrust);
      if (ctl.t < _LOW_THROTT)
        dFth = _tau0 * dFth;
      else
      {
        tau  = 0;
        if (abs(dFth) < (_tau1*dt))
          tau = dFth / dt;
        else
          tau = (dFth > 0 ? _tau1 : -_tau1);
        
        if ((thrust + tau*dt) > (_Cvb0 + _Cvb1*ctl.v))
          dFth = (_Cvb0 + _Cvb1*ctl.v - thrust) / dt;
        else
          dFth = tau;
      }

      // Update the drag force
      b_for = drag * (b_vel + b_wnd);   // Drag force
      b_for.z = thrust + dFth;          // Thrust force

      /////////////////////////////////////////
      // Add the force to the quadrotor body //
      /////////////////////////////////////////

      // Linear force 
      modPtr->GetLink("body")->AddForce(modPtr->GetWorldPose().rot.RotateVector(b_for));
     
      // Angular force
      modPtr->GetLink("body")->AddTorque(modPtr->GetWorldPose().rot.RotateVector(b_tor));

      ///////////////////////////
      // Save the current time //
      ///////////////////////////

      tim = _info.simTime.Double();
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Quadrotor)
}

