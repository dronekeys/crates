// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// Basic ROS includes
#include <hal/sensor/Energy.h>

// Basic ROS stuff
#include <ros/ros.h>

namespace gazebo
{
  class Energy : public ModelPlugin, private hal::sensor::Energy
  {
  private:

    // Pointer to the current model
    physics::ModelPtr     modPtr;

    // Pointer to the update event connection
    event::ConnectionPtr  conPtr;

    // Parameters
    double energyTotal;
    double energyRemaining;
    double energyConsumptionBase;
    double energyRateFactor;
    double energyKill;
    double energyWarn;

    // How much battery remains (amp hours)
    double tim, remaining, consumption;

    // Called every time tick to work out energy consumption
    void PrePhysics(const gazebo::common::UpdateInfo &_info)
    {
      // Time over which dynamics must be updated (needed for thrust update)
      double dt = _info.simTime.Double() - tim;

      // If simulation is paused, dont waste CPU cycles calculating a physics update...
      if (dt > 0) 
      {
        // Store current consumption rate
        consumption = energyConsumptionBase * energyRateFactor * modPtr->GetWorldLinearAccel().z;

        // Calculate th
        remaining -= dt * consumption;
      }

      // Update timer
      tim = _info.simTime.Double();
    }


  public:

    // Constructor
    Energy() : energyTotal(2.2), energyRemaining(2.2), energyConsumptionBase(0.2), 
      energyRateFactor(6.6), energyKill(0.1), energyWarn(0.2), tim(0)
    {
      // Do nothing
    }

    // All sensors must be configured using the current model information and the SDF
    void Load(physics::ModelPtr model, sdf::ElementPtr root)
    {
      // Initialise the HAL
      hal::HAL::Init((std::string)"/hal/" + model->GetName());

      // Save the model
      modPtr = model;

      // Extract parameters
      root->GetElement("total")->GetValue()->Get(energyTotal);
      root->GetElement("remaining")->GetValue()->Get(energyRemaining);
      root->GetElement("consumption")->GetElement("base")->GetValue()->Get(energyConsumptionBase);
      root->GetElement("consumption")->GetElement("flight")->GetValue()->Get(energyRateFactor);
      root->GetElement("limits")->GetElement("warn")->GetValue()->Get(energyKill);
      root->GetElement("limits")->GetElement("kill")->GetValue()->Get(energyWarn);

      // Pre physics - update quadrotor dynamics and wind
      conPtr = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&Energy::PrePhysics, this, _1));

      // Call RESET on first init
      Reset();
    }

    // All sensors must be resettable
    void Reset()
    {
      // Reset the timer
      tim = 0.0;

      // Reset the tally
      consumption = 0;
      remaining   = energyRemaining;
    }

    // SENSOR SPECIFIC STUFF ///////////////////////////////////////

    // Get the current altitude
    bool GetMeasurement(hal_sensor_energy::Data& msg)
    {
      msg.total       = energyTotal;
      msg.remaining   = remaining;
      msg.consumption = consumption;
      return true;
    }

  };

  // Register the plugin
  GZ_REGISTER_MODEL_PLUGIN(Energy);
}


/*
// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// Basic ROS includes
#include <ros/ros.h>

// Basic constants 
#define METERS_TO_FEET    3.2808399
#define FEET_TO_METERS    0.3048000
#define DEGREES_TO_RADIANS  0.01745329258399
#define RADIANS_TO_DEGREES  57.2957795000000

namespace gazebo
{
  class Energy : public ModelPlugin
  {

  private:

    // Pointer to the model object
    physics::ModelPtr  modPtr;

    // Parameters
    double tot, rem, cb, cf, kill, warn, th;

    // How much battery remains (amp hours)
    double remaining, throttle;

  public:

    // Default constructor
    Energy(): tot(2.2), rem(2.2), cb(0.2), cf(6.6), kill(0.1), warn(0.2)
    {
      // Do nothing
    }

    // All sensors must be configured using the current model information and the SDF
    void Load(physics::ModelPtr model, sdf::ElementPtr root)
    {
      // Extract parameters
      root->GetElement("total")->GetValue()->Get(tot);
      root->GetElement("remaining")->GetValue()->Get(rem);
      root->GetElement("consumption")->GetElement("base")->GetValue()->Get(cb);
      root->GetElement("consumption")->GetElement("flight")->GetValue()->Get(cf);
      root->GetElement("limits")->GetElement("warn")->GetValue()->Get(warn);
      root->GetElement("limits")->GetElement("kill")->GetValue()->Get(kill);

      // Reset the energy
      Reset();
    }

    // All sensors must be resettable
    void Reset()
    {

    }

    // Update the model based on the time step (and internal control)
    void Update(const double &dt)
    {
      remaining -= dt * (cb + cf * throttle) / 3600;
    }

    // Set the throttle value
    void SetThrottle(const double &th)
    {
      throttle = th;
    }

    // Predict voltage -- this should definitely be improved (not linear in reality)
    double GetVoltage()
    {
      return 9 + (remaining / tot) * 3.5;
    }

  };

  // Register the plugin
  GZ_REGISTER_MODEL_PLUGIN(Energy);
}
*/