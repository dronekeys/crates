// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

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