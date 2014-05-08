// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// Basic ROS includes
#include <hal/sensor/IMU.h>

// Basic ROS stuff
#include <ros/ros.h>

namespace gazebo
{
  class IMU : public ModelPlugin, private hal::sensor::IMU
  {
  private:

    // Pointer to the current model
    physics::ModelPtr modPtr;

  public:

    // All sensors must be configured using the current model information and the SDF
    void Load(physics::ModelPtr model, sdf::ElementPtr root)
    {
      hal::HAL::Init((std::string)"/hal/" + model->GetName());
    }

    // Set the pressure and height at ground level
    math::Vector3 GetAngularVelocity()
    {
      // Rotate angular velocity into body frame
      return modPtr->GetWorldPose().rot.GetInverse().RotateVector(
        modPtr->GetWorldAngularVel()
      );
    }

    // Set the pressure and height at ground level
    math::Vector3 GetLinearAcceleration()
    {
      // Rotate linear acceleration (specific force + gravity) into body frame
      return modPtr->GetWorldPose().rot.GetInverse().RotateVector(
        modPtr->GetWorldLinearAccel() + modPtr->GetWorld()->GetPhysicsEngine()->GetGravity()
      );
    }

    // All sensors must be resettable
    void Reset()
    {

    }

  };

  // Register the plugin
  GZ_REGISTER_MODEL_PLUGIN(IMU);
}

/*
namespace gazebo
{
  class IMU : public ModelPlugin, public hal::sensor::IMU
  {

  private:

    // Pointer to the current model
    physics::ModelPtr    modPtr;

    // Gravity and magnetic fields
    math::Vector3        grav;

    // Current temperature
    double t;
  
  public:

    // Default constructor
    IMU() : t(273.0), hal::sensor::IMU(ros::NodeHandle())
    {
      // Do nothing
    }

    // REQUIRED METHODS

    // All sensors must be configured using the current model information and the SDF
    void Load(physics::ModelPtr model, sdf::ElementPtr root)
    {
      // Save the model pointer
      modPtr = model;

      // Issue a Reset
      Reset();
    }

    // All sensors must be resettable
    void Reset()
    {

    }

    // Set the pressure and height at ground level
    void SetMeteorological( const double &te, const double &Gx, const double &Gy, const double &Gz)
    {
      // Set the ground variables
      t = te;

      // Magnetic and gravitational fields
      grav.Set(Gx,Gy,Gz);
    }

    // Set the pressure and height at ground level
    math::Vector3 GetAngularVelocity()
    {
      // Rotate angular velocity into body frame
      return modPtr->GetWorldPose().rot.GetInverse().RotateVector(
        modPtr->GetWorldAngularVel()
      );
    }

    // Set the pressure and height at ground level
    math::Vector3 GetLinearAcceleration()
    {
      // Rotate linear acceleration (specific force + gravity) into body frame
      return modPtr->GetWorldPose().rot.GetInverse().RotateVector(
        modPtr->GetWorldLinearAccel() + grav
      );
    }


  };

  // Register the plugin
  GZ_REGISTER_MODEL_PLUGIN(IMU);
}
*/
