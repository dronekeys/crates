//  Boost includes
#include <boost/bind.hpp>

// ROS includes
#include <ros/ros.h>

// Gazebo includes
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// HAL includes
#include <uas_hal/peripheral/Inertial.h>

namespace uas_controller
{
  class IMU : public uas_hal::Inertial,  public gazebo::ModelPlugin
  {

  private:

    // Pointer to the current model
    gazebo::physics::ModelPtr  modPtr;

    // Listen to broadcasts from the atmosphere
    ros::Timer timer;

    // Prevents needless allocation
    gazebo::math::Vector3 linacc, angvel;

    // When called published the data
    void Update(const ros::TimerEvent& event)
    {
      // Rotate linear acceleration into body frame
      linacc = modPtr->GetWorldPose().rot.GetInverse().RotateVector(
        modPtr->GetWorldLinearAccel() + modPtr->GetWorld()->GetPhysicsEngine()->GetGravity()
      );

      // Rotate angular velocity into body frame
      angvel = modPtr->GetWorldPose().rot.GetInverse().RotateVector(
        modPtr->GetWorldAngularVel()
      );

      // Immediately post the inertial information
      Post(
        angvel.x,
        angvel.y,
        angvel.z,
        linacc.x,
        linacc.y,
        linacc.z
      );

    }

  public:

    IMU() : uas_hal::Inertial("imu")
    {
      ROS_INFO("Loaded imu plugin");
    }

    // On initial load
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) 
    {
      // Save the model pointer
      modPtr = _model;

      // Set up callback for updating the model
      timer = GetROSNode().createTimer(
        ros::Duration(1.0),                   // duration
        boost::bind(&IMU::Update, this, _1),    // callback
        false                                           // oneshot?
      );
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(IMU)
}