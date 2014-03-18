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

    // Pointer to the update event connection
    gazebo::event::ConnectionPtr conPtr;
      
    // Listen to broadcasts from the atmosphere
    ros::Timer timer;

    // Prevents needless allocation
    gazebo::math::Vector3 linacc, angvel;

  public:

    // On initial load
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) 
    {
      // Save the model pointer
      modPtr = _model;

      // Initialise the HAL
      initialize(((std::string) "/hal/" 
        +         (std::string) modPtr->GetName() 
        +         (std::string) "/imu").c_str());          

      // Set up callback for updating the model
      timer = node.createTimer(
        ros::Duration(1.0),                   // duration
        boost::bind(&IMU::Update, this, _1),    // callback
        false                                           // oneshot?
      );
    }

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
      post(
        angvel.x,
        angvel.y,
        angvel.z,
        linacc.x,
        linacc.y,
        linacc.z
      );

    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(IMU)
}