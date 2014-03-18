//  Boost includes
#include <boost/bind.hpp>

// ROS includes
#include <ros/ros.h>

// Gazebo includes
#include <gazebo/physics/Model.hh>
#include <gazebo/gazebo.hh>

// HAL includes
#include <uas_hal/peripheral/Orientation.h>

namespace uas_controller
{
  class Inclinometer : public uas_hal::Orientation,  public gazebo::ModelPlugin
  {

  private:

    // Pointer to the current model
    gazebo::physics::ModelPtr  modPtr;

    // Pointer to the update event connection
    gazebo::event::ConnectionPtr conPtr;
    
    // Listen to broadcasts from the atmosphere
    ros::Timer timer;

    // Rotation
    gazebo::math::Vector3 rot;

  public:

    // On initial load
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) 
    {
      // Save the model pointer
      modPtr = _model;

      // Initialise the HAL
      initialize(((std::string) "/hal/" 
        +         (std::string) modPtr->GetName() 
        +         (std::string) "/inclinometer").c_str());    

      // Set up callback for updating the model
      timer = node.createTimer(
          ros::Duration(1.0),                   // duration
          boost::bind(&Inclinometer::Update, this, _1),    // callback
          false                                           // oneshot?
      );
    }

    // When called published the data
    void Update(const ros::TimerEvent& event)
    {
      // Convert orientation to euler angles
      rot = modPtr->GetWorldPose().rot.GetAsEuler();

      // Immediately post the euler angles
      post(
        rot.x,  // Rotation aroudn the X axis in rads
        rot.y,  // Rotation aroudn the Y axis in rads
        rot.z   // Rotation aroudn the Z axis in rads
      );

    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Inclinometer)
}