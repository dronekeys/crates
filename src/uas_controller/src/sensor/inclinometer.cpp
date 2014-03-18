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

  public:

    // On initial load
      void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) 
      {
      // Save the model pointer
      modPtr = _model;

      // Initialise the HAL
      initialize("inclinometer");            

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
      // Immediately post the z position and speed
      //post(modPtr->GetWorldPose().pos.z, modPtr->GetWorldLinearVel().z);
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Inclinometer)
}