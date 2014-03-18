/* 
  This gazebo plugin provides a simple model for aerodynamic turbulence
*/

// Required to bind to non-static methods
#include <boost/bind.hpp>

// Gazebo API
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>

// Useful includes
#define METERS_TO_FEET 3.28083990000
#define FEET_TO_METERS 0.30479999953
#define WIND_Z0        0.15000000000

namespace uas_controller
{
  class Turbulence : public gazebo::ModelPlugin
  {

  private:

    // Presents this as a node in the simulator
    gazebo::transport::Node           node;

    // Pointer to the current model
    gazebo::physics::ModelPtr         modelPtr;

    // Pointer to the update event connection
    gazebo::event::ConnectionPtr      conPtr;

    // Transport-related stuff
    gazebo::transport::NodePtr        nodePtr;
    gazebo::transport::SubscriberPtr  subPtr;

  public: 

    // On initial plugin load
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) 
    {
      // Save pointer to the model
      this->modelPtr = _model;

      // Initialize the node with the world name
      /*
      this->node.Init(_world->GetName());
      */
    }


  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Turbulence)
}