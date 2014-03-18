/* 
  This gazebo plugin provides a simple model for linear wind. 
*/

// Required to bind to non-static methods
#include <boost/bind.hpp>

// Gazebo libraries
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>

// Simulator description format (SDF)
#include <sdf/sdf.hh>

// Messages
#include "atmosphere.pb.h"

namespace uas_controller
{
  class Atmosphere : public gazebo::WorldPlugin
  {

  private:

    // Pointer to the current model
    gazebo::physics::WorldPtr       worldPtr;

    // Pointer to the update event connection
    gazebo::event::ConnectionPtr    conPtr;

    // Presents this as a node in the simulator
    gazebo::transport::Node         node;

    // Service that helps publish data
    gazebo::transport::PublisherPtr pubPtr;

    // Message contacining wind information
    msgs::Atmosphere                msg;

  public:
    
    Atmosphere() : WorldPlugin() {}

    // Called on plugin loaded
    void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      // Save the world points
      this->worldPtr = _world;

      // Initialize the node with the world name
      this->node.Init(_world->GetName());

      // Create a publisher on the ~/wind topic
      this->pubPtr = this->node.Advertise<msgs::Atmosphere>("~/atmosphere");

      // Set up callback for updating the model
      this->conPtr = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&Atmosphere::Update, this, _1));

      // Issue a reset immediately after load to initialise wind
      Reset();
    }

    // Broadcast the wind parameters
    void Update(const gazebo::common::UpdateInfo & _info)
    {
      // Package up the message
      msg.set_starttime(0.0);
      msg.set_temperature(0.0);
      msg.set_humidity(0.0);
      msg.set_wind_direction(0.0);
      msg.set_wind_speed(0.0);

      // Publish wind information to all subscribers
      pubPtr->Publish(msg);
    }

    // Called on load() or reset()
    void Reset()
    {
    
    }

  };

  GZ_REGISTER_WORLD_PLUGIN(Atmosphere)

} 