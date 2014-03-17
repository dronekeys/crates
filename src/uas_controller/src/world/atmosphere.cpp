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

// Useful includes
#define METERS_TO_FEET 3.28083990000
#define FEET_TO_METERS 0.30479999953
#define WIND_Z0        0.15000000000

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
    gazebo::transport::PublisherPtr pubPtrWind;

    // Message contacining wind information
    msgs::Wind                      msgWind;

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
      this->pubPtrWind = this->node.Advertise<msgs::Wind>("~/wind");

      // Set up callback for updating the model
      this->conPtr = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&Wind::Update, this, _1));

      // Issue a reset immediately after load to initialise wind
      Reset();
    }

    // Broadcast the wind parameters
    void Update(const gazebo::common::UpdateInfo & _info)
    {
      // Package up the message
      msgWind.set_z0(0.0);
      msgWind.set_direction(0.0);
      msgWind.set_speed(0.0);

      // Publish wind information to all subscribers
      pubPtrWind->Publish(msgWind);
    }

    // Called on load() or reset()
    void Reset()
    {
    
    }

  };

  GZ_REGISTER_WORLD_PLUGIN(Atmosphere)

} 