#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

namespace gazebo
{
  class Listener : public WorldPlugin
  {
  public:

    Listener() : WorldPlugin() {}

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {

    }

  };
  
  GZ_REGISTER_WORLD_PLUGIN(Listener)
} 