#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

namespace gazebo
{
  class Experiment : public WorldPlugin
  {
  public:

    Experiment() : WorldPlugin() {}

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {

    }

  };
  
  GZ_REGISTER_WORLD_PLUGIN(Experiment)
} 