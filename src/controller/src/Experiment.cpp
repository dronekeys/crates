#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

namespace controller
{
  class Experiment : public gazebo::WorldPlugin
  {
  public:

    Experiment() : gazebo::WorldPlugin() {}

    void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {

    }

  };
  
  GZ_REGISTER_WORLD_PLUGIN(Experiment)
} 