#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

// Classes for handling RINEX satellite navigation parameters (ephemerides)
#include <gpstk/Rinex3NavHeader.hpp>
#include <gpstk/Rinex3NavData.hpp>
#include <gpstk/Rinex3NavStream.hpp>

// Classes for handling RINEX files with meteorological parameters
#include <gpstk/RinexMetBase.hpp>
#include <gpstk/RinexMetData.hpp>
#include <gpstk/RinexMetHeader.hpp>
#include <gpstk/RinexMetStream.hpp>

namespace gazebo
{
  class Orbits : public WorldPlugin
  {
  public:

    Orbits() : WorldPlugin() {}

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      // Make sure the ROS node for Gazebo has already been initialized                                                                                    
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }

      ROS_INFO("Hello World!");
    }

  };
  
  GZ_REGISTER_WORLD_PLUGIN(Orbits)
} 