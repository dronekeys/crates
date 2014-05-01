#ifndef SIM_CLIENT
#define SIM_CLIENT

#include <gazebo/common/SystemPaths.hh>
#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
#include <ros/package.h>

#include <map>

namespace gazebo
{
  class GazeboRosPathsPlugin : public SystemPlugin
  {

  public:
    
    /// Constructor
    GazeboRosPathsPlugin();

    /// Destructor
    ~GazeboRosPathsPlugin();

    /// Called when plugin is initialised
    void Init();

    /// Called on plugin load
    void Load(int argc, char** argv);

  };
}

#endif