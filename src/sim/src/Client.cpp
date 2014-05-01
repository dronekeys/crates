#include "Client.h"

namespace gazebo 
{
  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosPathsPlugin)

  GazeboRosPathsPlugin::GazeboRosPathsPlugin()
  {
    // set gazebo media paths by adding all packages that exports "gazebo_media_path" for gazebo
    gazebo::common::SystemPaths::Instance()->gazeboPathsFromEnv = false;
    std::vector<std::string> gazebo_media_paths;
    ros::package::getPlugins("sim","gazebo_media_path",gazebo_media_paths);
    for (std::vector<std::string>::iterator iter=gazebo_media_paths.begin(); iter != gazebo_media_paths.end(); iter++)
    {
      ROS_DEBUG("med path %s",iter->c_str());
      gazebo::common::SystemPaths::Instance()->AddGazeboPaths(iter->c_str());
    }

    // set gazebo plugins paths by adding all packages that exports "plugin_path" for gazebo
    gazebo::common::SystemPaths::Instance()->pluginPathsFromEnv = false;
    std::vector<std::string> plugin_paths;
    ros::package::getPlugins("sim","plugin_path",plugin_paths);
    for (std::vector<std::string>::iterator iter=plugin_paths.begin(); iter != plugin_paths.end(); iter++)
    {
      ROS_DEBUG("plugin path %s",(*iter).c_str());
      gazebo::common::SystemPaths::Instance()->AddPluginPaths(iter->c_str());
    }

    // set model paths by adding all packages that exports "gazebo_model_path" for gazebo
    gazebo::common::SystemPaths::Instance()->modelPathsFromEnv = false;
    std::vector<std::string> model_paths;
    ros::package::getPlugins("sim","gazebo_model_path",model_paths);
    for (std::vector<std::string>::iterator iter=model_paths.begin(); iter != model_paths.end(); iter++)
    {
      ROS_DEBUG("model path %s",(*iter).c_str());
      gazebo::common::SystemPaths::Instance()->AddModelPaths(iter->c_str());
    }

    // set .gazeborc path to something else, so we don't pick up default ~/.gazeborc
    std::string gazeborc = ros::package::getPath("sim")+"/.do_not_use_gazeborc";
    setenv("GAZEBORC",gazeborc.c_str(),1);
  }

  GazeboRosPathsPlugin::~GazeboRosPathsPlugin()
  {
    // Do nothing
  };

  void GazeboRosPathsPlugin::Init()
  {
    // Do nothing
  }

  void GazeboRosPathsPlugin::Load(int argc, char** argv)
  {
    // Do nothing
  }
}
