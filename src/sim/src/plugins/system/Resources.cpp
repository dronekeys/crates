#include <gazebo/common/SystemPaths.hh>
#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
#include <ros/package.h>

#include <vector>

namespace gazebo
{
  class Resources : public SystemPlugin
  {

  public:

    Resources()
    {
      // set gazebo media paths by adding all packages that exports "gazebo_media_path" for gazebo
      common::SystemPaths::Instance()->gazeboPathsFromEnv = false;
      std::vector<std::string> gazebo_media_paths;
      ros::package::getPlugins("sim","gazebo_media_path",gazebo_media_paths);
      for (std::vector<std::string>::iterator iter=gazebo_media_paths.begin(); iter != gazebo_media_paths.end(); iter++)
      {
        ROS_DEBUG("med path %s",iter->c_str());
        common::SystemPaths::Instance()->AddGazeboPaths(iter->c_str());
      }

      // set gazebo plugins paths by adding all packages that exports "plugin_path" for gazebo
      common::SystemPaths::Instance()->pluginPathsFromEnv = false;
      std::vector<std::string> plugin_paths;
      ros::package::getPlugins("sim","plugin_path",plugin_paths);
      for (std::vector<std::string>::iterator iter=plugin_paths.begin(); iter != plugin_paths.end(); iter++)
      {
        ROS_DEBUG("plugin path %s",(*iter).c_str());
        common::SystemPaths::Instance()->AddPluginPaths(iter->c_str());
      }

      // set model paths by adding all packages that exports "gazebo_model_path" for gazebo
      common::SystemPaths::Instance()->modelPathsFromEnv = false;
      std::vector<std::string> model_paths;
      ros::package::getPlugins("sim","gazebo_model_path",model_paths);
      for (std::vector<std::string>::iterator iter=model_paths.begin(); iter != model_paths.end(); iter++)
      {
        ROS_DEBUG("model path %s",(*iter).c_str());
        common::SystemPaths::Instance()->AddModelPaths(iter->c_str());
      }

      // set .gazeborc path to something else, so we don't pick up default ~/.gazeborc
      std::string gazeborc = ros::package::getPath("sim")+"/.do_not_use_gazeborc";
      setenv("GAZEBORC",gazeborc.c_str(),1);
    }

    ~Resources()
    {
      // Do nothing
    };

    void Load(int argc, char** argv)
    {
      // Do nothing
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(Resources)

}