// ROS includes
#include <ros/ros.h>

// Gazebo includes
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

namespace gazebo
{
  class Experiment : public WorldPlugin
  {
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
      // Make sure the ROS node for Gazebo has already been initialized                                                                                    
      if (!ros::isInitialized())
      {
        ROS_FATAL("A ROS node for Gazebo has not been initialized. Unable to load plugin.");
        return;
      }
      ROS_INFO("Simulator plugin for Gazebo loaded!");

      /*
      // Option 1: Insert model from file via function call.
      // The filename must be in the GAZEBO_MODEL_PATH environment variable.
      _parent->InsertModelFile("model://box");

      // Option 2: Insert model from string via function call.
      // Insert a sphere model from string
      sdf::SDF sphereSDF;
      sphereSDF.SetFromString(
         "<sdf version ='1.4'>\
            <model name ='sphere'>\
              <pose>1 0 0 0 0 0</pose>\
              <link name ='link'>\
                <pose>0 0 .5 0 0 0</pose>\
                <collision name ='collision'>\
                  <geometry>\
                    <sphere><radius>0.5</radius></sphere>\
                  </geometry>\
                </collision>\
                <visual name ='visual'>\
                  <geometry>\
                    <sphere><radius>0.5</radius></sphere>\
                  </geometry>\
                </visual>\
              </link>\
            </model>\
          </sdf>");
      // Demonstrate using a custom model name.
      sdf::ElementPtr model = sphereSDF.root->GetElement("model");
      model->GetAttribute("name")->SetFromString("unique_sphere");
      _parent->InsertModelSDF(sphereSDF);
      */
    }
  
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(Experiment)

}