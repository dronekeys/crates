/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

// Include Rand.hh first due to compilation error on osx (boost #5010)
// https://svn.boost.org/trac/boost/ticket/5010
#include <gazebo/math/Rand.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>

namespace gazebo
{
  class Simulation : public SystemPlugin
  {
  public:

    void Load(int /*_argc*/, char ** /*_argv*/)
    {
      conPtr = event::Events::ConnectPreRender(
        boost::bind(&Simulation::Update, this)
      );
    }

  private: 

    event::ConnectionPtr conPtr;

    void Update()
    {
      // Get scene pointer
      rendering::ScenePtr scene = rendering::get_scene();

      // Wait until the scene is initialized.
      if (!scene || !scene->GetInitialized())
        return;

      // Print out the total number of visuals in the scene
      std::cout << "Visual Count:" << scene->GetVisualCount() << std::endl;

      // Look for a specific visual by name.
      if (scene->GetVisual("ground_plane"))
        std::cout << "Has ground plane visual\n";
    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(Simulation)
}