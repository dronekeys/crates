#include <gazebo/math/Rand.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>

namespace controller
{
  class Interface : public gazebo::SystemPlugin
  {
  public:

    void Load(int /*_argc*/, char ** /*_argv*/)
    {
      conPtr = gazebo::event::Events::ConnectPreRender(
        boost::bind(&Interface::Update, this)
      );
    }

  private: 

    gazebo::event::ConnectionPtr conPtr;

    void Update()
    {

    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(Interface)
}