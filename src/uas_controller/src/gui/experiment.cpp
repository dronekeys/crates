#include <gazebo/math/Rand.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>

namespace gazebo
{
  class Experiment : public SystemPlugin
  {
  public:

    void Load(int /*_argc*/, char ** /*_argv*/)
    {
      conPtr = event::Events::ConnectPreRender(
        boost::bind(&Experiment::Update, this)
      );
    }

  private: 

    event::ConnectionPtr conPtr;

    void Update()
    {

    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(Experiment)
}