// Gazebo includes
#include <gazebo/gazebo.hh>

namespace test
{
  // This class is responsible for managing the entire simulation. Most importantly,
  // it maintains the latest global time, and broadcasts meteorological information,
  // satellite positions and world properties (gravity, meagnetic, home position).
  class TestPlugin : public gazebo::WorldPlugin
  {

  private:

	// Pointer to the current model
	gazebo::physics::WorldPtr       worldPtr;

  public:
	
	// When the plugin is loaded ...
	void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr root)
	{
	  	// Do nothing
	}

	// When the simulation is reset...
	void Reset() 
	{
		// Do nothing
	}
  };

  GZ_REGISTER_WORLD_PLUGIN(TestPlugin)

} 