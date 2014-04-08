// Gazebo includes
#include <gazebo/gazebo.hh>

// Components of the simulator
#include "world/Environment.h"
#include "world/Meteorological.h"
#include "world/Satellites.h"

namespace controller
{
  // Forward declaration of the static module variables
  gpstk::CivilTime  	startTime;
  gpstk::CommonTime 	currentTime;

  // This class is responsible for managing the entire simulation. Most importantly,
  // it maintains the latest global time, and broadcasts meteorological information,
  // satellite positions and world properties (gravity, meagnetic, home position).
  class Simulation : public gazebo::WorldPlugin
  {

  private:

	// Pointer to the current model
	gazebo::physics::WorldPtr       worldPtr;

	// Pointer to the update event connection
	gazebo::event::ConnectionPtr    conPtr;

	// World modules
	Environment                     environment;
	Meteorological                  meteorological;
	Satellites                      satellites;

  public:
	
	// Constructor
	Simulation() : WorldPlugin() {}

	// When the plugin is loaded ...
	void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr root)
	{
	  // Save the world pointer
	  worldPtr = _world;

	  // Configure the gravitational fiel, magnetic field and ECEF home position
	  environment.Configure(root->GetElement("world"),worldPtr);

	  // Configure the temperature, pressure, humidty and wind broadcasting
	  meteorological.Configure(root->GetElement("meteorological"),worldPtr);

	  // Configure the GNSS satellites
	  satellites.Configure(root->GetElement("satellites"),worldPtr);
	}

	// When the simulation is reset...
	void Reset() 
	{
	  // Reset the environment
	  environment.Reset();

	  // Reset all meteorological effects
	  meteorological.Reset();

	  // Reset all satellites
	  satellites.Reset();
	}
  };

  GZ_REGISTER_WORLD_PLUGIN(Simulation)

} 