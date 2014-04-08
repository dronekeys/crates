// Gazebo includes
#include <gazebo/gazebo.hh>

// Components of the simulator
#include "world/Environment.h"
#include "world/Meteorological.h"
#include "world/Satellites.h"

namespace controller
{
  // Forward declaration of the static module variables
  gpstk::CivilTime  Feature::startTime;
  gpstk::CommonTime Feature::currentTime;

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
	Simulation() : WorldPlugin(), rosNode(ros::NodeHandle("controller")) {}

	// When the plugin is loaded ...
	void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr root)
	{
	  // Save the world pointer
	  worldPtr = _world;

	  // First, configure the feature class so that all children have access to time
	  Feature::Init(root->GetElement("time"));

	  // Configure the gravitational fiel, magnetic field and ECEF home position
	  environment.Configure(root->GetElement("world"),model);

	  // Configure the temperature, pressure, humidty and wind broadcasting
	  meteorological.Configure(root->GetElement("meteorological"),model);

	  // Configure the GNSS satellites
	  satellites.Configure(root->GetElement("satellites"),model);

	  // Set up callback for updating the timer
	  conPtr = gazebo::event::Events::ConnectWorldUpdateBegin(
		boost::bind(&Simulation::Update, this, _1));
	}

	// When the simulation is reset...
	void Reset() 
	{
	  // Reset the environment
	  environment.reset();

	  // Reset all meteorological effects
	  meteorological.reset();

	  // Reset all satellites
	  satellites.reset();
	}

	// Called at the resolution of physics updates
	void Update(const gazebo::common::UpdateInfo& _info)
	{
		// Update the internat prepresentation of time
	 	Feature::Update(_info.simTime.Double()); 
	}

  };

  GZ_REGISTER_WORLD_PLUGIN(Simulation)

} 