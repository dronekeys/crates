// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// Basic ROS includes
#include <hal/sensor/Altimeter.h>

// Basic ROS stuff
#include <ros/ros.h>

namespace gazebo
{
  class Altimeter : public ModelPlugin, private hal::sensor::Altimeter
  {
  private:

    // Pointer to the current model
    physics::ModelPtr modPtr;

  	// Store temp, humidity and pressure
  	double t0, p0, h0;

  public:

    // All sensors must be configured using the current model information and the SDF
    void Load(physics::ModelPtr model, sdf::ElementPtr root)
    {
    	// Initialise the HAL
       	hal::HAL::Init((std::string)"/hal/" + model->GetName());

    	// Save the model
    	modPtr = model;

    	// Call RESET on first init
    	Reset();
    }

    // All sensors must be resettable
    void Reset()
    {
    	// Do nothing
    }

    // SENSOR SPECIFIC STUFF ///////////////////////////////////////

    // Get the current altitude
    bool GetMeasurement(hal_sensor_altimeter::Data& msg)
	{
		msg.height   = modPtr->GetLink("body")->GetWorldPose().pos.z;
		msg.velocity = modPtr->GetLink("body")->GetWorldLinearVel().z;
		return true;
	}

    // Get the atmospheric pressure at the current altitude
    double GetPressure()
    {
		// Current altitude
		double h = modPtr->GetLink("body")->GetWorldPose().pos.z;

		// Gravitational field strength
		double g = modPtr->GetWorld()->GetPhysicsEngine()->GetGravity().GetLength();

		// Gas constant
		double R = 8.314472;

		// Standard molar mass of air
		double M = 0.0289644;

		// Dry adiabatic or saturated adiabatic lapse rates for air (Kelvin/m)
		// See: http://en.wikipedia.org/wiki/Troposphere
		double L = (h0 < 100.0 ? 0.00028295: 0.00027965);

		// Directly from http://en.wikipedia.org/wiki/Atmospheric_pressure
		return p0 * pow(1.0 - L*h/t0, (g-M) / (R-L));
    }

  };

  // Register the plugin
  GZ_REGISTER_MODEL_PLUGIN(Altimeter);
}