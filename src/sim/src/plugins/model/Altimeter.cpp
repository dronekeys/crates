// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// Basic ROS includes
#include <ros/ros.h>

// Basic constants 
#define METERS_TO_FEET 		3.2808399
#define FEET_TO_METERS 		0.3048000
#define DEGREES_TO_RADIANS  0.01745329258399
#define RADIANS_TO_DEGREES  57.2957795000000

namespace gazebo
{
	class Altimeter : public ModelPlugin
	{

	private:

	    // Pointer to the current model
	    gazebo::physics::ModelPtr	 modPtr;

	  	// Store temp, humidity and pressure
	  	double t0, p0, h0;

	public:

		// Defautl constructor
		Altimeter() : t0(300.0), p0(1000), h0(90.0)
		{
			// Do nothing
		}

        // All sensors must be configured using the current model information and the SDF
        void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr root)
        {
			// save the model pointer
			modPtr = model;

			// Parameters
			root->GetElement("temperature")->GetValue()->Get(t0);
			root->GetElement("pressure")->GetValue()->Get(p0);
			root->GetElement("humidity")->GetValue()->Get(h0);

			// Call a reset
			Reset();
        }

        // All sensors must be resettable
        void Reset()
        {

        }

        // Set the pressure and height at ground level
        void SetMeteorological(const double &te, const double &pr, const double &hu)
        {
			t0 = te;
			p0 = pr;
			h0 = hu;
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

        // Get the current altitude
        double GetAltitude()
		{
			return modPtr->GetLink("body")->GetWorldPose().pos.z;
		}

	};

	// Register the plugin
	GZ_REGISTER_MODEL_PLUGIN(Altimeter);
}