// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// Basic ROS includes
#include <ros/ros.h>

// Basic constants 
#define METERS_TO_FEET 		3.2808399
#define FEET_TO_METERS 		0.3048000
#define DEGREES_TO_RADIANS  0.01745329258399
#define RADIANS_TO_DEGREES  57.2957795000000

namespace gazebo
{
	class Compass : public ModelPlugin
	{

	private:

	    // Pointer to the current model
	    physics::ModelPtr modPtr;

	    // Magnetic vector
	    math::Vector3 mag;

	    // Current temperature
	    double t;

    public:

		// Default constructor
		Compass() : t(273.0) 
		{
			// Do nothing
		}

		// REQUIRED METHODS

		// All sensors must be configured using the current model information and the SDF
		void Load(physics::ModelPtr model, sdf::ElementPtr root)
		{
			// Save the model pointer
			modPtr = model;

			// Issue a Reset
			Reset();
		}

		// All sensors must be resettable
		void Reset()
		{

		}

		// Set the pressure and height at ground level
		void SetMeteorological( const double &te, const double &Bx, const double &By, const double &Bz)
		{
		  // Set the ground variables
		  t = te;

		  // Magnetic field
		  mag.Set(Bx,By,Bz);
		}

		// Set the pressure and height at ground level
		math::Vector3 GetMagneticField()
		{
			// Rotate world magnetic vector from global to local
			return modPtr->GetWorldPose().rot.GetInverse().RotateVector(mag);
		}

	};

	// Register the plugin
	GZ_REGISTER_MODEL_PLUGIN(Compass);
}