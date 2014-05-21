// Standard libraries
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// Integration with the CRATES HAL
#include <hal/quadrotor/Quadrotor.h>
#include <hal/sensor/Altimeter.h>
#include <hal/sensor/Compass.h>
#include <hal/sensor/GNSS.h>
#include <hal/sensor/IMU.h>
#include <hal/sensor/Orientation.h>

// Dynamics
#include "dynamics/Propulsion.h"
#include "dynamics/Aerodynamics.h"
#include "dynamics/Energy.h"

// Sensors
#include "sensors/Altimeter.h"
#include "sensors/Compass.h"
#include "sensors/GNSS.h"
#include "sensors/IMU.h"
#include "sensors/Orientation.h"

namespace gazebo
{
	class Quadrotor : 
		public ModelPlugin, 				/* Needed for Gazebo integration */
		public hal::quadrotor::Quadrotor,	/* Exposes Quadrotor model       */
		public hal::sensor::Altimeter,		/* Exposes Altimeter sensor      */
		public hal::sensor::Compass,		/* Exposes Compass sensor      	 */
		public hal::sensor::GNSS,			/* Exposes GNSS sensor      	 */
		public hal::sensor::IMU,			/* Exposes IMU sensor      		 */
		public hal::sensor::Orientation 	/* Exposes Orientation sensor    */
 	{

  	private:

	    // Pointer to the model object
	    physics::LinkPtr 		linkPtr;

	    // Pointer to the update event connection
	    event::ConnectionPtr 	conPtrBeg, conPtrEnd;

	    // Dynamics
	    Aerodynamics 	dAerodynamics;
	    Propulsion		dPropulsion;
	    Energy 			dEnergy;

	    // Sensors
	    Altimeter   	sAltimeter;
	    Compass    		sCompass;
	    GNSS    		sGNSS;
	    IMU    			sIMU;
	    Orientation 	sOrientation;

	    // Last time tick
	    double 			tim;

    	// Callback for physics timer
		void PrePhysics(const common::UpdateInfo &_info)
		{
			// Time over which dynamics must be updated (needed for thrust update)
			double dt = _info.simTime.Double() - tim;

			// If simulation is paused, dont waste CPU calculating physics
			if (dt > 0) 
			{
				// Propulsive forces
				dPropulsion.Update(linkPtr, dt);

				// Shear, turbulence and drag forces
				dAerodynamics.Update(linkPtr, dt);

				// Energy consumption
				dEnergy.Update(linkPtr, dt);
			}

			// Update timer
			tim = _info.simTime.Double();
		}

  	public:

	    // Default constructor
	    Quadrotor() : hal::model::Quadrotor(), tim(0.0)
	    {
	      // Do nothing
	    }

	    /// Destructor
	    ~Quadrotor() 
	    {
	    	// Make sure we clean up the connection
    		event::Events::DisconnectWorldUpdateBegin(conPtr);
	    }

	    // REQUIRED METHODS

	    // All sensors must be configured using the current model information and the SDF
	    void Load(physics::ModelPtr model, sdf::ElementPtr root)
	    {
	    	// BASE INITIALIZATION ///////////////////////////////////////////////

	    	// Initilize the HAL
	    	hal::HAL::Init((std::string)"/hal/" + model->GetName());

			// Get a pointer to the rigid body against which all forces are applied
	    	std::string linkName;
			root->GetElement("link")->GetValue()->Get(linkName)
			linkPtr = model->GetLink(linkName);

			// DYNAMICS CONFIGURATION /////////////////////////////////////////////

			// Configure the propulsion dynamics
			dPropulsion.Configure(root->GetElement("propulsion"));	

			// Configure the aerodynamics
			dAerodynamics.Configure(root->GetElement("aerodynamics"));	

			// Configure the aerodynamics
			dEnergy.Configure(root->GetElement("energy"));	

			// SENSOR CONFIGURATION ///////////////////////////////////////////////

			// Configure the altitude sensor
			sAltimeter.Configure(root->GetElement("altimeter"));	

			// Configure the compass sensor
			sCompass.Configure(root->GetElement("compass"));
			
			// Configure the GNSS sensor
			sGNSS.Configure(root->GetElement("gnss"));			
			
			// Configure the IMU sensor
			sIMU.Configure(root->GetElement("imu"));
			
			// Configure the orientation sensor
			sOrientation.Configure(root->GetElement("orientation"));
			
			// WORLD UPDATE CALLBACK CONFIGURATION ////////////////////////////////

			// Create a pre-physics update call (before any physics)
			conPtrBeg = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&Quadrotor::PrePhysics, this, _1));
			
			// Create a post-physics update call (after any physics)
			conPtrEnd = event::Events::ConnectWorldUpdateEnd(
				boost::bind(&Quadrotor::PostPhysics, this, _1));

			// Aways issue a reset on load
			Reset();
	    }

	    // All sensors must be resettable
	    void Reset()
	    {
			// Reset the propulsion dynamics
			dPropulsion.Reset();	

			// Reset the aerodynamics
			dAerodynamics.Reset();	

			// Reset the aerodynamics
			dEnergy.Reset();	

			// Reset the altitude sensor
			sAltimeter.Reset();

			// Reset the compass sensor
			sCompass.Reset();
			
			// Reset the GNSS sensor
			sGNSS.Reset();
			
			// Reset the IMU sensor
			sIMU.Reset();
			
			// Reset the orientation sensor
			sOrientation.Reset();		
	    }

	    /////////////////////////////////////////////////////////////////////////////////////
	    // HAL CALLBACKS ////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////////////////////////////

	    // Called when the HAL wants an altimeter reading
	    bool GetMeasurement(hal_sensor_altimeter::Data& msg)
	    {
	    	if (sAltimeter.Sample(msg))
	    	{
		    	Feed(msg);
		    	return true;
		    }
	    	return false;
	    }

		// Called when the HAL wants a compass reading
	    bool GetMeasurement(hal_sensor_compass::Data& msg)
	    {
	    	if (sCompass.Sample(msg))
	    	{
		    	Feed(msg);
		    	return true;
		    }
	    	return false;
	    }
	    
	    // Called when the HAL wants an imu reading
	    bool GetMeasurement(hal_sensor_imu::Data& msg)
	    {
	    	if (sIMU.Sample(msg))
	    	{
		    	Feed(msg);
		    	return true;
		    }
	    	return false;
	    }
	    
	    // Called when the HAL wants a gnss reading
	    bool GetMeasurement(hal_sensor_gnss::Data& msg)
	    {
	    	if (sGNSS.Sample(msg))
	    	{
		    	Feed(msg);
		    	return true;
		    }
	    	return false;
	    }
	    
	    // Called when the HAL wants an orientation reading
	    bool GetMeasurement(hal_sensor_orientation::Data& msg)
	    {
	    	if (sOrientation.Sample(msg))
	    	{
		    	Feed(msg);
		    	return true;
		    }
	    	return false;
	    }

    	// Called when the HAL wants the truthful state of the platform
		void GetTruth(hal_quadrotor::State& state)
		{
			// Get the state
			math::Vector3 pos = linkPtr->GetWorldPose().pos;
			math::Vector3 rot = linkPtr->GetWorldPose().rot.GetAsEuler();
			math::Vector3 vel = linkPtr->GetRelativeLinearVel();
			math::Vector3 ang = linkPtr->GetRelativeAngularVel();

			// Update the state
			state.x 		= pos.x;
			state.y 		= pos.y;
			state.z 		= pos.z;
			state.roll 		= rot.x;
			state.pitch 	= rot.y;
			state.yaw 		= rot.z;
			state.u 		= vel.x;
			state.v 		= vel.y;
			state.w 		= vel.z;
			state.p 		= ang.x;
			state.q 		= ang.y;
			state.r 		= ang.z;
			state.thrust 	= dPropulsion.GetThrust();
			state.remaining	= dEnergy.GetRemaining();
		}

	    // Called when the HAL wants to set the truthful state of the platform
		void SetTruth(const hal_quadrotor::State& state)
		{
			// Get the state
			math::Pose pose(
				state.x,    state.y,     state.z,		// Position
				state.roll, state.pitch, state.yaw 		// Orientation
			);
			math::Vector3 vel(state.u, state.v, state.w);
			math::Vector3 ang(state.p, state.q, state.r);

			// Set the kinematic quantities
			linkPtr->SetWorldPose(pose);
			linkPtr->SetAngularVel(
				pose.rot.RotateVector(ang)
			);
			linkPtr->SetLineaVel(
				pose.rot.RotateVector(vel)
			);

			// Set the thrust force
			dPropulsion.SetThrust(state.thrust);

			// Set the remaining energy
			dEnergy.SetRemaining(state.remaining);
		}

	    // Called when the HAL wants to pass down some control to the platform
		void SetControl(const hal_quadrotor::Control &control)
		{
			// Pass control to the propulsion module
			dPropulsion.SetControl(control);
		}
	};

	// Register the plugin
	GZ_REGISTER_MODEL_PLUGIN(Quadrotor);
}