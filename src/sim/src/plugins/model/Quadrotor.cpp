// Standard libraries
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// Integration with the CRATES HAL
#include <hal_quadrotor/Quadrotor.h>

// Noise distributions
#include "../../noise/NoiseFactory.h"

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

// Custom messages
#include "noise.pb.h"

namespace gazebo
{
	typedef const boost::shared_ptr<const msgs::Noise> NoisePtr;

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

	    // Pointer to the update event connection
	    event::ConnectionPtr 		conPtr;

	    // Pointer to the model object
	    physics::LinkPtr 			linkPtr;

	    // Dynamics
	    gazebo::Aerodynamics    	dA;
	    gazebo::Propulsion			dP;
	    gazebo::Energy 				dE;

	    // Sensors
	    gazebo::Altimeter   		sA;
	    gazebo::Compass    			sC;
	    gazebo::GNSS    			sG;
	    gazebo::IMU    				sI;
	    gazebo::Orientation 		sO;

	    // Gazebo communication
		transport::NodePtr 			nodePtr;
		transport::SubscriberPtr 	subPtr;

	    // Last time tick
	    double 						tim;

    	// Callback for physics timer
		void PrePhysics(const common::UpdateInfo &_info)
		{
			// Time over which dynamics must be updated (needed for thrust update)
			double dt = _info.simTime.Double() - tim;

			// If simulation is paused, dont waste CPU calculating physics
			if (dt > 0) 
			{
				// Propulsive forces
				dP.Update(dt);

				// Shear, turbulence and drag forces
				dA.Update(dt);

				// Energy consumption
				dE.Update(dt);
			}

			// Update timer
			tim = _info.simTime.Double();
		}

		// This will be called whenever a new meteorlogical topic is posted
		void ReceiveNoise(NoisePtr& inmsg)
		{
			NoiseFactory::SetEnabled(inmsg->enable());
		}

  	public:

	    // Default constructor
	    Quadrotor() : hal::quadrotor::Quadrotor(), tim(0.0)
	    {
    		// Kill all random number streams
	    	NoiseFactory::Init();
	    }

	    /// Destructor
	    ~Quadrotor() 
	    {
	    	// Make sure we clean up the connection
    		event::Events::DisconnectWorldUpdateBegin(conPtr);

    		// Kill all random number streams
	    	NoiseFactory::Destroy();
	    }

	    // REQUIRED METHODS

	    // All sensors must be configured using the current model information and the SDF
	    void Load(physics::ModelPtr model, sdf::ElementPtr root)
	    {
			// FIND LINK AGAINST WHICH ALL FORCES AND SENSING IS DONE /////////////
	    	
	    	std::string linkName;
			root->GetElement("link")->GetValue()->Get(linkName);
			linkPtr = model->GetLink(linkName);

			// INITIALIZE THE NOISE DISTRIBUTION GENERATOR AND HAL ////////////////

	    	// Het
	    	std::string halName = (std::string)"/hal/" + model->GetName();
	    	hal::quadrotor::Quadrotor::Init(halName);
	    	hal::sensor::Altimeter::Init(halName);
	    	hal::sensor::Compass::Init(halName);
	    	hal::sensor::GNSS::Init(halName);
	    	hal::sensor::IMU::Init(halName);
	    	hal::sensor::Orientation::Init(halName);

			// DYNAMICS/SENSOR CONFIGURATION ///////////////////////////////////////

			// Configure the propulsion dynamics
			dP.Configure(linkPtr, root->GetElement("propulsion"));	

			// Configure the aerodynamics
			dA.Configure(linkPtr, root->GetElement("aerodynamics"));	

			// Configure the aerodynamics
			dE.Configure(linkPtr, root->GetElement("energy"));	

			// Configure the altitude sensor
			sA.Configure(linkPtr, root->GetElement("altimeter"));	

			// Configure the compass sensor
			sC.Configure(linkPtr, root->GetElement("compass"));
			
			// Configure the GNSS sensor
			sG.Configure(linkPtr, root->GetElement("gnss"));			
			
			// Configure the IMU sensor
			sI.Configure(linkPtr, root->GetElement("imu"));
			
			// Configure the orientation sensor
			sO.Configure(linkPtr, root->GetElement("orientation"));
			
			// WORLD UPDATE CALLBACK CONFIGURATION ////////////////////////////////

			// Create and initialize a new Gazebo transport node
			nodePtr = transport::NodePtr(new transport::Node());
			nodePtr->Init(model->GetName());

			// Subscribe to meteorological updates
			subPtr = nodePtr->Subscribe("~/noise", 
				&Quadrotor::ReceiveNoise, this);

			// WORLD UPDATE CALLBACK CONFIGURATION ////////////////////////////////

			// Create a pre-physics update call (before any physics)
			conPtr = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&Quadrotor::PrePhysics, this, _1));
	    }

	    // All sensors must be resettable
	    void Reset()
	    {
			// Reset the propulsion dynamics
			dP.Reset();	

			// Reset the aerodynamics
			dA.Reset();	

			// Reset the energy 
			dE.Reset();	

			// Reset the altitude sensor
			sA.Reset();

			// Reset the compass sensor
			sC.Reset();
			
			// Reset the GNSS sensor
			sG.Reset();
			
			// Reset the IMU sensor
			sI.Reset();
			
			// Reset the orientation sensor
			sO.Reset();		
	    }

	    /////////////////////////////////////////////////////////////////////////////////////
	    // HAL CALLBACKS ////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////////////////////////////

	    // Called when the HAL wants an altimeter reading
	    bool GetMeasurement(hal_sensor_altimeter::Data& msg)
	    {
	    	if (sA.GetMeasurement(tim,msg))
	    	{
		    	GetNavPtr()->Process(msg);		// Also use for navigation
		    	return true;
		    }
	    	return false;
	    }

		// Called when the HAL wants a compass reading
	    bool GetMeasurement(hal_sensor_compass::Data& msg)
	    {
	    	if (sC.GetMeasurement(tim,msg))
	    	{
		    	GetNavPtr()->Process(msg);		// Also use for navigation
		    	return true;
		    }
	    	return false;
	    }
	    
	    // Called when the HAL wants an imu reading
	    bool GetMeasurement(hal_sensor_imu::Data& msg)
	    {
	    	if (sI.GetMeasurement(tim,msg))
	    	{
		    	GetNavPtr()->Process(msg);		// Also use for navigation
		    	return true;
		    }
	    	return false;
	    }
	    
	    // Called when the HAL wants a gnss reading
	    bool GetMeasurement(hal_sensor_gnss::Data& msg)
	    {
	    	if (sG.GetMeasurement(tim,msg))
	    	{
		    	GetNavPtr()->Process(msg);		// Also use for navigation
		    	return true;
		    }
	    	return false;
	    }
	    
	    // Called when the HAL wants an orientation reading
	    bool GetMeasurement(hal_sensor_orientation::Data& msg)
	    {
	    	if (sO.GetMeasurement(tim,msg))
	    	{
		    	GetNavPtr()->Process(msg);		// Also use for navigation
		    	return true;
		    }
	    	return false;
	    }

	    // Called to arm or disarm the motors
		void ArmMotors(bool arm)
		{
			// Enable and disable the propulsion system
			dP.SetEnabled(arm);
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
			state.t         = tim;
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
			state.thrust 	= dP.GetThrustForce();
			state.remaining	= dE.GetRemaining();
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
			linkPtr->SetLinearVel(
				pose.rot.RotateVector(vel)
			);

			// Set the thrust force
			dP.SetThrustForce(state.thrust);

			// Set the remaining energy
			dE.SetRemaining(state.remaining);
		}

	    // Called when the HAL wants to pass down some control to the platform
		double SetControl(const hal_quadrotor::Control &control)
		{
			// Pass control to the propulsion module
			dP.SetControl(control);

			// Return the time at which control was applied
			return tim;
		}
	};

	// Register the plugin
	GZ_REGISTER_MODEL_PLUGIN(Quadrotor);
}