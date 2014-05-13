// Standard libraries
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
	class Drag : public ModelPlugin
 	{

  	private:

	    // Pointer to the model object
	    physics::ModelPtr  modPtr;

		// Pointer to the update event connection
	    event::ConnectionPtr conPtr;
	    
	    // Drag parameters
	    double _kuv, _kw, tim;

	    // Drag vector
	    math::Vector3 drag;

    	// Callback for physics timer
		void PrePhysics(const common::UpdateInfo &_info)
		{
			// Time over which dynamics must be updated (needed for thrust update)
			double dt = _info.simTime.Double() - tim;

			// If simulation is paused, dont waste CPU cycles calculating a physics update...
			if (dt > 0) 
			{
			  	// set force and torque in gazebo
			  	modPtr->GetLink("body")->AddRelativeForce(
			  		drag*(modPtr->GetLink("body")->GetRelativeLinearVel())
		  		);
			}

			// Update timer
			tim = _info.simTime.Double();
		}

		// Update the system dynamics
		void Update(const double &dt)
		{
		  	// set force and torque in gazebo
		  	modPtr->GetLink("body")->AddRelativeForce(
		  		drag*(modPtr->GetLink("body")->GetRelativeLinearVel())
	  		);
		}

		// Clamp a value to a given range
		static double Clamp(const double& val, const double& minval, const double& maxval)
		{
		    if (val < minval) return minval;
		    if (val > maxval) return maxval;
		    return val;
		}

  	public:

	    // Default constructor
	    Drag() : _kuv(-4.97391e-01), _kw(-1.35341), tim(0.0)
	    {
	    	Reset();
	    }

	    // All model plugins must be configured using the current model information and the SDF
	    void Load(physics::ModelPtr model, sdf::ElementPtr root)
	    {
			// save the model pointer
			modPtr = model;

			// Control parameters
			root->GetElement("kuv")->GetValue()->Get(_kuv);
			root->GetElement("kw")->GetValue()->Get(_kw);

			// Set the drag constant vector for the platform
			drag.Set(_kuv, _kuv, _kw);
			drag *= modPtr->GetLink("body")->GetInertial()->GetMass();

			// Always call a reset 
			Reset();
	    }

	    // All sensors must be resettable
	    void Reset()
	    {
	    	// Reset the timer
	    	tim = 0.0;

			//  Create a pre-physics update call
			conPtr = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&Drag::PrePhysics, this, _1));
	    }
	};

	// Register the plugin
	GZ_REGISTER_MODEL_PLUGIN(Drag);
}