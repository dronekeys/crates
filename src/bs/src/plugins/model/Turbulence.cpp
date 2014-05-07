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
	class Turbulence : public ModelPlugin
	{

	private:

        // Pointer to the model object
        physics::ModelPtr  modPtr;

	    // Pointer to the update event connection
	    event::ConnectionPtr conPtr;

        double tim;
        int     bsiter;
        double  bstime, speed, dir, kuv, kw;

    	// Used internally to maintain turbulence properties
        math::Vector3 wind, s, l, drag;
        math::Quaternion d20, q;
    	double d, a, h, k;
    	double s20;

    	// Callback for physics timer
		void PrePhysics(const common::UpdateInfo &_info)
		{
			// Time over which dynamics must be updated (needed for thrust update)
			double dt = _info.simTime.Double() - tim;

			// If simulation is paused, dont waste CPU cycles calculating a physics update...
			if (dt > 0) 
				Update(dt, true);

			// Update timer
			tim = _info.simTime.Double();
		}

		// Update the turbulence based on the time step
		void Update(const double &dt, bool apply)
		{
			// Extract the altitude and orientation from the state
			d = METERS_TO_FEET * modPtr->GetLink("body")->GetWorldLinearVel().GetLength() * dt;
			a = METERS_TO_FEET * modPtr->GetLink("body")->GetWorldPose().pos.z;
			q = modPtr->GetLink("body")->GetWorldPose().rot;

			// optimization
			k = 0.177 + 0.000823 * a;

			// sigma
			s.z = s20 * 0.1;
			s.y = s.z / pow(k, 0.4);
			s.x = s.y;
			
			// length scale
			l.z = a;
			l.y = l.z / pow(k, 1.2);
			l.x = l.y;
			
			// Gust X component
			wind.x = math::Rand::GetDblNormal(
				(1-d/l.x) *  wind.x,	// Mean
				sqrt(2*d/l.x) * s.x		// Standard deviation
			);

			// Gust Y component
			wind.y = math::Rand::GetDblNormal(
				(1-d/l.y) *  wind.y,	// Mean
				sqrt(2*d/l.y) * s.y		// Standard deviation
			);

			// Gust Z component
			wind.z = math::Rand::GetDblNormal(
				(1-d/l.z) *  wind.z,	// Mean
				sqrt(2*d/l.z) * s.z		// Standard deviation
			);

			// Actually, apply the force to the body
			if (apply)
			{
				modPtr->GetLink("body")->AddRelativeForce
				(
					-drag*(q.RotateVector(d20.GetInverse().RotateVector(FEET_TO_METERS * wind)))
				);
			}
		}

    public:

		// Default constructor
		Turbulence() : tim(0.0), bsiter(1000), bstime(0.02), speed(0), dir(0), kuv(-4.97391e-01), kw(-1.35341) 
		{
		    // Make sure that ROS actually started, or there will be some issues...
		    if (!ros::isInitialized())
		        ROS_FATAL("A ROS node has not been initialized");
		}

		//  Configure the propulsion engine
		void Load(physics::ModelPtr model, sdf::ElementPtr root)
		{
			// save the model pointer
			modPtr = model;

			// Speed and direction
			root->GetElement("bsiter")->GetValue()->Get(bsiter);
			root->GetElement("bstime")->GetValue()->Get(bstime);

			// Speed and direction
			root->GetElement("speed")->GetValue()->Get(speed);
			root->GetElement("direction")->GetValue()->Get(dir);

			// Aerodynamic drag (math::Rand::GetDblUniform(-MATH_PI,MATH_PI)))
			root->GetElement("drag")->GetElement("kuv")->GetValue()->Get(kuv);
			root->GetElement("drag")->GetElement("k")->GetValue()->Get(kw);

			// Set the drag vector
			drag.Set(kuv,kuv,kw);
			drag *= modPtr->GetLink("body")->GetInertial()->GetMass();

			//  Create a pre-physics update call
			conPtr = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&Turbulence::PrePhysics, this, _1));

			// Boostrap the wind
			Reset();
		}

		// Reset the propulsion engine
		void Reset()
		{
			// Resey internal timer
			tim = 0.0;

			// Wind direction expressed as a rotation quaternion
			d20.SetFromEuler(0.0, 0.0, DEGREES_TO_RADIANS * dir);

			// Get the speed at 20ft
			s20 = METERS_TO_FEET * speed;

			// Bootstrap the gust model, so the wind doesn't start at zero
			wind = math::Vector3(0,0,0);
			for (int i = 0; i < bsiter; i++)
				Update(bstime, false);
		}

	};

	// Register the plugin
	GZ_REGISTER_MODEL_PLUGIN(Turbulence);
}