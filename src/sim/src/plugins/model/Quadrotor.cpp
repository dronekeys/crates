// Standard libraries
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// Basic ROS includes
#include <hal/model/Quadrotor.h>

// Thrust considered to be too low to animate :)
#define MOTOR_ANIMATION_THRESHOLD 	0.001
#define MOTOR_ANIMATION_ON_RPM    	200.0
#define MATH_PI						3.1415

namespace gazebo
{
	class Quadrotor : public ModelPlugin, public hal::model::Quadrotor
 	{

  	private:

	    // Pointer to the model object
	    physics::ModelPtr modPtr;

	    // Pointer to the update event connection
	    event::ConnectionPtr conPtr;

	    // Last time tick
	    double tim;

	    // Control parameters
	    double srs, sps, sys, sts, svs;
	    double srl, spl, syl, stl, svl;
	    double sru, spu, syu, stu, svu;

	    // Dynamics parameters
	    double _LOW_THROTT, _MAX_ANGVEL, _G,  _MASS_KG;         // Platform config
	    double _pq0, _pq1, _pq2, _r0, _r1;                      // Rotational params
	    double _Cth0, _Cth1, _Cth2, _Cvb0, _Cvb1, _tau0, _tau1; // Thrust params

	    // Thrust force required to hover
		double hover;		
	    
	    // Current control parameters
	    double roll, pitch, yaw, throttle, voltage, thrust;
	  
	  	// Motor animations
	  	std::map<std::string,common::NumericAnimationPtr> anim;

		// Animate a motor without any dynamics
		void AnimateMotors(bool enabled)
		{
			// Create motor animations
			for (int i = 0; i < 4; i++)
			{
				// Assemble a motor and name for the naimation
				std::string motor = (std::string) "motor" + boost::lexical_cast<std::string>(i);
				std::string name  = (std::string) "rpm"   + boost::lexical_cast<std::string>(i);

				/*
				// Select a random offset, so that the quads dont all align
				double offset = math::Rand::GetDblUniform(-MATH_PI,MATH_PI);

				// Clear the animation
				anim.empty();

				// Create the animation
				anim[motor].reset(new common::NumericAnimation(name,4,true));
				if (enabled)
				{
					common::NumericKeyFrame *key;
					for (int i = 0; i <= 4; i++)
					{ 
						key = anim[motor]->CreateKeyFrame((double)i*4.0);
						key->SetValue((double)i*MATH_PI/2.0);
					}
				}
				*/


				// Attache the animatin to the the model
				//modPtr->SetJointAnimation(anim);
			}
		}

    	// Callback for physics timer
		void PrePhysics(const common::UpdateInfo &_info)
		{
			// Time over which dynamics must be updated (needed for thrust update)
			double dt = _info.simTime.Double() - tim;

			// If simulation is paused, dont waste CPU cycles calculating a physics update...
			if (dt > 0) 
			{
				// Get the orientation
				math::Quaternion q = modPtr->GetLink("body")->GetWorldPose().rot;
				math::Vector3 o = modPtr->GetLink("body")->GetRelativeAngularVel();

				// Update thrust force
				double dFth = (_Cth0 + _Cth1*throttle + _Cth2*throttle*throttle) - thrust;
				if (throttle < _LOW_THROTT)
					dFth = _tau0 * dFth;
				else
				{
					double tau  = 0.0;
					if (abs(dFth) < (_tau1*dt))
					  tau = dFth / dt;
					else
					  tau = (dFth > 0 ? _tau1 : -_tau1);

					if ((thrust + tau*dt) > (_Cvb0 + _Cvb1*voltage))
					  dFth = (_Cvb0 + _Cvb1*voltage - thrust) / dt;
					else
					  dFth = tau;
				}

				// Update thrust
				thrust += dFth * dt;

				// Calculate body-frame thrust force
				math::Vector3 force = math::Vector3(0.0,0.0,thrust);
				math::Vector3 torque;

				// x = roll
				torque.x = _pq1*(_pq0*roll  - q.GetAsEuler().x) + _pq2*o.x;    
				if ((o.x > _MAX_ANGVEL && (torque.x > 0)) || (o.x < -_MAX_ANGVEL && (torque.x < 0)))
				  torque.x = 0;

				// y = pitch
				torque.y = _pq1*(_pq0*pitch - q.GetAsEuler().y) + _pq2*o.y;    
				if ((o.y > _MAX_ANGVEL && (torque.y > 0)) || (o.y < -_MAX_ANGVEL && (torque.y < 0)))
				  torque.y = 0;

				// Yaw
				torque.z = _r0*yaw + _r1*o.z;

			  	// set force and torque in gazebo
			  	modPtr->GetLink("body")->AddRelativeForce(force);
			  	modPtr->GetLink("body")->AddRelativeTorque(torque);

			  	// Only update the on-off action of motors
			  	//if ((!motors && thrust > MOTOR_ANIMATION_THRESHOLD) || (motors && thrust < MOTOR_ANIMATION_THRESHOLD)) 
				//	AnimateMotors(thrust > MOTOR_ANIMATION_THRESHOLD);
			}

			// Update timer
			tim = _info.simTime.Double();
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
	    Quadrotor() : 

	    	// Quadrotor HAL
	    	hal::model::Quadrotor(),

	    	// Last time tick
	    	tim(0.0),

			// Control parameters
			srs(-2291.83118052329), srl(-0.9), 	sru(0.9), 
		    sps(-2291.83118052329), spl(-0.9), 	spu(0.9), 
			sys(-460.597254433196), syl(-4.5), 	syu(4.5), 
			sts( 4097.0), 			stl(0.0), 	stu(1.0),
			svs( 1.0), 				svl(9.0), 	svu(12.0),

			// Dynamics parameters
			_LOW_THROTT(300.0), _MAX_ANGVEL(2.617993877991494),
			_pq0(-3.25060e-04), _pq1(1.79797e+02), 	_pq2(-24.3536),
			_r0(-4.81783e-03),  _r1(-5.08944),
			_Cth0(6.63881e-01), _Cth1(7.44649e-04), _Cth2(2.39855e-06),
			_Cvb0(-18.0007), 	_Cvb1(4.23754),
			_tau0(3.07321), 	_tau1(46.8004),

			// Current control
			roll(0.0), pitch(0.0), yaw(0.0), throttle(0.0), voltage(12.0)
	    {
	      // Do nothing
	    }

	    // REQUIRED METHODS

	    // All sensors must be configured using the current model information and the SDF
	    void Load(physics::ModelPtr model, sdf::ElementPtr root)
	    {
	    	// Initilize the HAL
	    	hal::HAL::Init((std::string)"/hal/" + model->GetName());

			// save the model pointer
			modPtr = model;

			// Control parameters
			root->GetElement("control")->GetElement("roll")->GetElement("scale")->GetValue()->Get(srs);
			root->GetElement("control")->GetElement("roll")->GetElement("min")->GetValue()->Get(srl);
			root->GetElement("control")->GetElement("roll")->GetElement("max")->GetValue()->Get(sru);
			root->GetElement("control")->GetElement("pitch")->GetElement("scale")->GetValue()->Get(sps);
			root->GetElement("control")->GetElement("pitch")->GetElement("min")->GetValue()->Get(spl);
			root->GetElement("control")->GetElement("pitch")->GetElement("max")->GetValue()->Get(spu);
			root->GetElement("control")->GetElement("yaw")->GetElement("scale")->GetValue()->Get(sys);
			root->GetElement("control")->GetElement("yaw")->GetElement("min")->GetValue()->Get(syl);
			root->GetElement("control")->GetElement("yaw")->GetElement("max")->GetValue()->Get(syu);
			root->GetElement("control")->GetElement("throttle")->GetElement("scale")->GetValue()->Get(sts);
			root->GetElement("control")->GetElement("throttle")->GetElement("min")->GetValue()->Get(stl);
			root->GetElement("control")->GetElement("throttle")->GetElement("max")->GetValue()->Get(stu);
			root->GetElement("control")->GetElement("voltage")->GetElement("scale")->GetValue()->Get(svs);
			root->GetElement("control")->GetElement("voltage")->GetElement("min")->GetValue()->Get(svl);
			root->GetElement("control")->GetElement("voltage")->GetElement("max")->GetValue()->Get(svu);

			// Dynamics parameters
			root->GetElement("dynamics")->GetElement("low_throttle")->GetValue()->Get(_LOW_THROTT);
			root->GetElement("dynamics")->GetElement("max_angvel")->GetValue()->Get(_MAX_ANGVEL);
			root->GetElement("dynamics")->GetElement("pq0")->GetValue()->Get(_pq0);
			root->GetElement("dynamics")->GetElement("pq1")->GetValue()->Get(_pq1);
			root->GetElement("dynamics")->GetElement("pq2")->GetValue()->Get(_pq2);
			root->GetElement("dynamics")->GetElement("r0")->GetValue()->Get(_r0);
			root->GetElement("dynamics")->GetElement("r1")->GetValue()->Get(_r1);
			root->GetElement("dynamics")->GetElement("Cth0")->GetValue()->Get(_Cth0);
			root->GetElement("dynamics")->GetElement("Cth1")->GetValue()->Get(_Cth1);
			root->GetElement("dynamics")->GetElement("Cth2")->GetValue()->Get(_Cth2);
			root->GetElement("dynamics")->GetElement("Cvb0")->GetValue()->Get(_Cvb0);
			root->GetElement("dynamics")->GetElement("Cvb1")->GetValue()->Get(_Cvb1);
			root->GetElement("dynamics")->GetElement("tau0")->GetValue()->Get(_tau0);
			root->GetElement("dynamics")->GetElement("tau1")->GetValue()->Get(_tau1);

			// How much thrust force is required to hover (simple F = mG)
			hover = modPtr->GetLink("body")->GetInertial()->GetMass() 
				  * modPtr->GetWorld()->GetPhysicsEngine()->GetGravity().GetLength();

			//  Create a pre-physics update call
			conPtr = event::Events::ConnectWorldUpdateBegin(boost::bind(&Quadrotor::PrePhysics, this, _1));
			
			// Aways issue a reset on load
			Reset();
	    }

	    // All sensors must be resettable
	    void Reset()
	    {

	    }

    	// The HAL gets the simulated state through this function
		void GetState(hal_model_quadrotor::State& state)
		{
			// Get the state
			math::Pose pose = modPtr->GetWorldPose();
			math::Vector3 vel = modPtr->GetWorldLinearVel();
			math::Vector3 ang = modPtr->GetWorldAngularVel();

			// Update the state
			state.x = pose.pos.x;
			state.y = pose.pos.y;
			state.z = pose.pos.z;
			state.roll = pose.rot.x;
			state.pitch = pose.rot.y;
			state.yaw = pose.rot.z;
			state.u = vel.x;
			state.v = vel.y;
			state.w = vel.z;
			state.p = ang.x;
			state.q = ang.y;
			state.r = ang.z;
			state.thrust = thrust;
			state.voltage = voltage;
		}

	    // The HAL sets the simulated state through this function
		void SetState(const hal_model_quadrotor::State& state)
		{
			// Get the state
			math::Pose pose(
				state.x,    state.y,     state.z,		// Position
				state.roll, state.pitch, state.yaw 		// Orientation
			);
			math::Vector3 vel(state.u,state.v,state.w);
			math::Vector3 ang(state.p,state.q,state.r);

			// Set the state
			modPtr->SetWorldPose(pose);
			modPtr->SetWorldTwist(vel, ang);
			thrust = state.thrust;
			voltage = state.voltage;
		}

	    // The HAL sets the control through this function
		void SetControl(const hal_model_quadrotor::Control &control)
		{
			roll 		= srs * Clamp(control.roll,		srl,sru);
			pitch 		= sps * Clamp(control.pitch,	spl,spu);
			yaw 		= sys * Clamp(control.yaw,		syl,syu);
			throttle 	= sts * Clamp(control.throttle,	stl,stu);
			voltage 	= svs * Clamp(control.voltage,	svl,svu);
		}
	};

	// Register the plugin
	GZ_REGISTER_MODEL_PLUGIN(Quadrotor);
}