/* This gazebo model plugin implements a second order model for quadrotor dynamics */ 
#include "Propulsion.h"

// Required for (int) -> (std::string)
#include <boost/lexical_cast.hpp>

// Thrust considered to be too low to animate :)
#define MOTOR_ANIMATION_THRESHOLD 0.001
#define MOTOR_ANIMATION_ON_RPM    200.0

using namespace uas_controller;

// Default constructor
Propulsion::Propulsion() : 

	// Control parameters
	srs(-2291.83118052329), srl(-0.9), sru(0.9), 
    sps(-2291.83118052329), spl(-0.9), spu(0.9), 
	sys(-460.597254433196), syl(-4.5), syu(4.5), 
	sts( 4097.0), 			stl(0.0), stu(1.0),
	svs( 1.0), 				svl(9.0), svu(12.0),

	// Dynamics parameters
	_LOW_THROTT(300.0), _MAX_ANGVEL(2.617993877991494),
	_pq0(-3.25060e-04), _pq1(1.79797e+02), 	_pq2(-24.3536),
	_r0(-4.81783e-03),  _r1(-5.08944),
	_Cth0(6.63881e-01), _Cth1(7.44649e-04), _Cth2(2.39855e-06),
	_Cvb0(-18.0007), 	_Cvb1(4.23754),
	_tau0(3.07321), 	_tau1(46.8004),
	_kuv(-4.97391e-01),	_kw(-1.35341),

	// Current control
	roll(0.0), pitch(0.0), yaw(0.0), throttle(0.0), voltage(0.0),

	// Are motors animated?
	motors(false)

{}

// Animate a motor without any dynamics
void Propulsion::AnimateMotors(bool enabled)
{
	// Create motor animations
	for (int i = 0; i < 4; i++)
	{
		// Assemble a motor and name for the naimation
		std::string motor = (std::string) "motor" + boost::lexical_cast<std::string>(i);
		std::string name  = (std::string) "rpm"   + boost::lexical_cast<std::string>(i);

		// Select a random offset, so that the quads dont all align
		double offset = gazebo::math::Rand::GetDblUniform(-MATH_PI,MATH_PI);

		// Create the animation
		std::map<std::string,gazebo::common::NumericAnimationPtr> anim;
		anim[motor].reset(new gazebo::common::NumericAnimation(name, 60.0/MOTOR_ANIMATION_ON_RPM, true));
		if (enabled)
		{
			gazebo::common::NumericKeyFrame *key;
			for (int i = 0; i <= 4; i++)
			{ 
				key = anim[motor]->CreateKeyFrame((double)i*15.0/MOTOR_ANIMATION_ON_RPM);
				key->SetValue((double)i*1.57079632679+offset);
			}
		}

		// Attache the animatin to the the model
		modPtr->SetJointAnimation(anim);
	}

	// Store whther the motors are on or off
	motors = enabled;
}

// Required override 
void Propulsion::Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr model) 
{
	// save the model pointer
	modPtr      = model;

	// Control parameters
	srs 		= GetSDFDouble(root,"control.roll.scale",srs);
	srl 		= GetSDFDouble(root,"control.roll.min",srl);
	sru 		= GetSDFDouble(root,"control.roll.max",sru);
	sps 		= GetSDFDouble(root,"control.pitch.scale",sps);
	spl 		= GetSDFDouble(root,"control.pitch.min",spl);
	spu 		= GetSDFDouble(root,"control.pitch.max",spu);
	sys 		= GetSDFDouble(root,"control.yaw.scale",sys);
	syl 		= GetSDFDouble(root,"control.yaw.min",syl);
	syu 		= GetSDFDouble(root,"control.yaw.max",syu);
	sts 		= GetSDFDouble(root,"control.throttle.scale",sts);
	stl 		= GetSDFDouble(root,"control.throttle.min",stl);
	stu 		= GetSDFDouble(root,"control.throttle.max",stu);
	svs 		= GetSDFDouble(root,"control.voltage.scale",svs);
	svl 		= GetSDFDouble(root,"control.voltage.min",svl);
	svu 		= GetSDFDouble(root,"control.voltage.max",svu);

	// Dynamics parameters
	_LOW_THROTT = GetSDFDouble(root, "dynamics.low_throttle", _LOW_THROTT);
	_MAX_ANGVEL = GetSDFDouble(root, "dynamics.max_angvel", _MAX_ANGVEL);
	_pq0        = GetSDFDouble(root, "dynamics.pq0", _pq0);
	_pq1        = GetSDFDouble(root, "dynamics.pq1", _pq1);
	_pq2        = GetSDFDouble(root, "dynamics.pq2", _pq2);
	_r0         = GetSDFDouble(root, "dynamics.r0", _r0);
	_r1         = GetSDFDouble(root, "dynamics.r1", _r1);
	_Cth0       = GetSDFDouble(root, "dynamics.Cth0", _Cth0);
	_Cth1       = GetSDFDouble(root, "dynamics.Cth1", _Cth1);
	_Cth2       = GetSDFDouble(root, "dynamics.Cth2", _Cth2);
	_Cvb0       = GetSDFDouble(root, "dynamics.Cvb0", _Cvb0);
	_Cvb1       = GetSDFDouble(root, "dynamics.Cvb1", _Cvb1);
	_tau0       = GetSDFDouble(root, "dynamics.tau0", _tau0);
	_tau1       = GetSDFDouble(root, "dynamics.tau1", _tau1);
	_kuv        = GetSDFDouble(root, "dynamics.kuv", _kuv);
	_kw         = GetSDFDouble(root, "dynamics.kw", _kw);

	// PRECOMPUTE SOME VERY USEFUL NUMBERS //////////////////////

	// Get the pose of the model (not the link!)
	pose = modPtr->GetWorldPose();

	// Set the drag constant vector for the platform
	drag.Set(_kuv, _kuv, _kw);
	drag *= modPtr->GetLink("body")->GetInertial()->GetMass();

	// How much thrust force is required to hover (simple F = mG)
	hover = modPtr->GetLink("body")->GetInertial()->GetMass() 
		  * modPtr->GetWorld()->GetPhysicsEngine()->GetGravity().GetLength();

	// Always call a reset 
	Reset();
}

// Reset the internal control
void Propulsion::SetControl(const double &r,const double &p,const double &y,const double &t,const double &v)
{
	roll 		= srs * limit(r,srl,sru);
	pitch 		= sps * limit(p,spl,spu);
	yaw 		= sys * limit(y,syl,syu);
	throttle 	= sts * limit(t,stl,stu);
	voltage 	= svs * limit(v,svl,svu);
}

// Update the system dynamics
void Propulsion::Update(const double &dt)
{
	// GET ORIENTATION AND ANGULAR VELOCITY //////////////////////////////////

	q = modPtr->GetLink("body")->GetWorldPose().rot;
	o = modPtr->GetLink("body")->GetRelativeAngularVel();

	// Update thrust force ////////////////////////////////////////////////////

	dFth = (_Cth0 + _Cth1*throttle + _Cth2*throttle*throttle) - thrust;
	if (throttle < _LOW_THROTT)
		dFth = _tau0 * dFth;
	else
	{
		tau  = 0.0;
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
	thrust += dFth*dt;

	// OBTAIN FORCE AND TORQUE INDUCED BY ROTORS ///////////////////////////////

	// Calculate force
	force = gazebo::math::Vector3(0.0,0.0,thrust) + drag*(modPtr->GetLink("body")->GetRelativeLinearVel());

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

	// UPDATE PHYSICS ///////////////////////////////////////////////////////////

  	// set force and torque in gazebo
  	modPtr->GetLink("body")->AddRelativeForce(force);
  	modPtr->GetLink("body")->AddRelativeTorque(torque);

	// UPDATE MOTOR ANIMATION ///////////////////////////////////////////////////

  	// Only update the on-off action of motors
  	if ((!motors && thrust > MOTOR_ANIMATION_THRESHOLD) || (motors && thrust < MOTOR_ANIMATION_THRESHOLD)) 
		AnimateMotors(thrust > MOTOR_ANIMATION_THRESHOLD);
}

// Get the current thurst force
double Propulsion::GetThrust()
{
	return thrust;
}

// Reset the class
void Propulsion::Reset()
{
	// Reset the pose of the model (not the link!)
	modPtr->SetWorldPose(pose);

	// If the platform has taken off, the initial thrust should be set to hover
	thrust = (pose.pos.z > 0 ? hover : 0);

	// Animate the motors to 300 rpm if hovering
	AnimateMotors(pose.pos.z > 0);
}
