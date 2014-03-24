/* This gazebo model plugin implements a second order model for quadrotor dynamics */ 
#include "dynamics.h"

using namespace uas_controller;

// Default constructor
Dynamics::Dynamics() : 
	mscale(1000.0),						// Motor speed scaling
	thrust(16.464000),					// Default for hovering!
	_LOW_THROTT(300.0),
	_MAX_ANGVEL(2.617993877991494),
	_pq0(-3.25060e-04),
	_pq1(1.79797e+02),
	_pq2(-24.3536),
	_r0(-4.81783e-03),
	_r1(-5.08944),
	_Cth0(6.63881e-01),
	_Cth1(7.44649e-04),
	_Cth2(2.39855e-06),
	_Cvb0(-18.0007),
	_Cvb1(4.23754),
	_tau0(3.07321),
	_tau1(46.8004),
	_kuv(-4.97391e-01),
	_kw(-1.35341)
{}

// Animate a motor
void Dynamics::AnimateMotor(gazebo::physics::ModelPtr& model, 
	const char *name, const char* motor, double rpm)
{
	std::map<std::string,gazebo::common::NumericAnimationPtr> anim;
	anim[motor].reset(new gazebo::common::NumericAnimation(name, 60.0/rpm, true));
	gazebo::common::NumericKeyFrame *key;
	for (int i = 0; i <= 4; i++)
	{ 
		key = anim[motor]->CreateKeyFrame((double)i*15.0/rpm);
		key->SetValue((double)i*1.57079632679);
	}
	model->SetJointAnimation(anim);
}


void Dynamics::Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr& model) 
{

	/*****************************************

    <dynamics>
        <parameters>
          <low_throttle>300</low_throttle>
          <max_angvel>2.617993877991494</max_angvel>
          <pq0>-3.25060e-04</pq0>
          <pq1>1.79797e+02</pq1>
          <pq2>-24.3536</pq2>
          <r0>-4.81783e-03</r0>
          <r1>-5.08944</r1>
          <Cth0>6.63881e-01</Cth0>
          <Cth1>7.44649e-04</Cth1>
          <Cth2>2.39855e-06</Cth2>
          <Cvb0>-18.0007</Cvb0>
          <Cvb1>4.23754</Cvb1>
          <tau0>3.07321</tau0>
          <tau1>46.8004</tau1>
          <kuv>-4.97391e-01</kuv>
          <kw>-1.35341</kw>
        </parameters>
        <noise>
          <force>
            <x><gaussian>0.0 0.2</gaussian></x>
            <y><gaussian>0.0 0.2</gaussian></y>
            <z><gaussian>0.0 0.2</gaussian></z>
          </force>
          <torque>
            <x><gaussian>0.0 0.2</gaussian></x>
            <y><gaussian>0.0 0.2</gaussian></y>
            <z><gaussian>0.0 0.2</gaussian></z>
          </torque>
        </noise>
      </dynamics>

      *****************************************/

	_LOW_THROTT = GetSDFDouble(root, "dynamics.parameters.low_throttle", _LOW_THROTT);
	_MAX_ANGVEL = GetSDFDouble(root, "dynamics.parameters.max_angvel", _MAX_ANGVEL);
	_pq0        = GetSDFDouble(root, "dynamics.parameters.pq0", _pq0);
	_pq1        = GetSDFDouble(root, "dynamics.parameters.pq1", _pq1);
	_pq2        = GetSDFDouble(root, "dynamics.parameters.pq2", _pq2);
	_r0         = GetSDFDouble(root, "dynamics.parameters.r0", _r0);
	_r1         = GetSDFDouble(root, "dynamics.parameters.r1", _r1);
	_Cth0       = GetSDFDouble(root, "dynamics.parameters.Cth0", _Cth0);
	_Cth1       = GetSDFDouble(root, "dynamics.parameters.Cth1", _Cth1);
	_Cth2       = GetSDFDouble(root, "dynamics.parameters.Cth2", _Cth2);
	_Cvb0       = GetSDFDouble(root, "dynamics.parameters.Cvb0", _Cvb0);
	_Cvb1       = GetSDFDouble(root, "dynamics.parameters.Cvb1", _Cvb1);
	_tau0       = GetSDFDouble(root, "dynamics.parameters.tau0", _tau0);
	_tau1       = GetSDFDouble(root, "dynamics.parameters.tau1", _tau1);
	_kuv        = GetSDFDouble(root, "dynamics.parameters.kuv", _kuv);
	_kw         = GetSDFDouble(root, "dynamics.parameters.kw", _kw);

	// Set the drag
	drag.Set(_kuv, _kuv, _kw);

	// Work out the mass of the platform
	mass = model->GetLink("body")  ->GetInertial()->GetMass();

	// If the platform has taken off
	thrust = 0;
	if (model->GetLink("body")->GetWorldPose().pos.z > 0)
		thrust = 16.464000; 

	// Animate the motors
	AnimateMotor(model, "rpm0", "motor0", 300);
	AnimateMotor(model, "rpm1", "motor1", 300);
	AnimateMotor(model, "rpm2", "motor2", 300);
	AnimateMotor(model, "rpm3", "motor3", 300);
};

// Update the system dynamics
void Dynamics::Update(
            const gazebo::physics::ModelPtr& model,             // Pointer to physics element                          
            const gazebo::math::Vector3& wind,					// Wind force
            const double& roll,                                 // RC roll
            const double& pitch,                                // RC pitch
            const double& yaw,                                  // RC yaw
            const double& throttle,                             // RC throttle
            const double& voltage,                              // RC voltage
            const double& dt)  	                                // Time
{
	///////////////////////////////////////////
	// Get the current state of the platform //
	///////////////////////////////////////////

	q    	  = model->GetLink("body")->GetWorldPose().rot;
	b_lin_vel = model->GetLink("body")->GetRelativeLinearVel();
	b_ang_vel = model->GetLink("body")->GetRelativeAngularVel();

	/////////////////////////
	// Update thrust force //
	/////////////////////////

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

	///////////////////////////////////////////////
	// OBTAIN FORCE AND TORQUE INDUCED BY ROTORS //
	///////////////////////////////////////////////

	// Calculate force
	force = gazebo::math::Vector3(0.0,0.0,thrust) 
	      + mass*drag*(b_lin_vel-q.RotateVector(wind));

	// x = roll
	torque.x = _pq1*(_pq0*roll  - q.GetAsEuler().x) + _pq2*b_ang_vel.x;    
	if ((b_ang_vel.x > _MAX_ANGVEL && (torque.x > 0)) || (b_ang_vel.x < -_MAX_ANGVEL && (torque.x < 0)))
	  torque.x = 0;

	// y = pitch
	torque.y = _pq1*(_pq0*pitch - q.GetAsEuler().y) + _pq2*b_ang_vel.y;    
	if ((b_ang_vel.y > _MAX_ANGVEL && (torque.y > 0)) || (b_ang_vel.y < -_MAX_ANGVEL && (torque.y < 0)))
	  torque.y = 0;

	// Yaw
	torque.z = _r0*yaw + _r1*b_ang_vel.z;

	////////////////////
	// UPDATE PHYSICS //
	////////////////////

  	// set force and torque in gazebo
  	model->GetLink("body")->AddRelativeForce(force);
  	model->GetLink("body")->AddRelativeTorque(torque);
}

// Reset the component
double Dynamics::GetThrust()
{
	return thrust;
}
