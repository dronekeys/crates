/* This gazebo model plugin implements a second order model for quadrotor dynamics */ 
#include "dynamics.h"

using namespace uas_controller;

// Default constructor
Dynamics::Dynamics() : 
	thrust(0),							// Current thrust
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

void Dynamics::Configure(sdf::ElementPtr root) 
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
};

// Update the system dynamics
void Dynamics::Update(
            const gazebo::physics::LinkPtr& link,               // Pointer to physics element                          
            const gazebo::math::Vector3& wind,					// Wind force
            const double& pitch,                                // RC pitch
            const double& roll,                                 // RC roll
            const double& yaw,                                  // RC yaw
            const double& throttle,                             // RC throttle
            const double& voltage,                              // RC voltage
            const double& dt)  	                                // Time
{
	// Get the world-frame position and orientation
	n_rot = link->GetWorldPose().rot.GetAsEuler();
	b_lin_vel = link->GetRelativeLinearVel();
	b_ang_vel = link->GetRelativeAngularVel();

	/////////////////////////////////////////
	// Update b-frame angular acceleration //
	/////////////////////////////////////////

	// Pitch
	torq.x = _pq1*(_pq0*roll - n_rot.x) + _pq2*b_ang_vel.x;    
	if ((b_ang_vel.x > _MAX_ANGVEL && (torq.x > 0)) || (b_ang_vel.x < -_MAX_ANGVEL && (torq.x < 0)))
	  torq.x = 0;

	// Roll
	torq.y = _pq1*(_pq0*pitch  - n_rot.y) + _pq2*b_ang_vel.y;    
	if ((b_ang_vel.y > _MAX_ANGVEL && (torq.y > 0)) || (b_ang_vel.y < -_MAX_ANGVEL && (torq.y < 0)))
	  torq.y = 0;

	// Yaw
	torq.z = _r0*yaw + _r1*b_ang_vel.z;

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
	
	// Update thrust force (link below should idle the quadrotor)
	thrust = dFth + thrust;
	
	//thrust = -link->GetInertial()->GetMass() 
	//	      * link->GetModel()->GetWorld()->GetPhysicsEngine()->GetGravity().z;

	// Force is always orthogonal to rotor plane
	forc = gazebo::math::Vector3(0.0,0.0,throttle);
	
	// Drag is proportional to airspeed and wind
	drag  = link->GetRelativeLinearVel();
	drag -= link->GetWorldPose().rot.RotateVector(wind);

	// Convert from a force to a mass
	drag.x *= link->GetInertial()->GetMass() *_kuv;
	drag.y *= link->GetInertial()->GetMass() *_kuv;
	drag.z *= link->GetInertial()->GetMass() *_kw;

	// set force and torque in gazebo
	//link->AddRelativeForce(forc + drag);
	//link->AddRelativeTorque(torq);
}

// Reset the component
double Dynamics::GetThrust()
{
	return thrust;
}
