/* This gazebo model plugin implements a second order model for quadrotor dynamics */ 
#include "dynamics.h"

using namespace uas_controller;

// Default constructor
Dynamics::Dynamics(sdf::ElementPtr _sdf, gazebo::physics::LinkPtr _lp) 
	: linkPtr(lp)
{
  _LOW_THROTT = GetSDFDouble(_sdf, "low_throttle",  300.0);
  _MAX_ANGVEL = GetSDFDouble(_sdf, "max_angvel",    2.617993877991494);
  _pq0        = GetSDFDouble(_sdf, "pq0",          -3.25060e-04);
  _pq1        = GetSDFDouble(_sdf, "pq1",           1.79797e+02);
  _pq2        = GetSDFDouble(_sdf, "pq2",          -24.3536);
  _r0         = GetSDFDouble(_sdf, "r0",           -4.81783e-03);
  _r1         = GetSDFDouble(_sdf, "r1",           -5.08944);
  _Cth0       = GetSDFDouble(_sdf, "Cth0",          6.63881e-01);
  _Cth1       = GetSDFDouble(_sdf, "Cth1",          7.44649e-04);
  _Cth2       = GetSDFDouble(_sdf, "Cth2",          2.39855e-06);
  _Cvb0       = GetSDFDouble(_sdf, "Cvb0",         -18.0007);
  _Cvb1       = GetSDFDouble(_sdf, "Cvb1",          4.23754);
  _tau0       = GetSDFDouble(_sdf, "tau0",          3.07321);
  _tau1       = GetSDFDouble(_sdf, "tau1",          46.8004);
  _kuv        = GetSDFDouble(_sdf, "kuv",          -4.97391e-01);
  _kw         = GetSDFDouble(_sdf, "kw",           -1.35341);
};

// Update the system dynamics
Dynamics::Update(const double &dt)
{
	// Get the b-frame linear velocity
	b_vel = modPtr->GetWorldPose().rot.GetInverse().RotateVector(
	model->GetWorldLinearVel()
	);

	// Get the b-frame angular velocity
	b_ang = modPtr->GetWorldPose().rot.GetInverse().RotateVector(
	model->GetWorldAngularVel()
	);

	//////////////////////////
	// Get the wind values  //
	//////////////////////////

	// Wind = shear(altitude) + turbulence(altitude,speed,dt)
	b_wnd = shear.GetGlobalVelocity(n_pos.z)
	    + turbulence.GetGlobalVelocity(n_pos.z,b_vel.GetLength(),dt);

	// Navigation frame -> Body frame
	b_wnd = modPtr->GetWorldPose().rot.RotateVector(b_wnd);

	////////////////////////////////////////////
	// Calculate b-frame angular acceleration //
	////////////////////////////////////////////

	// X torque
	b_tor.x = _pq1*(_pq0*ctl.r - n_rot.x) + _pq2*b_ang.x;    
	if ((b_ang.x > _MAX_ANGVEL && (b_tor.x > 0)) || (b_ang.x < -_MAX_ANGVEL && (b_tor.x < 0)))
	  b_tor.x = 0;

	// Y torque
	b_tor.y = _pq1*(_pq0*ctl.p - n_rot.y) + _pq2*b_ang.y;    
	if ((b_ang.y > _MAX_ANGVEL && (b_tor.y > 0)) || (b_ang.y < -_MAX_ANGVEL && (b_tor.y < 0)))
	  b_tor.y = 0;

	// Z torque
	b_tor.z = _r0*ctl.y + _r1*b_ang.z;

	////////////////////////////
	// Calculate thrust force //
	////////////////////////////

	dFth = ((_Cth0 + _Cth1*ctl.t + _Cth2*ctl.t*ctl.t) - thrust);
	if (ctl.t < _LOW_THROTT)
		dFth = _tau0 * dFth;
	else
	{
		tau  = 0;
		if (abs(dFth) < (_tau1*dt))
		  tau = dFth / dt;
		else
		  tau = (dFth > 0 ? _tau1 : -_tau1);

		if ((thrust + tau*dt) > (_Cvb0 + _Cvb1*ctl.v))
		  dFth = (_Cvb0 + _Cvb1*ctl.v - thrust) / dt;
		else
		  dFth = tau;
	}

	// Update the drag force
	b_for = drag * (b_vel + b_wnd);   // Drag force
	b_for.z = thrust + dFth;          // Thrust force

}