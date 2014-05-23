#include "Propulsion.h"

using namespace gazebo;


Propulsion::Propulsion() :

	// Control parameters
	_srs(-2291.83118052329), _srl(-0.9), _sru(0.9), 
    _sps(-2291.83118052329), _spl(-0.9), _spu(0.9), 
	_sys(-460.597254433196), _syl(-4.5), _syu(4.5), 
	_sts( 4097.0), 			 _stl(0.0),  _stu(1.0),

	// Dynamics parameters
	_LOW_THROTT(300.0), _MAX_ANGVEL(2.617993877991494),
	_pq0(-3.25060e-04), _pq1(1.79797e+02), 	_pq2(-24.3536),
	_r0(-4.81783e-03),  _r1(-5.08944),
	_Cth0(6.63881e-01), _Cth1(7.44649e-04), _Cth2(2.39855e-06),
	_Cvb0(-18.0007), 	_Cvb1(4.23754),
	_tau0(3.07321), 	_tau1(46.8004),

	// Current control
	thrust(0.0), roll(0.0), pitch(0.0), yaw(0.0), throttle(0.0), voltage(12.0)
{
	// Do nothing
}

// Clamp a value to a given range
double Propulsion::Clamp(const double& val, const double& minval, const double& maxval)
{
	if (val < minval) return minval;
	if (val > maxval) return maxval;
	return val;
}

// All sensors must be configured using the current model information and the SDF
bool Propulsion::Configure(physics::LinkPtr link, sdf::ElementPtr root)
{
	// Backup the link
	linkPtr = link;

	// Control parameters
	root->GetElement("control")->GetElement("roll")->GetElement("scale")->GetValue()->Get(_srs);
	root->GetElement("control")->GetElement("roll")->GetElement("min")->GetValue()->Get(_srl);
	root->GetElement("control")->GetElement("roll")->GetElement("max")->GetValue()->Get(_sru);
	root->GetElement("control")->GetElement("pitch")->GetElement("scale")->GetValue()->Get(_sps);
	root->GetElement("control")->GetElement("pitch")->GetElement("min")->GetValue()->Get(_spl);
	root->GetElement("control")->GetElement("pitch")->GetElement("max")->GetValue()->Get(_spu);
	root->GetElement("control")->GetElement("yaw")->GetElement("scale")->GetValue()->Get(_sys);
	root->GetElement("control")->GetElement("yaw")->GetElement("min")->GetValue()->Get(_syl);
	root->GetElement("control")->GetElement("yaw")->GetElement("max")->GetValue()->Get(_syu);
	root->GetElement("control")->GetElement("throttle")->GetElement("scale")->GetValue()->Get(_sts);
	root->GetElement("control")->GetElement("throttle")->GetElement("min")->GetValue()->Get(_stl);
	root->GetElement("control")->GetElement("throttle")->GetElement("max")->GetValue()->Get(_stu);

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

    // Setup the noise distribution
    nForce  = NoiseFactory::Create(root->GetElement("errors")->GetElement("force"));
	nTorque = NoiseFactory::Create(root->GetElement("errors")->GetElement("torque"));

    return true;
}

// All sensors must be resettable
void Propulsion::Reset()
{
	// Reset error streams
	nForce->Reset();
	nTorque->Reset();

	// Reset control
	roll 		= _srs * Clamp(0,_srl,_sru);
	pitch 		= _sps * Clamp(0,_spl,_spu);
	yaw 		= _sys * Clamp(0,_syl,_syu);
	throttle 	= _sts * Clamp(0,_stl,_stu);
	thrust = 0.0;
}

// Get the current altitude
void Propulsion::Update(double dt)
{
	// Get the orientation
	math::Quaternion q = linkPtr->GetWorldPose().rot;
	math::Vector3    o = linkPtr->GetRelativeAngularVel();

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
	math::Vector3 force  = math::Vector3(0.0,0.0,thrust);
	math::Vector3 torque = math::Vector3(
		_pq1*(_pq0*roll  - q.GetAsEuler().x) + _pq2*o.x,    
		_pq1*(_pq0*pitch - q.GetAsEuler().y) + _pq2*o.y, 
		_r0*yaw + _r1*o.z
	);

	// Enforcce some limits on the angular velocity
	if ((o.x > _MAX_ANGVEL && (torque.x > 0)) || (o.x < -_MAX_ANGVEL && (torque.x < 0)))
	  torque.x = 0;
	if ((o.y > _MAX_ANGVEL && (torque.y > 0)) || (o.y < -_MAX_ANGVEL && (torque.y < 0)))
	  torque.y = 0;

	// Add errors to the dynamics
	force  += (math::Vector3)  nForce->DrawVector(dt);
	torque += (math::Vector3) nTorque->DrawVector(dt);

  	// Apply force and torque
  	linkPtr->AddRelativeForce(force);
  	linkPtr->AddRelativeTorque(torque + linkPtr->GetInertial()->GetCoG().Cross(force));
}

// Get the remaining energy (mAh)
double Propulsion::GetThrustForce()
{
	return thrust;
}

// Set the remaining energy (mAh)
void Propulsion::SetThrustForce(double val)
{
	thrust = val;
} 

// Copy in some control 
void Propulsion::SetControl(const hal_quadrotor::Control& control)
{
	roll 		= _srs * Clamp(control.roll,	_srl,_sru);
	pitch 		= _sps * Clamp(control.pitch,	_spl,_spu);
	yaw 		= _sys * Clamp(control.yaw,		_syl,_syu);
	throttle 	= _sts * Clamp(control.throttle,_stl,_stu);
}