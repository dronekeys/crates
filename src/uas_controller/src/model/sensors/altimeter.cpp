//  Boost includes
#include "altimeter.h"

#define GAS_CONSTANT 		8.314472
#define HPA_TO_PA    		100.00
#define MOLAR_MASS_DRYAIR	28.964
#define MOLAR_MASS_WATER	18.016

using namespace uas_controller;

// Constructor
Altimeter::Altimeter() : uas_hal::Altitude("altimeter"),  t(300.0), p(1000), h(90.0) {}

// All sensors must be configured using the current model information and the SDF
void Altimeter::Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr model)
{
	// save the model pointer
	modPtr = model;

	// Parameters
	t = GetSDFDouble(root,"temperature",t);		// Temperature (K) @ ground
	p = GetSDFDouble(root,"pressure",p);		// Pressure (hPa) @ ground
	h = GetSDFDouble(root,"humidity",h);		// Rel. humidity (%) @ ground
}

// All sensors must be resettable
void Altimeter::Reset()
{

}

// EXTRA METHODS

// Set the pressure and height at ground level
void Altimeter::SetMeteorological(const double &temperature, const double &pressure, const double &humidity)
{
	// Set the ground variables
	t = temperature;
	p = pressure;
	h = humidity;
}

// Get the altitude
double Altimeter::GetAltitude()
{
	// Get the true altitude
	return modPtr->GetLink("body")->GetWorldPose().pos.z;

	/*
	// Dalton's law: the total pressure exerted by a mixture is the sum of the partial pressures
	// See http://en.wikipedia.org/wiki/Density_of_air for more information


	// Calculate the saturation vapour pressure
	double p_s = 610.78*exp(17.67*t/(243.5+t)); 

	// Calculate the actual vapur pressure
	double p_v = h / 100.0 * p_s;

	// Density of the air, based on the relative humidity
	d = ((101325.0 - p_v) * MOLAR_MASS_DRYAIR + p_v * MOLAR_MASS_WATER) / (1000.0 * GAS_CONSTANT * t);

	// Get the true altitude
	a = modPtr->GetLink("body")->GetWorldPose().pos.z;

	// Work out the gravitational field strength
	g = modPtr->GetWorl()->GetPhysicsEngine()->GetGravity()->GetLength();

	// Molar mass

	// See: http://www.hills-database.co.uk/altim.html#Appendix1
	return exp(ln(p)/(g*d) - a);
	*/
}
