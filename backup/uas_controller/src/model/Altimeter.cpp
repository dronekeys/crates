//  Boost includes
#include "Altimeter.h"

using namespace uas_controller;

// Constructor
Altimeter::Altimeter() : t0(300.0), p0(1000), h0(90.0) {}

// All sensors must be configured using the current model information and the SDF
void Altimeter::Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr model)
{
	// save the model pointer
	modPtr = model;

	// Parameters
	t0 = GetSDFDouble(root,"temperature",t0);		// Temperature (K) @ ground
	p0 = GetSDFDouble(root,"pressure",p0);		// Pressure (hPa) @ ground
	h0 = GetSDFDouble(root,"humidity",h0);		// Rel. humidity (%) @ ground
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
	t0 = temperature;
	p0 = pressure;
	h0 = humidity;
}

// Get the altitude
double Altimeter::GetPressure()
{
	// Current altitude
	double h = modPtr->GetLink("body")->GetWorldPose().pos.z;

	// Gravitational field strength
	double g = modPtr->GetWorld()->GetPhysicsEngine()->GetGravity().GetLength();

	// Gas constant
	double R = 8.314472;

	// Standard molar mass of air
	double M = 0.0289644;

	// Dry adiabatic or saturated adiabatic lapse rates for air (Kelvin/m)
	// See: http://en.wikipedia.org/wiki/Troposphere
	double L = (h0 < 100.0 ? 0.00028295: 0.00027965);

	// Directly from http://en.wikipedia.org/wiki/Atmospheric_pressure
	return p0 * pow(1.0 - L*h/t0, (g-M) / (R-L));
}

// Get the altitude
double Altimeter::GetAltitude()
{
	return modPtr->GetLink("body")->GetWorldPose().pos.z;
}