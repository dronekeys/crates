//  Boost includes
#include "gnss.h"

// Standa libraries
#include <vector>

using namespace uas_controller;

// Default constructor - basically gives a standard GPS L1 (code) L2 (carrier), and Glonass L1 (CODE)
// L2 (code) receiver. Possible other options are "NONE", "CODE", "CARRIER" and "MILITARY"
GNSS::GNSS() : uas_hal::Position("gnss"),
	gps_L1("CODE"), gps_L2("NONE"), gps_eph(false), gps_clk(false), gps_tro(false), gps_ion(false),
	glo_L1("CODE"), glo_L2("CODE"), glo_eph(false), glo_clk(false), glo_tro(false), glo_ion(false)
{}

// REQUIRED METHODS

// All sensors must be configured using the current model information and the SDF
void GNSS::Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr model)
{
	// Save the model pointer
	modPtr = model;

	// We are going to need a tropospheric model, but we're handinlin
   	tropModel = &tropModelZero;

	// GPS Parameters
	gps_L1  = GetSDFString(root,"gps.frequencies.L1",gps_L1);		// Use the GPS L1 frequency
	gps_L2  = GetSDFString(root,"gps.frequencies.L2",gps_L2);		// Use the GPS L2 frequency
	gps_clk = GetSDFBool(root,"gps.correction.clk",gps_clk);		// Correct for clock error
	gps_rel = GetSDFBool(root,"gps.correction.rel",gps_rel);		// Correct for relativistic effects
	gps_rot = GetSDFBool(root,"gps.correction.rot",gps_rot);		// Correct for eath's rotation
	gps_eph = GetSDFBool(root,"gps.correction.eph",gps_eph);		// Correct for eph error
	gps_tro = GetSDFBool(root,"gps.correction.tro",gps_tro);		// Correct for tro error
	gps_ion = GetSDFBool(root,"gps.correction.ion",gps_ion);		// Correct for ion error

	// GLONASS Parameters
	glo_L1  = GetSDFString(root,"glonass.frequencies.L1",glo_L1);	// Use the GPS L1 frequency
	glo_L2  = GetSDFString(root,"glonass.frequencies.L2",glo_L2);	// Use the GPS L2 frequency
	glo_clk = GetSDFBool(root,"glonass.correction.clk",glo_clk);	// Correct for clock error
	glo_rel = GetSDFBool(root,"glonass.correction.rel",glo_rel);	// Correct for relativistic effects
	glo_rot = GetSDFBool(root,"glonass.correction.rot",glo_rot);	// Correct for eath's rotation
	glo_eph = GetSDFBool(root,"glonass.correction.eph",glo_eph);	// Correct for eph error
	glo_tro = GetSDFBool(root,"glonass.correction.tro",glo_tro);	// Correct for tro error
	glo_ion = GetSDFBool(root,"glonass.correction.ion",glo_ion);	// Correct for ion error
}

// All sensors must be resettable
void GNSS::Reset()
{

}

// EXTRA METHODS

// Set the  GNSS navigation solution from gps and glonass ephemerides. These actually
// also contain the ephemeride error, tropospheric dry and wet delays, clock error and
// ionospherid delay, based on the satellite elevation and short experiment baseline.
void GNSS::SetNavigationSolution(EnvironmentPtr env)
{
	// Reset the navigation solution
	Reset();

	/*
	// Get the actual pseudoranges to each satellite
	vector<SatID> 		satellites;
	vector<double> 		pseudoranges;

	// Get the ECEF position of the receiver
	gazebo::math::Vector3 msPositionLocal = modPtr->GetLink("body")->GetWorldPose().pos;

	// COnvert this position to geodeteic WGS84 coordinates
	gazebo::math::Vector3 msPositionGlobal = 
		modPtr()->GetWorld()->GetSphericalCoordinates().GlobalFromLocal(msPositionLocal);

	// Convert to a gpstk position type
	Position msPositionGeodetic(msPositionGlobal.x, msPositionGlobal.y, msPositionGlobal.z, &wgs84);

	// COnvert to ECEF coordinates
	Position msPositionECEF = msPositionGeodetic.asECEF();

	// Get the geodetic coordinates for this position, based on Gazebo's sperical model
	for (int i = 0; i < env->gps_size(); i++)
	{
		// TRUE RANGE////////////////////////////////////////////////////////////////////////

		// Get the ECEF position of the satellite 
		Position svPositionECEF(
			env->gps(i)->pos().x(),
			env->gps(i)->pos().y(),
			env->gps(i)->pos().z()
		);

		// This is the true range
		double range = (svPositionECEF - msPositionECEF).mag();

		// I

		// L1 ERROR /////////////////////////////////////////////////////////////////////////



		// L2 ERROR /////////////////////////////////////////////////////////////////////////


		// If we have a dual-frequency receiver, then the ionospheric delay correction
		// comes for free, because the delay is frequency-dependent. However, there 
		// may be a very small quantity of error in the correction, represented here.
		else
			pseudorange += env->gps(i)->env->gps(i);

		// Work out the range to satellite
		pseudoranges.push_back(pseudorange);
	}

	// Map this position to geodetic coordinates, given the spherical world model
	CommonTime          timestamp;
	vector<SatID> 		satellites;
	vector<double> 		pseudoranges;
	XvtStore<SatID> 	ephemerides;
	
	// Calculate a position solution - Return values: 
	// ( 2) solution is found, but it is not good (RMS residual exceed limits) 
	// ( 1) solution is found, but it is suspect (slope is large) 
	// ( 0) ok
	// (-1) algorithm failed to converge 
	// (-2) singular problem, no solution is possible 
	// (-3) not enough good data (> 4) to form a (RAIM) solution (check isValid()) 
	// (-4) ephemeris is not found for one or more satellites 
	status = solver.RAIMCompute( 
		timestamp,
	    satellites,
	    pseudoranges,
	    ephemerides,
	    tropModel
	);
	(*/
}


// Set the pressure and height at ground level
gazebo::math::Vector3 GNSS::GetPosition()
{
	return modPtr->GetLink("body")->GetWorldPose().pos;
}

// Set the pressure and height at ground level
gazebo::math::Vector3 GNSS::GetVelocity()
{
	return modPtr->GetLink("body")->GetWorldLinearVel();
}
