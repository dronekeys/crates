//  Boost includes
#include "gnss.h"

// STandar dincludes
#include <vector>

// For logging
#include <ros/ros.h>

// This does the GPS heavy lifting (pseudorange solver)
#include "PRSolutionNoRotation.hpp"		// For solving 
#include <gpstk/GNSSconstants.hpp>  // For the speed of light
#include <gpstk/WGS84Ellipsoid.hpp> // GPS ellipsoid
#include <gpstk/TropModel.hpp>      // Tropospheric model
#include <gpstk/GPSEllipsoid.hpp>	// Ellipsoid model

using namespace std;
using namespace gpstk;
using namespace uas_controller;

// Default constructor - basically gives a standard GPS L1 (code) L2 (carrier), and Glonass L1 (CODE)
// L2 (code) receiver. Possible other options are "NONE", "CODE", "CARRIER" and "MILITARY"
GNSS::GNSS() : uas_hal::Position("gnss"),
	gps_L1(true), gps_L2(true), gps_eph(false), gps_tro(false), gps_ion(false),
	glo_L1(true), glo_L2(true), glo_eph(false), glo_tro(false), glo_ion(false)
{}

// REQUIRED METHODS

// All sensors must be configured using the current model information and the SDF
void GNSS::Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr model)
{
	// Save the model pointer
	modPtr = model;

	// GPS Parameters
	gps_L1  = GetSDFBool(root,"gps.frequencies.civ",gps_L1);		// Use the GPS L1 frequency
	gps_L2  = GetSDFBool(root,"gps.frequencies.mil",gps_L2);		// Use the GPS L2 frequency
	gps_eph = GetSDFBool(root,"gps.correction.eph",gps_eph);		// Correct for eph error
	gps_tro = GetSDFBool(root,"gps.correction.tro",gps_tro);		// Correct for tro error
	gps_ion = GetSDFBool(root,"gps.correction.ion",gps_ion);		// Correct for ion error

	// GLONASS Parameters
	glo_L1  = GetSDFBool(root,"glonass.frequencies.civ",glo_L1);	// Use the GPS L1 frequency
	glo_L2  = GetSDFBool(root,"glonass.frequencies.mil",glo_L2);	// Use the GPS L2 frequency
	glo_eph = GetSDFBool(root,"glonass.correction.eph",gps_eph);	// Correct for eph error
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
	// Create a timestamp from the epoch encoded in the environment message
	CommonTime currentTime;
	currentTime.set((long)env->epoch().days(),(double)env->epoch().secondsofdays(),(TimeSystem)TimeSystem::UTC);

	// Determine the ECEF position of the receiver
    gazebo::math::Vector3 msPositionGlobal = modPtr->GetWorld()->GetSphericalCoordinates()
    	->SphericalFromLocal(modPtr->GetLink("body")->GetWorldPose().pos);
      
    // Convert the position from geotetic spherical to geocentric cartesian
    WGS84Ellipsoid wgs84;
  	gpstk::Position msPosGeodetic    = gpstk::Position(
  		msPositionGlobal.x, msPositionGlobal.y, msPositionGlobal.z, gpstk::Position::Geodetic, &wgs84);
	gpstk::Position msPosGeocentric  = msPosGeodetic.transformTo(gpstk::Position::Geocentric);
    gpstk::Position msPosECEF        = msPosGeocentric.asECEF();

    // Obtain the satellites, pseudoranges and ephemerides
	vector<SatID::SatelliteSystem>  systems;
	vector<SatID> 					satellites;
	Matrix<double>					covariance(0,0);	
	Matrix<double>					ephemerides(env->gnss_size(),4,0.0);

	// We will be using two GNSS systems in total
	systems.push_back(SatID::systemGPS);
	systems.push_back(SatID::systemGlonass);

	ROS_INFO("Number of satellites in view is %d ",env->gnss_size());

    // Create a store f the
	for (int i = 0; i < env->gnss_size(); i++)
	{
		// Work out the range
		try
		{
			// Satellite ID and system
			SatID sid(env->gnss(i).prn(),(SatID::SatelliteSystem)env->gnss(i).system());

			// POSITION AND ERROR ESTIMATION /////////////////////////////////

			// Get the true position
			gpstk::Position svPosECEF = gpstk::Position(
				env->gnss(i).pos().x(),
				env->gnss(i).pos().y(),
				env->gnss(i).pos().z(), 
				gpstk::Position::Cartesian
			);

			// Get the boradcast position
			gpstk::Position erPosECEF = gpstk::Position(
				env->gnss(i).pos().x() + env->gnss(i).err().x(),
				env->gnss(i).pos().y() + env->gnss(i).err().y(),
				env->gnss(i).pos().z() + env->gnss(i).err().z()
			);

			// GET THE RANGE AND ERROR ESTIMATION ////////////////////////////////

			// Get the true range
			double trurange = range(msPosECEF, svPosECEF);
			double errEphem = range(msPosECEF, erPosECEF) - trurange;

			// Now calculate the new pseudorange
			double pseudorange = trurange;

			// PSEUDORANGE PERTURBATION BASED ON SYSTEM AND FREQUENCY ///////////

			// Based on the frequency, decide how much ionospheric error
			switch (env->gnss(i).system())
			{
			case SatID::systemGPS:
				if (!gps_ion)
				{
					if (gps_L1 && !gps_L2)
						pseudorange += env->gnss(i).delay_iono_f1();
					if (!gps_L1 && gps_L2)
						pseudorange += env->gnss(i).delay_iono_f2();
				}
				if (!gps_tro) pseudorange += env->gnss(i).delay_trop();
				if (!gps_eph) pseudorange += errEphem;
				break;
				
			case SatID::systemGlonass:
				if (!glo_ion)
				{
					if (glo_L1 && !glo_L2)
						pseudorange += env->gnss(i).delay_iono_f1();
					if (!glo_L1 && glo_L2)
						pseudorange += env->gnss(i).delay_iono_f2();
				}
				if (!glo_tro) pseudorange += env->gnss(i).delay_trop();
				if (!glo_eph) pseudorange += errEphem;
				break;
			}
			
			// Summary
			ROS_INFO("-- Satellite %d (range %f) with eph err %f, clock err %f, and rel err %f", i, trurange, errEphem,
				C_MPS*env->gnss(i).clkbias(), C_MPS*env->gnss(i).relcorr());

			// DATA PREPARATION ////////////////////////////////////////////////
         	
			// Add satellite
			satellites.push_back(sid);

         	// Add ephemeride
         	ephemerides(i,0) = env->gnss(i).pos().x();
         	ephemerides(i,1) = env->gnss(i).pos().y();
         	ephemerides(i,2) = env->gnss(i).pos().z();
         	ephemerides(i,3) = pseudorange;
		}
		catch (Exception &e)
		{
			ROS_WARN("Processing exception : %s", e.what().c_str());
		}
	}

	ROS_INFO("Obtaining position solution");

  	try
  	{
		// Create a new RAIM solver
		PRSolutionNoRotation solver;

		// We want a once-off solution (segfaults if on)
		solver.hasMemory = false;
		solver.RMSLimit  = 3e6;

		// These wills tore the resifuals and slopes after convergence
	  	Vector<double> 	resids(env->gnss_size());
	  	Vector<double> 	slopes(env->gnss_size());
	 
	    // Tropospheric correction model (NULL)
        ZeroTropModel   tropModelZero;
        TropModel*      tropModel = &tropModelZero;

	  	// Perform solving
		int statusPOS =  solver.SimplePRSolution(
			currentTime,
			satellites,
			ephemerides,
			covariance,
			tropModel,
			100,
			3.e-7,
			systems,
			resids,
			slopes);
		int statusDOP = solver.DOPCompute();

		// Get the position
		gpstk::Position solPosECEF = gpstk::Position(
			solver.Solution[0],
			solver.Solution[1],
			solver.Solution[2]
		);

		// Get the error
		gpstk::Position errPosECEF = solPosECEF - msPosECEF;

		// Finished!
		ROS_WARN("Solver finished in %d iterations with SOL status %d and DOP status %d", solver.NIterations, statusPOS, statusDOP);
		ROS_WARN("-- POS: X:%f Y:%f Z:%f", solPosECEF.X(), solPosECEF.Y(), solPosECEF.Z());
		ROS_WARN("-- ERR: X:%f Y:%f Z:%f", errPosECEF.X(), errPosECEF.Y(), errPosECEF.Z());
		ROS_WARN("-- DOP: T:%f P:%f G:%f", solver.TDOP, solver.PDOP, solver.GDOP);
	}
	catch (Exception &e)
	{
		ROS_WARN("Solver exception : %s", e.what().c_str());
	}
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
