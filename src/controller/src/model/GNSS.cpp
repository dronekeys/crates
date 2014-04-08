//  Boost includes
#include "GNSS.h"

// STandar dincludes
#include <vector>

// For logging
#include <ros/ros.h>

#define DEBUG TRUE

using namespace controller;

// Default constructor
GNSS::GNSS() : tropModel(&tropModelZero),
	gps_typ("L1"), gps_sig(0.0), gps_clk(0.0), gps_rel(0.0), gps_rot(0.0), gps_eph(0.0), gps_tro(0.0), gps_ion(0.0),
	glo_typ("DUAL"), glo_sig(0.0), glo_clk(0.0), glo_rel(0.0), glo_rot(0.0), glo_eph(0.0), glo_tro(0.0), glo_ion(0.0)
{}

// REQUIRED METHODS

// All sensors must be configured using the current model information and the SDF
void GNSS::Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr model)
{
	// Save the model pointer
	modPtr = model;

	// GPS Parameters
	gps_typ = GetSDFString(root,"gps.signal",gps_typ);
	gps_sig = GetSDFDouble(root,"gps.corrections.sig",gps_sig);
	gps_clk = GetSDFDouble(root,"gps.corrections.clk",gps_clk);
	gps_rel = GetSDFDouble(root,"gps.corrections.rel",gps_rel);
	gps_rot = GetSDFDouble(root,"gps.corrections.rot",gps_rot);
	gps_eph = GetSDFDouble(root,"gps.corrections.eph",gps_eph);
	gps_tro = GetSDFDouble(root,"gps.corrections.tro",gps_tro);
	gps_ion = GetSDFDouble(root,"gps.corrections.ion",gps_ion);

	// GLONASS Parameters
	glo_typ = GetSDFString(root,"glonass.signal",glo_typ);
	glo_sig = GetSDFDouble(root,"glonass.corrections.sig",glo_sig);
	glo_clk = GetSDFDouble(root,"glonass.corrections.clk",glo_clk);
	glo_rel = GetSDFDouble(root,"glonass.corrections.rel",glo_rel);
	glo_rot = GetSDFDouble(root,"glonass.corrections.rot",glo_rot);
	glo_eph = GetSDFDouble(root,"glonass.corrections.eph",glo_eph);
	glo_tro = GetSDFDouble(root,"glonass.corrections.tro",glo_tro);
	glo_ion = GetSDFDouble(root,"glonass.corrections.ion",glo_ion);

	// We will be using two GNSS systems in total
	systems.push_back(SatID::systemGPS);
	systems.push_back(SatID::systemGlonass);

	// Issue a reset on load
	Reset();
}

// All sensors must be resettable
void GNSS::Reset()
{
	// Reset the solver state
	solver = PRSolutionNoRotation();
}

// EXTRA METHODS


// Set the  GNSS navigation solution from gps and glonass ephemerides. These actually
// also contain the ephemeride error, tropospheric dry and wet delays, clock error and
// ionospherid delay, based on the satellite elevation and short experiment baseline.
void GNSS::SetNavigationSolution(EnvironmentPtr env)
{
	// BACKUP THE LAST POSITION AND TIME FOR VELOCITY CALCULATION /////////////////////

	oldTime = currentTime;
	oldSolution = gpstk::Position(
		solver.Solution[0],
		solver.Solution[1],
		solver.Solution[2]
	);

	// DETERMINE THE CURRENT TIME ////////////////////////////////////////////////////

	currentTime.set((long)env->epoch().days(),(double)env->epoch().secondsofdays(),(TimeSystem)TimeSystem::UTC);

	// DETERMINE THE CURRENT POSITION ////////////////////////////////////////////////

    msPositionGlobal = modPtr->GetWorld()->GetSphericalCoordinates()
    	->SphericalFromLocal(modPtr->GetLink("body")->GetWorldPose().pos);
      
    // Convert the position from geotetic spherical to geocentric cartesian
  	msPosGeodetic    = gpstk::Position(
  		msPositionGlobal.x, 
  		msPositionGlobal.y, 
  		msPositionGlobal.z, 
  		gpstk::Position::Geodetic, 
  		&wgs84
	);
	msPosGeocentric  = msPosGeodetic.transformTo(gpstk::Position::Geocentric);
    msPosECEF        = msPosGeocentric.asECEF();

    // Clear the satellite list
	satellites.clear();
	
	// Make the ephemerides container big enough to store all SV information
	ephemerides.resize(env->gnss_size(),4,0.0);

    // Count the useful satellites
    int counter = 0;

    // Create a store f the
	for (int i = 0; i < env->gnss_size(); i++)
	{
		// Work out the range
		try
		{
			// Satellite ID and system
			SatID sid(env->gnss(i).prn(),(SatID::SatelliteSystem)env->gnss(i).system());

			// Param mapping		
			bool rot, eph, tro, rel, ion, clk, sig;
			std::string F1, F2, typ;
			switch (env->gnss(i).system())
			{
			case ((int)SatID::systemGPS) :
				rot = gps_rot; eph = gps_eph; tro = gps_tro;
				rel = gps_rel; ion = gps_ion; clk = gps_clk;
				sig = gps_sig; typ = gps_typ;
				F1  = "L1"; F2  = "L2";
				break;

			case ((int)SatID::systemGlonass) :
				rot = glo_rot; eph = glo_eph; tro = glo_tro;
				rel = glo_rel; ion = glo_ion; clk = glo_clk;
				sig = glo_sig; typ = glo_typ;
				F1  = "L1O"; F2  = "L2O"; 
				break;
			}

			// DONT ADD A RECORD IF THE SYSTEM IS DISABLED ////////////////////

			if (typ.compare("DISABLED")==0)
				continue;

			// POSITION AND ERROR ESTIMATION /////////////////////////////////

			// Get the true position
			gpstk::Position svPosECEF = gpstk::Position(
				env->gnss(i).pos().x(),
				env->gnss(i).pos().y(),
				env->gnss(i).pos().z(), 
				gpstk::Position::Cartesian
			);

			// Get the true range
			double trurange = range(msPosECEF,svPosECEF);

			// Initialise the pseudorange
			double pseudorange = trurange;

			// DEAL WITH ROTATIONAL ERROR ////////////////////////////////////////

			if (rot < 0)
			{
               // True time of flight
               double rho = trurange / C_MPS;

               // cAngle change in radians
               double wt = ellip.angVelocity()*rho;
               
               // Update the satellite position to reflect error
               svPosECEF = gpstk::Position( 
                	::cos(wt)*svPosECEF[0] + ::sin(wt)*svPosECEF[1],
               	   -::sin(wt)*svPosECEF[0] + ::cos(wt)*svPosECEF[1],
               		svPosECEF[2]);
			}
			else
				pseudorange += gazebo::math::Rand::GetDblNormal(0,rot);

			// DEAL WITH EPHEMERIS ERROR /////////////////////////////////////////

			if (eph < 0)
			{
				// Get the broadcast position
				svPosECEF += gpstk::Position(
					env->gnss(i).err().x(),
					env->gnss(i).err().y(),
					env->gnss(i).err().z()
				);
			}
			else
				pseudorange += gazebo::math::Rand::GetDblNormal(0,eph);


			// TROPOSPHERIC DELAY ///////////////////////////////////////////////

			if (tro < 0)
				pseudorange += env->gnss(i).delay_trop();
			else
				pseudorange += gazebo::math::Rand::GetDblNormal(0,tro);

			// IONOSPHERIC DELAY ////////////////////////////////////////////////
						
			if (ion < 0)
			{
				if (typ.compare(F1)==0)
					pseudorange += env->gnss(i).delay_iono_f1();
				if (typ.compare(F2)==0)
					pseudorange += env->gnss(i).delay_iono_f2();
			}
			else
				pseudorange += gazebo::math::Rand::GetDblNormal(0,ion);

			// RELATIVITY DELAY /////////////////////////////////////////////////

			if (rel < 0)
				pseudorange += env->gnss(i).relcorr() * C_MPS;
			else
				pseudorange += gazebo::math::Rand::GetDblNormal(0,rel);

			// CLOCK DELAY //////////////////////////////////////////////////////

			if (clk < 0)
				pseudorange += env->gnss(i).clkbias() * C_MPS;
			else
				pseudorange += gazebo::math::Rand::GetDblNormal(0,clk);

			// CLOCK DELAY //////////////////////////////////////////////////////

			if (sig > 0)
				pseudorange += gazebo::math::Rand::GetDblNormal(0,sig);

			// SUMMARY //////////////////////////////////////////////////////////

			if (DEBUG)
			{
				ROS_INFO("-- Satellite %d : true range %f with pseudorange error %f",
					i, trurange, pseudorange - trurange);
			}

			// DATA PREPARATION //////////////////////////////////////////////////
         	
			// Add satellite
			satellites.push_back(sid);

         	// Add ephemeride
         	ephemerides(counter,0) = svPosECEF[0];	// Erronous satellite X
         	ephemerides(counter,1) = svPosECEF[1];
         	ephemerides(counter,2) = svPosECEF[2];
         	ephemerides(counter,3) = pseudorange;	// Erroneous pseudorange
         	counter++;

		}
		catch (Exception &e)
		{
			ROS_WARN("Processing exception : %s", e.what().c_str());
		}
	}


	if (DEBUG)
	{
		ROS_INFO("Number of satellites in view is %d ",counter);
	}

	// Resize the matrix
	SVD.resize(counter,4,0.0);
	for (int i = 0; i < counter; i++)
		for (int j = 0; j < 4; j++)
			SVD(i,j) = ephemerides(i,j);

	// Get a solution!
  	try
  	{

		// We want a once-off solution (segfaults if on)
		solver.hasMemory = false;
		solver.RMSLimit  = 3e6;

		// These wills tore the resifuals and slopes after convergence
	  	resids.resize(counter);
	  	slopes.resize(counter);
	 
	  	// Position solution
		statusFix =  solver.SimplePRSolution(
			currentTime,
			satellites,
			SVD,
			covariance,
			tropModel,
			6,
			3.e-7,
			systems,
			resids,
			slopes);
		
		// Dilution of precision
		statusDOP = solver.DOPCompute();


		// Finished!
		if (DEBUG)
		{

			// Get the position
			gpstk::Position solPosECEF = gpstk::Position(
				solver.Solution[0],
				solver.Solution[1],
				solver.Solution[2]
			);

			// Get the error
			gpstk::Position errPosECEF = solPosECEF - msPosECEF;

			// DEBUG!
			ROS_INFO("Solver finished in %d iterations with SOL status %d and DOP status %d", solver.NIterations, statusFix, statusDOP);
			ROS_INFO("-- POS: X:%f Y:%f Z:%f", solPosECEF.X(), solPosECEF.Y(), solPosECEF.Z());
			ROS_INFO("-- ERR: X:%f Y:%f Z:%f", errPosECEF.X(), errPosECEF.Y(), errPosECEF.Z());
			ROS_INFO("-- DOP: T:%f P:%f G:%f", solver.TDOP, solver.PDOP, solver.GDOP);
		}
	}
	catch (Exception &e)
	{
		ROS_WARN("Solver exception : %s", e.what().c_str());
	}
}

bool GNSS::GetStatusFix()
{
	return statusFix;
}

bool GNSS::GetStatusDOP()
{
	return statusDOP;
}

// Number of satellites
int GNSS::GetNumSats()
{
	return solver.Nsvs;
}

// Position
gazebo::math::Vector3 GNSS::GetPosition()
{
	return gazebo::math::Vector3(
		solver.Solution[0],
		solver.Solution[1],
		solver.Solution[2]
	);
}

// Velocity
gazebo::math::Vector3 GNSS::GetVelocity()
{
	return gazebo::math::Vector3(
		solver.Solution[0] - oldSolution[0],
		solver.Solution[1] - oldSolution[1],
		solver.Solution[2] - oldSolution[2]
	) / (currentTime - oldTime);
}

// Dilution of precision
gazebo::math::Vector3 GNSS::GetDOP()
{
	return gazebo::math::Vector3(
		solver.TDOP,
		solver.PDOP,
		solver.GDOP
	);
}
