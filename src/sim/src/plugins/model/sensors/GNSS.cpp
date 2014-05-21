#include "GNSS.h"

using namespace gazebo;

// Constructor
GNSS::GNSS() : ready(false), tropModel(&tropModelZero) {}

// When new environment data arrives
void GNSS::Receive(SatellitesPtr sats)
{
	// Use the copy constructor to copy the message
	msg = *sats;

	// We now have data, so we can obtain a solution
	ready = true;
}

// All sensors must be configured using the current model information and the SDF
bool GNSS::Solve()
{
	// Get a solution!
  	try
  	{
		// We want a once-off solution (segfaults if on)
		solver.hasMemory = false;
		solver.RMSLimit  = 3e6;
	 
	  	// Position solution
		statusFix =  solver.SimplePRSolution(
			currentTime,							// Current time
			satellites,								// Satellite with negative id are ignored by the solver
			SVD,									// Satellite direction cosines and corrected pseudorange data.
			covariance,								// NXN pseudorange covariance matrix inverse (m^-2)
			tropModel,								// Model for tropospheric correction
			6,										// Integer limit on solver iterations
			3.e-7,									// Convergence criterion
			systems,								// Satellite systems to be used
			resids,									// Post-fit range residuals
			slopes									// Slope value used for each satellite
		);

		// Did we manage to get a fix?
		return (statusFix == 0);
	}
	catch (gpstk::Exception &e)
	{
		ROS_WARN("Solver exception : %s", e.what().c_str());
	}

	// Unsuccessful
	return false;
}

// All sensors must be configured using the current model information and the SDF
bool GNSS::Configure(sdf::ElementPtr root)
{
	// We will be using two GNSS systems in total
	systems.push_back(SatID::systemGPS);
	systems.push_back(SatID::systemGlonass);

    // Initialise a node pointer
    nodePtr = transport::NodePtr(new transport::Node());
    nodePtr->Init(modPtr->GetWorld()->GetName());

    // Subscribe to messages about wind conditions
    subPtr = nodePtr->Subscribe("~/satellites", &GNS::Receive, this);

    // Success
	return true;
}

// All sensors must be resettable
void GNSS::Reset()
{  
	// Not ready yet
	ready = false;
}

// Get the current altitude
bool GNSS::GetMeasurement(physics::LinkPtr linkPtr, hal_sensor_gnss::Data& msg)
{
	// SPECIAL CASE: NO SATELLITES RECEIVED YET ////////////////////////////

	if (!ready)
		return false;

	// OBTAIN THE CURRENT RECEIVER POSITION ////////////////////////////////
   
   	// Get the gazebo WGS84 position of the receiver
    math::vector3 rcvPos = linkPtr->GetModel()->GetWorld()->GetSphericalCoordinates()
    	->SphericalFromLocal(linkPtr->GetWorldPose().pos);
      
    // Get the gpstk WGS84 position of the receiver
  	gpstk::Position rcvWGS84(
  		pos.x, pos.y, pos.z, gpstk::Position::Geodetic, &wgs84
	);

  	// Convert to ECEF position
	gpstk::Position rcvECEF 
		= rcvWGS84.transformTo(gpstk::Position::Geocentric).asECEF();

	// PROCESS THE SATELLITE DATA //////////////////////////////////////////

	// Allocate for solution
    std::vector<gpstk::SatID>       satellites;     // List of satellite vehicles
    gpstk::Matrix<double>           covariance;     // For weighting variou solutions!
    gpstk::Matrix<double>           ephemerides;    // All ephemerides
    gpstk::Matrix<double>           SVD;            // Solution vector 
    gpstk::Vector<double>           resids;         // Residual errors 
    gpstk::Vector<double>           slopes;         // Residual slopes
	
	// Make the ephemerides container big enough to store all SV info
	ephemerides.resize(mag.gps_size() + mag.glo_size(), 4, 0.0);

	// Iterate over the GPS satellite vehicles
	for (int i = 0; i < mag.gps_size(); i++)
	{
		// Work out the range
		try
		{
			// Get the true position
			gpstk::Position satECEF = gpstk::Position(
				mag.gps(i).pos().x(),
				mag.gps(i).pos().y(),
				mag.gps(i).pos().z(), 
				gpstk::Position::Cartesian
			);

			// If the satellite is below a certain elevation, ignore
			// it by changing the ID to negative (see PRSOlver)
			int id = mag.gps(i).prn();
			if (rcvECEF.elvAngle(satECEF) < _me)
				id *= -1;

			// During the flight time of the radio signal the earth rotated, 
			// meaning that the receiver changed position. So, we're going to
			// subtract this amount from the satellite position.
			double tof = range(rcvECEF, satECEF) / ellip.c();
			double ang = ellip.angVelocity() * tof;
			satECEF[0] = -::cos(ang)*satECEF[0] - ::sin(ang)*satECEF[1];
			satECEF[1] =  ::sin(ang)*satECEF[0] - ::cos(ang)*satECEF[1];

			// Get the true range
			satellites.push_back(
				gpstk::SatID(id, gpstk::SatID::systemGPS)
			);

         	// Add ephemeride
         	ephemerides(i,0) = satECEF[0];
         	ephemerides(i,1) = satECEF[1];
         	ephemerides(i,2) = satECEF[2];
         	ephemerides(i,3) = range(rcvECEF, satECEF);
		}
		catch (gpstk::Exception &e)
		{
			ROS_WARN("Processing exception : %s", e.what().c_str());
		}
	}

	// Iterate over the GPS satellite vehicles
	for (int i = 0; i < mag.glo_size(); i++)
	{
		// Work out the range
		try
		{
			// Get the true position
			gpstk::Position satECEF = gpstk::Position(
				mag.glo(i).pos().x(),
				mag.glo(i).pos().y(),
				mag.glo(i).pos().z(), 
				gpstk::Position::Cartesian
			);

			// If the satellite is below a certain elevation, ignore
			// it by changing the ID to negative (see PRSOlver)
			int id = mag.glo(i).prn();
			if (rcvECEF.elvAngle(satECEF) < _me)
				id *= -1;

			// During the flight time of the radio signal the earth rotated, 
			// meaning that the receiver changed position. So, we're going to
			// subtract this amount from the satellite position.
			double tof = range(rcvECEF, satECEF) / ellip.c();
			double ang = ellip.angVelocity() * tof;
			satECEF[0] = -::cos(ang)*satECEF[0] - ::sin(ang)*satECEF[1];
			satECEF[1] =  ::sin(ang)*satECEF[0] - ::cos(ang)*satECEF[1];

			// Get the true range
			satellites.push_back(
				gpstk::SatID(id, gpstk::SatID::systemGlonass)
			);

         	// Add ephemeride
         	ephemerides(mag.gps_size()+i,0) = satECEF[0];
         	ephemerides(mag.gps_size()+i,1) = satECEF[1];
         	ephemerides(mag.gps_size()+i,2) = satECEF[2];
         	ephemerides(mag.gps_size()+i,3) = range(rcvECEF, satECEF);
		}
		catch (gpstk::Exception &e)
		{
			ROS_WARN("Processing exception : %s", e.what().c_str());
		}
	}

	// Resize other matrices used in the computation
	SVD.resize(mag.gps_size() + mag.glo_size(),4,0.0);
	for (int i = 0; i < counter; i++)
		for (int j = 0; j < 4; j++)
			SVD(i,j) = ephemerides(i,j);
  	resids.resize(mag.gps_size() + mag.glo_size());
  	slopes.resize(mag.gps_size() + mag.glo_size());

  	// TRY AND OBTAIN A NAVIGATION SOLUTION ///////////////////////////////////

  	// Did we
  	int statusFix = -1;
	try
	{
		// Solve roptions
		solver.hasMemory = false;
		solver.RMSLimit  = 3e6;
	 
	  	// Position solution
		statusFix =  solver.SimplePRSolution(
			currentTime,							// Current time
			satellites,								// Satellite with negative id are ignored by the solver
			SVD,									// Satellite direction cosines and corrected pseudorange data.
			covariance,								// NXN pseudorange covariance matrix inverse (m^-2)
			tropModel,								// Model for tropospheric correction
			6,										// Integer limit on solver iterations
			3.e-7,									// Convergence criterion
			systems,								// Satellite systems to be used
			resids,									// Post-fit range residuals
			slopes									// Slope value used for each satellite
		);
	}
	catch (gpstk::Exception &e)
	{
		ROS_WARN("Solver exception : %s", e.what().c_str());
	}

	// CONVERT BACK TO GAZEBO COORDINATE FRAME //////////////////////////////
	      
    // Get the ECEF position of the receiver
  	gpstk::Position estPos(
  		solver.Solution[0], solver.Solution[0], solver.Solution[0]
	);

  	// Convert to a wgs84 position
  	estPos = estPos.asGeodetic(&wgs84);

   	// Get the gazebo WGS84 position of the receiver
    posNew = linkPtr->GetModel()->GetWorld()->GetSphericalCoordinates()
    	->LocalFromSpherical(math::Vector3(estPos[0],estPos[1],estPos[2]));

    // Work out the velocity
    velNew = (posNew - posOld) / dt;

    // Backup the old position for next iteration
    posOld = posNew;

	// Copy the solution
	msg.x = posNew.x;
	msg.y = posNew.y;
	msg.z = posNew.z;
	msg.u = velNew.x;
	msg.v = velNew.y;
	msg.w = velNew.z;

	// Success!
	return (statusFix == 0);
}
