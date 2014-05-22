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
bool GNSS::Configure(physics::LinkPtr linkPtr, sdf::ElementPtr root)
{
	// Backup the link
	linkPtr = linkPtr;

	// Get parameters from the SDF file
    std::string enabled, gpsUse, gpsEph, gpsClk, gpsIon, gpsTro, gloUse, gloEph, gloClk, gloIon, gloTro;
    root->GetElement("solver")->GetElement("gps")->GetElement("use")->GetValue()->get(gpsUse);
    root->GetElement("solver")->GetElement("gps")->GetElement("ion")->GetValue()->get(gpsIon);
    root->GetElement("solver")->GetElement("gps")->GetElement("tro")->GetValue()->get(gpsTro);
    root->GetElement("solver")->GetElement("gps")->GetElement("eph")->GetValue()->get(gpsEph);
    root->GetElement("solver")->GetElement("gps")->GetElement("clk")->GetValue()->get(gpsClk);
    root->GetElement("solver")->GetElement("gps")->GetElement("use")->GetValue()->get(gloUse);
    root->GetElement("solver")->GetElement("gps")->GetElement("ion")->GetValue()->get(gloIon);
    root->GetElement("solver")->GetElement("gps")->GetElement("tro")->GetValue()->get(gloTro);
    root->GetElement("solver")->GetElement("gps")->GetElement("eph")->GetValue()->get(gloEph);
    root->GetElement("solver")->GetElement("gps")->GetElement("clk")->GetValue()->get(gloClk);
    root->GetElement("solver")->GetElement("nsiterations")->GetValue()->get(_maxIterations);
    root->GetElement("solver")->GetElement("errtolerance")->GetValue()->get(_minError);
    root->GetElement("solver")->GetElement("minelevation")->GetValue()->get(_minElevation);
    root->GetElement("enabled")->GetElement("use")->GetValue()->get(enabled);
    _enabled = (enabled.compare("true")==0);
    _gpsUse  = ( gpsUse.compare("true")==0);
    _gpsEph  = ( gpsEph.compare("true")==0);
    _gpsClk  = ( gpsClk.compare("true")==0);
    _gpsTro  = ( gpsTro.compare("true")==0);
    _gpsIon  = ( gpsIon.compare("true")==0);
    _gloUse  = ( gloUse.compare("true")==0);
    _gloEph  = ( gloEph.compare("true")==0);
    _gloIon  = ( gloIon.compare("true")==0);
    _gloClk  = ( gloClk.compare("true")==0);
    _gloTro  = ( gloTro.compare("true")==0);

    // Create a noise distribution to model receiver clock error
	nReceiver = NoiseFactory::Create(linkPtr,root->GetElement("errors")->GetElement("rcvclk"));

	// We will be using two GNSS systems in total
	if (_gpsUse) 
		systems.push_back(SatID::systemGPS);
	if (_gloUse) 
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
bool GNSS::GetMeasurement(double t, hal_sensor_gnss::Data& msg)
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
	ephemerides.resize(msg.svs_size(), 4, 0.0);

	// Iterate over the GPS satellite vehicles
	for (int i = 0; i < msg.svs_size(); i++)
	{
		// Work out the range
		try
		{
			// Get the true position
			gpstk::Position satECEF = gpstk::Position(
				msg.svs(i).pos().x(),
				msg.svs(i).pos().y(),
				msg.svs(i).pos().z(), 
				gpstk::Position::Cartesian
			);

			// If the satellite is below a certain elevation, ignore
			// it by changing the ID to negative (see PRSOlver)
			int id = msg.svs(i).prn();
			if (rcvECEF.elvAngle(satECEF) < _minElevation)
				id *= -1;

			// Calculate the true pseudorange
			double pseudorange = range(rcvECEF, satECEF);

			// During the flight time of the radio signal the earth rotated, 
			// meaning that the receiver changed position. So, we're going to
			// subtract this amount from the satellite position. Th PRSolver
			// will add it back, thereby correcting for the effect.
			double tof = pseudorange / ellip.c();
			double ang = ellip.angVelocity() * tof;
			satECEF[0] = -::cos(ang)*satECEF[0] - ::sin(ang)*satECEF[1];
			satECEF[1] =  ::sin(ang)*satECEF[0] - ::cos(ang)*satECEF[1];

			// Get the satellite system
			gpstk::SatID::SatelliteSystem sys = msg.svs(i).sys();

			// Save the satellite
			satellites.push_back(gpstk::SatID(id,sys));

			// Perform error perturbation
			switch(sys)
			{
			case gpstk::SatID::systemGPS:
				if (!_gpsEph)
				{
					satECEF[0] += msg.svs(i).err_pos().x();
					satECEF[1] += msg.svs(i).err_pos().y();
					satECEF[2] += msg.svs(i).err_pos().z();
				}
				if (!_gpsClk) pseudorange += msg.svs(i).err_clk();
				if (!_gpsTro) pseudorange += msg.svs(i).err_tro();
				if (!_gpsIon) pseudorange += msg.svs(i).err_ion();
				break;
			case gpstk::SatID::systemGlonass:
				if (!_gloEph)
				{
					satECEF[0] += msg.svs(i).err_pos().x();
					satECEF[1] += msg.svs(i).err_pos().y();
					satECEF[2] += msg.svs(i).err_pos().z();
				}
				if (!_gloClk) pseudorange += msg.svs(i).err_clk();
				if (!_gloTro) pseudorange += msg.svs(i).err_tro();
				if (!_gloIon) pseudorange += msg.svs(i).err_ion();
				break;
			}

			// Add the local receive clock noise
			pseudorange += nReceiver.DrawScalar(t);

         	// Add ephemeride
         	ephemerides(i,0) = satECEF[0];
         	ephemerides(i,1) = satECEF[1];
         	ephemerides(i,2) = satECEF[2];
         	ephemerides(i,3) = pseudorange;
		}
		catch (gpstk::Exception &e)
		{
			ROS_WARN("Processing exception : %s", e.what().c_str());
		}
	}


	// Resize other matrices used in the computation
	SVD.resize(msg.svs_size(),4,0.0);
	for (int i = 0; i < msg.svs_size(); i++)
		for (int j = 0; j < 4; j++)
			SVD(i,j) = ephemerides(i,j);
  	resids.resize(msg.svs_size());
  	slopes.resize(msg.svs_size());

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
			_maxIterations							// Integer limit on solver iterations
			_minError, 								// Convergence criterion
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
   	timNew = t;
    posNew = linkPtr->GetModel()->GetWorld()->GetSphericalCoordinates()
    	->LocalFromSpherical(math::Vector3(estPos[0],estPos[1],estPos[2]));

    // Work out the velocity
	if (timNew - timOld > 0)
    	velNew = (posNew - posOld) / (timNew - timOld);

    // Backup the old position for next iteration
    timOld = timNew;
    posOld = posNew;

	// Copy the solution
	msg.t = timNew;
	msg.x = posNew.x;
	msg.y = posNew.y;
	msg.z = posNew.z;
	msg.u = velNew.x;
	msg.v = velNew.y;
	msg.w = velNew.z;

	// Success!
	return (statusFix == 0);
}
