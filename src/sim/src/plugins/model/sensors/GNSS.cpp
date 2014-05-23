#include "GNSS.h"

using namespace gazebo;

// Constructor
GNSS::GNSS() : ready(false), tropModel(&tropModelZero) {}

// When new environment data arrives
void GNSS::Receive(SatellitesPtr& msg)
{
	// Use the copy constructor to copy the message
	sats = *msg;

	// We now have data, so we can obtain a solution
	ready = true;
}

// All sensors must be configured using the current model information and the SDF
bool GNSS::Configure(physics::LinkPtr link, sdf::ElementPtr root)
{
	// Backup the link
	linkPtr = link;

	// Get parameters from the SDF file
    std::string gpsUse, gpsEph, gpsClk, gpsIon, gpsTro, gloUse, gloEph, gloClk, gloIon, gloTro;
    root->GetElement("solver")->GetElement("gps")->GetElement("use")->GetValue()->Get(gpsUse);
    root->GetElement("solver")->GetElement("gps")->GetElement("ion")->GetValue()->Get(gpsIon);
    root->GetElement("solver")->GetElement("gps")->GetElement("tro")->GetValue()->Get(gpsTro);
    root->GetElement("solver")->GetElement("gps")->GetElement("eph")->GetValue()->Get(gpsEph);
    root->GetElement("solver")->GetElement("gps")->GetElement("clk")->GetValue()->Get(gpsClk);
    root->GetElement("solver")->GetElement("glo")->GetElement("use")->GetValue()->Get(gloUse);
    root->GetElement("solver")->GetElement("glo")->GetElement("ion")->GetValue()->Get(gloIon);
    root->GetElement("solver")->GetElement("glo")->GetElement("tro")->GetValue()->Get(gloTro);
    root->GetElement("solver")->GetElement("glo")->GetElement("eph")->GetValue()->Get(gloEph);
    root->GetElement("solver")->GetElement("glo")->GetElement("clk")->GetValue()->Get(gloClk);
    root->GetElement("solver")->GetElement("nsiterations")->GetValue()->Get(_maxIterations);
    root->GetElement("solver")->GetElement("errtolerance")->GetValue()->Get(_minError);
    root->GetElement("solver")->GetElement("minelevation")->GetValue()->Get(_minElevation);
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
	nReceiver = NoiseFactory::Create(root->GetElement("errors")->GetElement("clock"));

	// We will be using two GNSS systems in total
	if (_gpsUse) 
		systems.push_back(gpstk::SatID::systemGPS);
	if (_gloUse) 
		systems.push_back(gpstk::SatID::systemGlonass);

    // Initialise a node pointer
    nodePtr = transport::NodePtr(new transport::Node());
    nodePtr->Init(linkPtr->GetModel()->GetWorld()->GetName());

    // Subscribe to messages about wind conditions
    subPtr = nodePtr->Subscribe("~/satellites", &GNSS::Receive, this);

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

	// OBTAIN THE CURRENT TIME /////////////////////////////////////////////

	currentTime.set(
		(double) sats.epoch(),
		(gpstk::TimeSystem) gpstk::TimeSystem::UTC
	);

	// OBTAIN THE CURRENT RECEIVER POSITION ////////////////////////////////
   
   	// Get the gazebo WGS84 position of the receiver
    math::Vector3 rcvPos = linkPtr->GetModel()->GetWorld()->GetSphericalCoordinates()
    	->SphericalFromLocal(linkPtr->GetWorldPose().pos);
      
    // Get the gpstk WGS84 position of the receiver
  	gpstk::Position rcvWGS84(
  		rcvPos.x, rcvPos.y, rcvPos.z, gpstk::Position::Geodetic, &wgs84
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
	ephemerides.resize(sats.svs_size(), 4, 0.0);

	// Iterate over the GPS satellite vehicles
	for (int i = 0; i < sats.svs_size(); i++)
	{
		// Work out the range
		try
		{
			// Get the true position
			gpstk::Position satECEF = gpstk::Position(
				sats.svs(i).pos().x(),
				sats.svs(i).pos().y(),
				sats.svs(i).pos().z(), 
				gpstk::Position::Cartesian
			);

			// If the satellite is below a certain elevation, ignore
			// it by changing the ID to negative (see PRSOlver)
			int id = sats.svs(i).prn();
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
			gpstk::SatID::SatelliteSystem sys = (gpstk::SatID::SatelliteSystem) sats.svs(i).sys();

			// Save the satellite
			satellites.push_back(gpstk::SatID(id,sys));

			// Perform error perturbation
			switch(sys)
			{
			case gpstk::SatID::systemGPS:
				if (!_gpsEph)
				{
					satECEF[0] += sats.svs(i).err_pos().x();
					satECEF[1] += sats.svs(i).err_pos().y();
					satECEF[2] += sats.svs(i).err_pos().z();
				}
				if (!_gpsClk) pseudorange += sats.svs(i).err_clk();
				if (!_gpsTro) pseudorange += sats.svs(i).err_tro();
				if (!_gpsIon) pseudorange += sats.svs(i).err_ion();
				break;
			case gpstk::SatID::systemGlonass:
				if (!_gloEph)
				{
					satECEF[0] += sats.svs(i).err_pos().x();
					satECEF[1] += sats.svs(i).err_pos().y();
					satECEF[2] += sats.svs(i).err_pos().z();
				}
				if (!_gloClk) pseudorange += sats.svs(i).err_clk();
				if (!_gloTro) pseudorange += sats.svs(i).err_tro();
				if (!_gloIon) pseudorange += sats.svs(i).err_ion();
				break;
			}

			// Add the local receive clock noise
			pseudorange += nReceiver->DrawScalar(t);

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
	SVD.resize(sats.svs_size(),4,0.0);
	for (int i = 0; i < sats.svs_size(); i++)
		for (int j = 0; j < 4; j++)
			SVD(i,j) = ephemerides(i,j);
  	resids.resize(sats.svs_size());
  	slopes.resize(sats.svs_size());

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
			_maxIterations,							// Integer limit on solver iterations
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

   	// TODO: fix this
    //posNew = linkPtr->GetModel()->GetWorld()->GetSphericalCoordinates()
    // 	->LocalFromSpherical(math::Vector3(estPos[0],estPos[1],estPos[2]));
   	posNew.x = estPos[0];
   	posNew.y = estPos[1];
   	posNew.z = estPos[2];

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
