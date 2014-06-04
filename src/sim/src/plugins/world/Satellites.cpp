// Standar dlibraries
#include <vector>

// Simulator description format (SDF)
#include <sdf/sdf.hh>

// Boost includes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

// For converting Gazebo <-> ECEF coordinates
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// A common time and position system
#include <gpstk/CommonTime.hpp>
#include <gpstk/Position.hpp>
#include <gpstk/WGS84Ellipsoid.hpp>

// Class for storing "broadcast-type" ephemerides
#include <gpstk/SP3EphemerisStore.hpp>
#include <gpstk/GPSEphemerisStore.hpp>
#include <gpstk/GloEphemerisStore.hpp>

// Tropospheric modelling
#include <gpstk/TropModel.hpp>

// Classes for handling RINEX satellite navigation parameters (ephemerides)
#include <gpstk/Rinex3NavHeader.hpp>
#include <gpstk/Rinex3NavData.hpp>
#include <gpstk/Rinex3NavStream.hpp>

// Classes for handling RINEX files with IONEX data
#include <gpstk/IonexStore.hpp>
#include <gpstk/IonexData.hpp>
#include <gpstk/IonexHeader.hpp>
#include <gpstk/IonexStream.hpp>

// Class to store satellite precise navigation data
#include <gpstk/SP3Data.hpp>
#include <gpstk/SP3Header.hpp>
#include <gpstk/SP3Stream.hpp>

// ROS includes
#include <ros/ros.h>

// Custom messages
#include "environment.pb.h"
#include "meteorological.pb.h"
#include "satellites.pb.h"
#include "noise.pb.h"

// Required for noise distributions
#include "../../noise/NoiseFactory.h"

// Approx frequencies used to claculate ionospheric perturbation
#define SPEED_OF_LIGHT 	299792458.0

namespace gazebo
{
	// Convenience declarations
	typedef const boost::shared_ptr<const msgs::Meteorological> MeteorologicalPtr;
	typedef const boost::shared_ptr<const msgs::Environment> 	EnvironmentPtr;
	typedef const boost::shared_ptr<const msgs::Noise> 			NoisePtr;

	class Satellites : public WorldPlugin
	{

	private:

		// BASIC PARAMETERS ////////////////////////////////////////////////////

		double                              rate;

		// MDATA STORAGE MECHANISMS ////////////////////////////////////////////

		// UTC time systems
		gpstk::CommonTime 					currentTimeUTC, currentTimeGPS, currentTimeGLO;
		gpstk::CivilTime 					civilTime;


	    // For storing GPS and Glonass ephemerides
	    gpstk::GPSEphemerisStore    		gpsEphemerides;
	    gpstk::GloEphemerisStore    		gloEphemerides;
	    gpstk::SP3EphemerisStore    		sp3Ephemerides;
	    gpstk::IonexStore           		tecStore;

 		// Goad and Goodman (1974) troppspheric model
 		gpstk::WGS84Ellipsoid           	wgs84;
	    gpstk::GGTropModel                 	tropModel;
		gpstk::Triple 						ionoModel;

	    // Locally store temperature, pressure and humidity (troposhperic model)
	    Noise* 								nGPSeph[gpstk::MAX_PRN];
	    Noise* 								nGPSclk[gpstk::MAX_PRN];
	    Noise* 								nGPStro[gpstk::MAX_PRN];
	    Noise* 								nGPSion[gpstk::MAX_PRN];
	    Noise* 								nGLOeph[gpstk::MAX_PRN];
	    Noise* 								nGLOclk[gpstk::MAX_PRN];
	    Noise* 								nGLOtro[gpstk::MAX_PRN];
	    Noise*								nGLOion[gpstk::MAX_PRN];

	    // Required for messaging
	    physics::WorldPtr 					worldPtr;
		transport::NodePtr 					nodePtr;
		transport::PublisherPtr 			pubPtr;
		transport::SubscriberPtr 			subPtrMet, subPtrEnv, subPtrNoi;
		
		// Required for ROS interaction
		ros::Timer 							timer;
		ros::NodeHandle 					rosNode;

      	// Message containing information
      	msgs::Satellites 					sat;
      	msgs::Meteorological                met;
      	msgs::Environment  		            env;

      	// Meterological info
      	bool								rcvMet, rcvEnv;

	    //  Called to update the world information
	    void Update(const ros::TimerEvent& event)
	    {
	    	// We can onyl proceed if we know the UTC time and current weather
	    	if (!rcvMet || !rcvEnv)
	    		return;

			// CALCULATE THE CURRENT TIME /////////////////////////////////////////////////////
			
			// Simulated seconds since UTC start point
			double t = worldPtr->GetSimTime().Double();

            // Create a common time variable from the UTC info in the SDF data
            civilTime = gpstk::CivilTime(
            	env.year(),
            	env.month(),
            	env.day(),
            	env.hour(),
            	env.minute(),
            	env.second(),
            	gpstk::TimeSystem::UTC
            );

			// Calculate UTC time
			currentTimeUTC = civilTime.convertToCommonTime();
			currentTimeUTC.addSeconds(t);
			currentTimeUTC.setTimeSystem(gpstk::TimeSystem::UTC);

			// Calculate UTC time
			currentTimeGPS = civilTime.convertToCommonTime();
			currentTimeGPS.addSeconds(t +
                gpstk::TimeSystem::Correction(
                    gpstk::TimeSystem::UTC, gpstk::TimeSystem::GPS,
                    civilTime.year, civilTime.month, civilTime.day 
                )
            );
			currentTimeGPS.setTimeSystem(gpstk::TimeSystem::GPS);

			// Calculate UTC time
			currentTimeGLO = civilTime.convertToCommonTime();
			currentTimeGLO.addSeconds(t +
                gpstk::TimeSystem::Correction(
                    gpstk::TimeSystem::UTC, gpstk::TimeSystem::GLO,
                    civilTime.year, civilTime.month, civilTime.day 
                )
            );
			currentTimeGLO.setTimeSystem(gpstk::TimeSystem::GLO);

			// Get the epoch from the time subsystem
			double epoch;
			currentTimeUTC.get(epoch);
			sat.set_epoch(epoch);

			// CALCULATE THE CURRENT POSITION /////////////////////////////////////////////////

			// Use geographiclib for projection
			GeographicLib::Geocentric wgs84_ecef(
				GeographicLib::Constants::WGS84_a(), 
				GeographicLib::Constants::WGS84_f()
			);
			GeographicLib::LocalCartesian wgs84_enu(
				worldPtr->GetSphericalCoordinates()->GetLatitudeReference().Degree(), 
				worldPtr->GetSphericalCoordinates()->GetLongitudeReference().Degree(), 
				worldPtr->GetSphericalCoordinates()->GetElevationReference(), 
				wgs84_ecef
			);

			// Find the latitude, longitude and height for the origin
			double lat, lon, h;
			wgs84_enu.Reverse(0, 0, 0, lat, lon, h);

			// USE gpstk for projection
			gpstk::Position originPosGeodetic    = gpstk::Position(
				lat, 
				lon, 
				h, 
				gpstk::Position::Geodetic, 
				&wgs84
			);
			gpstk::Position originPosGeocentric  = originPosGeodetic.transformTo(gpstk::Position::Geocentric);
			gpstk::Position originPosECEF        = originPosGeodetic.asECEF();

			// TROPOSHPERIC DELAYS //////////////////////////////////////////////////////////////

			tropModel.setWeather(met.temperature() - 273.15, met.pressure(), met.humidity());

			// IONOSPHERIC DELAYS ///////////////////////////////////////////////////////////////

			gpstk::Triple tec;
			try
			{
				tec = tecStore.getIonexValue(currentTimeGPS, originPosGeocentric, 1); 
			}
			catch (const gpstk::Exception& e)
			{
				ROS_WARN("Problem setting ionospheric data: %s", e.what().c_str());
			}

			// TIME EPOCH //////////////////////////////////////////////////////////////////////

			// Clear all existing satellites
			sat.clear_svs();

			// GPS EPHEMERIDES /////////////////////////////////////////////////////////////////
			
			// Iterate over possible satellite identifiers
			for (int prn = 1; prn <= gpstk::MAX_PRN; ++prn)
			{
				try
				{
					// Don't look for invalid satellites
					if (sp3Ephemerides.isPresent(gpstk::SatID(prn,gpstk::SatID::systemGPS)))
					{
						// Get the ephemeris
						gpstk::Xvt eph = sp3Ephemerides.getXvt(gpstk::SatID(prn,gpstk::SatID::systemGPS),currentTimeGPS);

						// Elevation of the current satellite with respect to HOME position
						gpstk::Triple satellitePos = eph.getPos();

						// Save to the message
						msgs::Vehicle* gps = sat.add_svs();
						gps->set_sys(gpstk::SatID::systemGPS);
						gps->set_prn(prn);

						// Write the position
						gps->mutable_pos()->set_x(satellitePos[0]);
						gps->mutable_pos()->set_y(satellitePos[1]);
						gps->mutable_pos()->set_z(satellitePos[2]);

						// Default = zero error
						gps->mutable_err_pos()->set_x(0);
						gps->mutable_err_pos()->set_y(0);
						gps->mutable_err_pos()->set_z(0);
						gps->set_err_clk(0);
						gps->set_err_tro(0);
						gps->set_err_ion(0);

						// Try and get the broadcast ephemeride
						try
						{
							// Only add noise if its needed
							if (!NoiseFactory::GetEnabled())
								continue;

							// Get the broadcast ephemeris
							gpstk::Xvt ephb = gpsEphemerides.getXvt(
								gpstk::SatID(prn,gpstk::SatID::systemGPS),
								currentTimeGPS
							);

							// Elevation of the current satellite with respect to HOME position
							gpstk::Triple err = ephb.getPos() - satellitePos;

							// Set the ephemeris error
							gps->mutable_err_pos()->set_x(err[0]);
							gps->mutable_err_pos()->set_y(err[1]);
							gps->mutable_err_pos()->set_z(err[2]);

							// Write the clock bias and relativity correction
							gps->set_err_clk(
								SPEED_OF_LIGHT * (ephb.getClockBias() - eph.getClockBias())
							);

							// Set the tropospheic delay, based on the simulation location
							gps->set_err_tro(
								tropModel.correction(
									originPosECEF,                     	 // Current receiver position
									satellitePos,                        // Current satellite position
									currentTimeGPS                       // Time of observation
								)
							);

							// GET L1 ionosphere delay
							gps->set_err_ion(
								tecStore.getIonoL1(
									originPosECEF.elvAngle(satellitePos), // Elevation of the satellite
									tec[0],                               // Total electron count
									"NONE"                                // Mapping function
								) 
							);
						}
						catch (const gpstk::Exception& e)
						{ 
							ROS_DEBUG("GPS-B: %s", e.what().c_str());
							
							// Draw a new ephemeride error
							math::Vector3 	err_pos = nGPSeph[prn-1]->DrawVector(t);
							double       	err_clk = nGPSclk[prn-1]->DrawScalar(t);
							double       	err_ion = nGPSion[prn-1]->DrawScalar(t);
							double       	err_tro = nGPStro[prn-1]->DrawScalar(t);

							// Set the ephemeris error
							gps->mutable_err_pos()->set_x(err_pos.x);
							gps->mutable_err_pos()->set_y(err_pos.y);
							gps->mutable_err_pos()->set_z(err_pos.z);
							gps->set_err_clk(err_clk);
							gps->set_err_tro(err_tro);
							gps->set_err_ion(err_ion);
							
						}
					}
				}
				catch (const gpstk::Exception& e)
				{ 
				  ROS_DEBUG("GPS-F: %s", e.what().c_str());
				}
			}

			// GLO EPHEMERIDES /////////////////////////////////////////////////////////////////

			for (int prn = 1; prn <= gpstk::MAX_PRN; ++prn)
			{
				try
				{
				  	// Don't look for invalid satellites
				  	if (sp3Ephemerides.isPresent(gpstk::SatID(prn,gpstk::SatID::systemGlonass)))
				  	{
					    // Get the ephemeris (not that SP3 are in GPST)
					    gpstk::Xvt eph = sp3Ephemerides.getXvt(gpstk::SatID(prn,gpstk::SatID::systemGlonass),currentTimeGPS);

					    // Elevation of the current satellite with respect to HOME position
					    gpstk::Triple satellitePos = eph.getPos();
					
						// Save to the message
						msgs::Vehicle* glo = sat.add_svs();
						glo->set_sys(gpstk::SatID::systemGlonass);
						glo->set_prn(prn);

						// Write the position
						glo->mutable_pos()->set_x(satellitePos[0]);
						glo->mutable_pos()->set_y(satellitePos[1]);
						glo->mutable_pos()->set_z(satellitePos[2]);

						// Default = zero error
						glo->mutable_err_pos()->set_x(0);
						glo->mutable_err_pos()->set_y(0);
						glo->mutable_err_pos()->set_z(0);
						glo->set_err_clk(0);
						glo->set_err_tro(0);
						glo->set_err_ion(0);

						// Try and get the broadcast ephemeride
						try
						{
							// Only add noise if its needed
							if (!NoiseFactory::GetEnabled())
								continue;
							
							// Get the broadcast ephemeris
							gpstk::Xvt ephb = gloEphemerides.getXvt(
								gpstk::SatID(prn,gpstk::SatID::systemGlonass),
								currentTimeGLO
							);

							// Elevation of the current satellite with respect to HOME position
							gpstk::Triple err = ephb.getPos() - satellitePos;

							// Set the ephemeris error
							glo->mutable_err_pos()->set_x(err[0]);
							glo->mutable_err_pos()->set_y(err[1]);
							glo->mutable_err_pos()->set_z(err[2]);

							// Write the clock bias and relativity correction
							glo->set_err_clk(
								SPEED_OF_LIGHT * (ephb.getClockBias()-eph.getClockBias())
							);

							// Set the tropospheic delay, based on the simulation location
							glo->set_err_tro(
								tropModel.correction(
									originPosECEF,                       // Current receiver position
									satellitePos,                        // Current satellite position
									currentTimeGPS                       // Time of observation
								)
							);

							// GET L1 ionosphere delay
							glo->set_err_ion(
								tecStore.getIonoL1(
									originPosECEF.elvAngle(satellitePos), // Elevation of the satellite
									tec[0],                               // Total electron count
									"NONE"                                // Mapping function
								) 
							);
						}
						catch (const gpstk::Exception& e)
						{ 
							ROS_DEBUG("GLO-B: %s", e.what().c_str());

							// Draw a new ephemeride error
							math::Vector3	err_pos = nGLOeph[prn-1]->DrawVector(t);
							double       	err_clk = nGLOclk[prn-1]->DrawScalar(t);
							double       	err_ion = nGLOion[prn-1]->DrawScalar(t);
							double       	err_tro = nGLOtro[prn-1]->DrawScalar(t);

							// Set the ephemeris error
							glo->mutable_err_pos()->set_x(err_pos.x);
							glo->mutable_err_pos()->set_y(err_pos.y);
							glo->mutable_err_pos()->set_z(err_pos.z);
							glo->set_err_clk(err_clk);
							glo->set_err_tro(err_tro);
							glo->set_err_ion(err_ion);
						}			  
					}
				}
				catch(const gpstk::Exception& e)
				{ 
				  ROS_DEBUG("GLO-F: %s", e.what().c_str());
				}
			}
			
			// Publish wind information to all subscribers
			ROS_DEBUG("Sending satellite data");
			pubPtr->Publish(sat);
	    }

	    // CALLBACKS FOR MESSAGES //////////////////////////////////////////////////

		// This will be called whenever a new meteorlogical topic is posted
		void ReceiveEnvironment(EnvironmentPtr& msg)
		{
			// Use copy constructor to indicate
			env = *msg;

			// Received an environment update message
			rcvEnv = true;
		}

		// This will be called whenever a new meteorlogical topic is posted
		void ReceiveMeteorological(MeteorologicalPtr& msg)
		{
			// Use copy constructor to indicate
			met = *msg;

			// Received an environment update message
			rcvMet = true;
		}

		// This will be called whenever a new meteorlogical topic is posted
		void ReceiveNoise(NoisePtr& inmsg)
		{
			NoiseFactory::SetEnabled(inmsg->enable());
		}

	public:

		// Constructor
		Satellites() : rosNode(ros::NodeHandle("satellites")), 
			rate(1.0), rcvMet(false), rcvEnv(false)
		{
			// Make sure that ROS actually started, or there will be some issues...
			if (!ros::isInitialized())
			    ROS_FATAL("A ROS node has not been initialized");

			// Initialise a noise factory
			NoiseFactory::Init();

		}

		// Destructor
		~Satellites()
		{
			// Clear the noise factory
			NoiseFactory::Destroy();
		}

        // All sensors must be configured using the current world information
        void Load(physics::WorldPtr world, sdf::ElementPtr root)
        {
			// Save the world pointer
			worldPtr = world;

			// Basic parameters
			root->GetElement("rate")->GetValue()->Get(rate);

		    // Create a noise distribution for each satellite
		    for (int prn = 1; prn <= gpstk::MAX_PRN; ++prn)
		    {
				nGPSeph[prn-1] = NoiseFactory::Create(root->GetElement("errors")->GetElement("gpseph"));
				nGPSclk[prn-1] = NoiseFactory::Create(root->GetElement("errors")->GetElement("gpsclk"));
				nGPStro[prn-1] = NoiseFactory::Create(root->GetElement("errors")->GetElement("gpstro"));
				nGPSion[prn-1] = NoiseFactory::Create(root->GetElement("errors")->GetElement("gpsion"));
				nGLOeph[prn-1] = NoiseFactory::Create(root->GetElement("errors")->GetElement("gloeph"));
				nGLOclk[prn-1] = NoiseFactory::Create(root->GetElement("errors")->GetElement("gloclk"));
				nGLOtro[prn-1] = NoiseFactory::Create(root->GetElement("errors")->GetElement("glotro"));
				nGLOion[prn-1] = NoiseFactory::Create(root->GetElement("errors")->GetElement("gloion"));
			}

			// PROCESS THE RINEX AND SP3 FILES FOR DATA ///////////////////////////

			// Will be useful for parsing the sdf
			sdf::ElementPtr el;

			// So that we don't use bad satellite positions as the truth
			sp3Ephemerides.rejectBadPositions(true);
			sp3Ephemerides.rejectBadClocks(true);

			// Extract any
			try
			{
				el = root->GetElement("ephemerides")->GetElement("final")->GetElement("gps")->GetFirstElement();
				do
				{
					// Obtain the URI
					std::string uri;
					el->GetValue()->Get(uri);

					// Open and store final ephemerides (Note that all times are GPS)
					sp3Ephemerides.loadSP3File(
						common::SystemPaths::Instance()->FindFileURI(uri).c_str());
				}
				while(el = el->GetNextElement());
			}
			catch (const std::exception& e)
			{
				ROS_DEBUG("Could not obtain GPS final ephemerides: %s",e.what());
			}

			try
			{
				el = root->GetElement("ephemerides")->GetElement("final")->GetElement("glo")->GetFirstElement();
				do
				{
					// Obtain the URI
					std::string uri;
					el->GetValue()->Get(uri);

					// Open and store final ephemerides
					sp3Ephemerides.loadSP3File(
						common::SystemPaths::Instance()->FindFileURI(uri).c_str());
				}
				while(el = el->GetNextElement());
			}
			catch (const std::exception& e)
			{
				ROS_DEBUG("Could not obtain final Glonass ephemerides: %s",e.what());
			}

			// OPEN AND STORE GPS AND GLONASS BROADCAST EPHEMERIDES /////////////////////////////////////////////

			try
			{
				// This is not how I would have chosen to setup the API...
				el = root->GetElement("ephemerides")->GetElement("broadcast")->GetElement("gps")->GetFirstElement();
				do
				{
					// Obtain the URI
					std::string uri;
					el->GetValue()->Get(uri);

					// Read nav file and store unique list of ephemerides
					gpstk::Rinex3NavStream gps_rnffs(
						common::SystemPaths::Instance()->FindFileURI(uri).c_str());
					gpstk::Rinex3NavData   gps_rne;
					gpstk::Rinex3NavHeader gps_hdr;

					// Let's read the header (may be skipped)
					gps_rnffs >> gps_hdr;

					// Storing the ephemeris in "bcstore"
					while (gps_rnffs >> gps_rne) 
					   gpsEphemerides.addEphemeris(gps_rne);

				}
				while(el = el->GetNextElement());
			}
			catch (const std::exception& e)
			{
				ROS_WARN("Could not obtain broadcast GPS ephemerides: %s",e.what());
			}

			try
			{
				// This is not how I would have chosen to setup the API...
				el = root->GetElement("ephemerides")->GetElement("broadcast")->GetElement("glo")->GetFirstElement();
				do
				{
					// Obtain the URI
					std::string uri;
					el->GetValue()->Get(uri);

					// Read nav file and store unique list of ephemerides
					gpstk::Rinex3NavStream glo_rnffs(common::SystemPaths::Instance()->FindFileURI(uri).c_str());
					gpstk::Rinex3NavData   glo_rne;
					gpstk::Rinex3NavHeader glo_hdr;

					// Let's read the header (may be skipped)
					glo_rnffs >> glo_hdr;

					// Storing the ephemeris in "bcstore"
					while (glo_rnffs >> glo_rne) 
					   gloEphemerides.addEphemeris(glo_rne);

				}
				while(el = el->GetNextElement()); 
			}
			catch (const std::exception& e)
			{
				ROS_DEBUG("Could not obtain any broadcast Glonass ephemerides: %s",e.what());
			}

			// OPEN AND STORE IONOSPHERERE INFORMATION /////////////////////////////////////////////

			try
			{
				// This is not how I would have chosen to setup the API...
				el = root->GetElement("ionosphere")->GetFirstElement();
				do
				{
					// Obtain the URI
					std::string uri;
					el->GetValue()->Get(uri);

					// Open meteorological data file
					gpstk::IonexStream is(common::SystemPaths::Instance()->FindFileURI(uri).c_str()); 
					gpstk::IonexHeader ih;
					gpstk::IonexData   id;

					// Let's read the header (may be skipped)
					is >> ih;

					// Storing the ephemeris in "bcstore"
					while (is >> id) 
					{
						id.time.setTimeSystem(gpstk::TimeSystem::GPS);
						tecStore.addMap(id);
					}
				}
				while (el = el->GetNextElement());
			}
			catch (const std::exception& e)
			{
				ROS_DEBUG("Could not obtain ionex data: %s",e.what());
			}

			// SETUP MESSAGING //////////////////////////////////////////////////////////////////
			
			// Create and initialize a new Gazebo transport node
			nodePtr = transport::NodePtr(new transport::Node());
			nodePtr->Init(worldPtr->GetName());

			// Broadcast messages on the /global/satellites topic
			pubPtr = nodePtr->Advertise<msgs::Satellites>("~/satellites");

			// Subscribe to meteorological updates
			subPtrMet = nodePtr->Subscribe("~/meteorological", 
				&Satellites::ReceiveMeteorological, this);

			// Subscribe to meteorological updates
			subPtrEnv = nodePtr->Subscribe("~/environment", 
				&Satellites::ReceiveEnvironment, this);

			// Subscribe to meteorological updates
			subPtrNoi = nodePtr->Subscribe("~/noise", 
				&Satellites::ReceiveNoise, this);

			// ROS timer respects gazebo
			if (rate > 0)
			{
				timer = rosNode.createTimer(
					ros::Duration(1.0/rate),
					&Satellites::Update,
					this,
			        false,                                      // Oneshot
			        true                                        // Autostart
				);     
			}  

			// Reset module
			Reset();
        }

        // All sensors must be resettable
        void Reset()
        {
			rcvMet = false;
			rcvEnv = false;
        }

	};

	// Resgister the plugin
	GZ_REGISTER_WORLD_PLUGIN(Satellites);
}