// Simulator description format (SDF)
#include <sdf/sdf.hh>

// Boost includes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// A common time and position system
#include <gpstk/CommonTime.hpp>
#include <gpstk/Position.hpp>

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

// ROS
#include <ros/ros.h>

// Custom messages
#include "environment.pb.h"
#include "meteorological.pb.h"
#include "satellites.pb.h"

// Frequencies constants used for ionospheric correction
#define GPS_L1               1575.42e6
#define GPS_L2               1227.60e6
#define GLONASS_L1OF_OFFSET  1602.0e6
#define GLONASS_L1OF_SCALE   0.5625e6
#define GLONASS_L2OF_OFFSET  1246.0e6
#define GLONASS_L2OF_SCALE   0.4375e6

namespace gazebo
{
	class Satellites : public WorldPlugin
	{

	private:

		// BASIC PARAMETERS ////////////////////////////////////////////////////

		double                              rate, minElevation;

		// MDATA STORAGE MECHANISMS ////////////////////////////////////////////

	    // For storing GPS and Glonass ephemerides
	    gpstk::GPSEphemerisStore    		gpsEphemerides;
	    gpstk::GloEphemerisStore    		gloEphemerides;
	    gpstk::SP3EphemerisStore    		sp3Ephemerides;
	    gpstk::IonexStore           		tecStore;

 		// Goad and Goodman (1974) troppspheric model
	    gpstk::GGTropModel                 	tropModel;
		gpstk::Triple 						ionoModel;

		// Position of the world origin in ECEF
		gpstk::Position 					originECEF;

	    // Locally store temperature, pressure and humidity (troposhperic model)
	    double 								te, pr, hu;

	    // Required for messaging
	    physics::WorldPtr 					worldPtr;
		transport::NodePtr 					nodePtr;
		transport::PublisherPtr 			pubPtr;
		transport::SubscriberPtr 			subPtr;
		ros::Timer 							timer;

      	// Message containing information
      	msgs::Satellites 					msg;

	    //  Called to update the world information
	    void Update(const ros::TimerEvent& event)
	    {
			// TIME CACHING /////////////////////////////////////////////////////////////////////
	
			gpstk::CommonTime currentTimeUTC = GetTime(gpstk::TimeSystem::UTC);
			gpstk::CommonTime currentTimeGPS = GetTime(gpstk::TimeSystem::GPS);
			gpstk::CommonTime currentTimeGLO = GetTime(gpstk::TimeSystem::GLO);

			// TROPOSHPERIC DELAYS //////////////////////////////////////////////////////////////

			tropModel.setWeather(temperature - 273.15, pressure, humidity);

			// IONOSPHERIC DELAYS ///////////////////////////////////////////////////////////////

			gpstk::Triple tec;
			try
			{
				tec = tecStore.getIonexValue(currentTimeGPS, originECEF, 1); 
			}
			catch (const std::exception& e)
			{
				ROS_DEBUG("Problem setting ionospheric data: %s", e.what());
			}

			// TIME EPOCH //////////////////////////////////////////////////////////////////////

			// Get the epoch from the time subsystem
			double epoch;
			currentTimeUTC.get(epoch);

			// Set the epoch
			msg.set_epoch(epoch);

			// GPS EPHEMERIDES /////////////////////////////////////////////////////////////////

			// Clear any existing glonass
			msg.clear_gnss();

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

						// Get the elevation
						double elv = originECEF.elvAngle(satellitePos);

						// Only count satellites above a minnimum elevation
						if (elv > minElevation)
						{          
						  // Save to the message
						  msgs::Ephemeris* gps = msg.add_gnss();
						  gps->set_system(gpstk::SatID::systemGPS);
						  gps->set_prn(prn);

						  // Write the position
						  gps->mutable_pos()->set_x(satellitePos[0]);
						  gps->mutable_pos()->set_y(satellitePos[1]);
						  gps->mutable_pos()->set_z(satellitePos[2]);

						  // Try and get the broadcast ephemeride
						  try
						  {
						    // Get the broadcast ephemeris
						    gpstk::Xvt ephb = gpsEphemerides.getXvt(gpstk::SatID(prn,gpstk::SatID::systemGPS),currentTimeGPS);

						    // Elevation of the current satellite with respect to HOME position
						    gpstk::Triple err = ephb.getPos() - satellitePos;

						    // Set the ephemeris error
						    gps->mutable_err()->set_x(err[0]);
						    gps->mutable_err()->set_y(err[1]);
						    gps->mutable_err()->set_z(err[2]);

						    // Write the clock bias and relativity correction
						    gps->set_clkbias(ephb.getClockBias()-eph.getClockBias());
						    gps->set_relcorr(ephb.getRelativityCorr()-eph.getRelativityCorr());

						  }
						  catch (const std::exception& e)
						  { 
						    // Set the ephemeris error (zero = problem)
						    gps->mutable_err()->set_x(0);
						    gps->mutable_err()->set_y(0);
						    gps->mutable_err()->set_z(0);
						  }

						  // Set the tropospheic delay, based on the simulation location
						  gps->set_delay_trop(
						    tropModel.correction(
						      originECEF,                       // Current receiver position
						      satellitePos,                        // Current satellite position
						      currentTimeGPS                       // Time of observation
						    )
						  );

						  // GET L1 ionosphere delay
						  gps->set_delay_iono_f1(
						    tecStore.getIono(
						      elv,                                  // Elevation of the satellite
						      tec[0],                               // Total electron count
						      GPS_L1,                               // L1 GPS frequency
						      "NONE"                                // Mapping function
						    ) 
						  );

						  // GET L2 ionosphere delay
						  gps->set_delay_iono_f2(
						    tecStore.getIono(
						      elv,                                  // Elevation of the satellite
						      tec[0],                               // Total electron count
						      GPS_L2,                               // L1 GPS frequency
						      "NONE"                                // Mapping function
						    ) 
						  );
						}
					}
				}
				catch (const std::exception& e)
				{ 
				  ROS_DEBUG("Problem with GPS satellite: %s", e.what());
				}
			}

			// Iterate over possible satellite identifiers
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

				    // Get the elevation
				    double elv = originECEF.elvAngle(satellitePos);

				    // Only count satellites above a minnimum elevation
				    if (elv > minElevation)
				    {          
				      // Save to the message
				      msgs::Ephemeris* glonass = msg.add_gnss();
				      glonass->set_system(gpstk::SatID::systemGlonass);
				      glonass->set_prn(prn);

				      // Write the position
				      glonass->mutable_pos()->set_x(satellitePos[0]);
				      glonass->mutable_pos()->set_y(satellitePos[1]);
				      glonass->mutable_pos()->set_z(satellitePos[2]);

				      // Default frequency approximation
				      double f = (double) prn;

				      // Try and get the broadcast ephemeride
				      try
				      {
				        // Get the broadcast ephemeris
				        gpstk::Xvt ephb = gloEphemerides.getXvt(gpstk::SatID(prn,gpstk::SatID::systemGlonass),currentTimeGLO);

				       	// Elevation of the current satellite with respect to HOME position
				        gpstk::Triple err = ephb.getPos() - satellitePos;

				        // Set the ephemeris error
				        glonass->mutable_err()->set_x(err[0]);
				        glonass->mutable_err()->set_y(err[1]);
				        glonass->mutable_err()->set_z(err[2]);

				        // Write the clock bias and relativity correction
				        glonass->set_clkbias(ephb.getClockBias()-eph.getClockBias());
				        glonass->set_relcorr(ephb.getRelativityCorr()-eph.getRelativityCorr());

				        // Set the frequency
				        f = (double) gloEphemerides.findEphemeris(gpstk::SatID(prn,gpstk::SatID::systemGlonass),currentTimeGLO).getfreqNum();
				      }
				      catch (const std::exception& e)
				      { 
				        ROS_WARN("Problem with satellite: %s", e.what());

				        // Set the ephemeris error (zero = problem)
				        glonass->mutable_err()->set_x(0);
				        glonass->mutable_err()->set_y(0);
				        glonass->mutable_err()->set_z(0);
				      }

				      // GET L1 ionosphere delay bsed on PRN
				      glonass->set_delay_iono_f1(
				        tecStore.getIono(
				          elv,                                       // Elevation of the satellite
				          tec[0],                                    // Total electron count
				          GLONASS_L1OF_OFFSET+f*GLONASS_L1OF_SCALE,  // L1 GPS frequency
				          "NONE"                                     // Mapping function
				        ) 
				      );

				      // GET L2 ionosphere delay bsed on PRN
				      glonass->set_delay_iono_f2(
				        tecStore.getIono(
				          elv,                                        // Elevation of the satellite
				          tec[0],                                     // Total electron count
				          GLONASS_L2OF_OFFSET+f*GLONASS_L2OF_SCALE, // L1 GPS frequency
				          "NONE"                                      // Mapping function
				        ) 
				      );

				      // Set the tropospheic delay, based on the simulation location
				      glonass->set_delay_trop(
				        tropModel.correction(
				          originECEF,                       // Current receiver position
				          satellitePos,                        // Current satellite position
				          currentTimeGPS                       // Time of observation
				        )
				      );
				    }
				  }
				}
				catch(const std::exception& e)
				{ 
				  ROS_DEBUG("Problem with Glonass satellite: %s", e.what());
				}
			}

			// Publish wind information to all subscribers
			pubPtr->Publish(msg);
	    }

		// This will be called whenever a new meteorlogical topic is posted
		void ReceiveMeteorological(MeteorologicalPtr& meteorological)
		{
			te = meteorological->temperature();
			pr = meteorological->pressure();
			hu = meteorological->humidity();
		}

	public:

		// Constructor
		Satellites() : te(273), pr(1000.0), hu(95.0), minElevation(15.0), rate(1.0)
		{
			// Do nothing
		}

        // All sensors must be configured using the current world information
        void Load(physics::WorldPtr world, sdf::ElementPtr root)
        {
			// Save the world pointer
			worldPtr = world;

			// Basic parameters
			root->GetElement("rate")->GetValue()->Get(rate);
			root->GetElement("minelevation")->GetValue()->Get(minElevation);

			// Will be useful for parsing the sdf
			sdf::ElementPtr el;

			// So that we don't use bad satellite positions as the truth
			sp3Ephemerides.rejectBadPositions(true);
			sp3Ephemerides.rejectBadClocks(true);

			// Extract any
			try
			{
				el = root->GetElement("gps")->GetElement("final")->GetFirstElement();
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
				el = root->GetElement("glonass")->GetElement("final")->GetFirstElement();
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
				el = root->GetElement("gps")->GetElement("broadcast")->GetFirstElement();
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
				ROS_DEBUG("Could not obtain broadcast GPS ephemerides: %s",e.what());
			}

			try
			{
				// This is not how I would have chosen to setup the API...
				el = root->GetElement("glonass")->GetElement("broadcast")->GetFirstElement();
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
				ROS_DEBUG("Could not obtain any broadcast Glonass ephemerides: %s",e.what());
			}

			// RESET THIS MODULE //////////////////////////////////////////////////////////////////

			// Issue a reset
			Reset();
        }

        // All sensors must be resettable
        void Reset()
        {
			// Create and initialize a new Gazebo transport node
			nodePtr = transport::NodePtr(new transport::Node());
			nodePtr->Init(worldPtr->GetName());

			// Broadcast messages on the /global/satellites topic
			pubPtr = nodePtr->Advertise<msgs::Satellites>("~/satellites");

			// Subscribe to meteorological updates
			subPtr = nodePtr->Subscribe("~/meteorological", 
				&Satellites::ReceiveMeteorological, this);
        }

	};

}