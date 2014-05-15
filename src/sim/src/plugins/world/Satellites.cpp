// Simulator description format (SDF)
#include <sdf/sdf.hh>

// Boost includes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

// Gazebo includes
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// ROS includes
#include <ros/ros.h>

// GPSTk includes
#include <gpstk/CommonTime.hpp>				// Common time class
#include <gpstk/Position.hpp>				// Common position class
#include <gpstk/GNSSconstants.hpp>			// For satellite frequency definitions
#include <gpstk/SP3EphemerisStore.hpp>		// Class for storing final ephemerides
#include <gpstk/GPSEphemerisStore.hpp>		// Class for storing GPS broadcast ephemerides
#include <gpstk/GloEphemerisStore.hpp>		// Class for storing GLO broadcast ephemerides
//#include <gpstk/GloFreqIndex.hpp>			// Class for working out Glonass frequencies
#include <gpstk/IonexStore.hpp>				// Class for storing ion delay data
#include <gpstk/TropModel.hpp>				// Class for modelling tropospheric delay
#include <gpstk/IonoModel.hpp>				// Class for modelling ionospheric delay
#include <gpstk/Rinex3NavHeader.hpp>		// Classes for handling RINEX data
#include <gpstk/Rinex3NavData.hpp>
#include <gpstk/Rinex3NavStream.hpp>
#include <gpstk/IonexData.hpp> 				// Classes for handling IONEX data
#include <gpstk/IonexHeader.hpp>
#include <gpstk/IonexStream.hpp>
#include <gpstk/SP3Data.hpp>				// Classes for handling SP4 data
#include <gpstk/SP3Header.hpp>
#include <gpstk/SP3Stream.hpp>

// Receives these messages
#include "environment.pb.h"
#include "meteorological.pb.h"

// Sends these messages
#include "satellites.pb.h"

/*

namespace gazebo
{
	// Message declarations
	typedef const boost::shared_ptr<const msgs::Environment> 	EnvironmentPtr;
	typedef const boost::shared_ptr<const msgs::Meteorological> MeteorologicalPtr;

	// Class for modelling GPS satellites
	class GPS : public WorldPlugin
	{

	private:

		// Parameters 
		bool 								rcvTime;				// True when UTC start has been received
		bool 								rcvMeteorological;		// True when UTC start has been received
		double                              rate;					// Rate at which 

      	// Time messages
      	gpstk::CommonTime           		startTime;				// Current simulated time and start time (UTC)
		gpstk::CivilTime 					civilTime;				// Civilian time (year month day etc)

	    // For storing GPS and Glonass ephemerides
	    gpstk::GPSEphemerisStore    		gpsEphemerides;
	    gpstk::GPSEphemerisStore    		gloEphemerides;
	    gpstk::SP3EphemerisStore    		sp3Ephemerides;
	    
 		// Tropspheric error model
	    gpstk::GGTropModel                 	tropModel;

	    // Ionospheric model
		gpstk::IonexStore           		tecStore;

		// 'Home' position for calculating iono and tropo delays
		gpstk::Position 					originECEF;

	    // Locally store temperature, pressure and humidity (troposhperic model)
	    double 								te, pr, hu;

	    // Required for messaging
	    physics::WorldPtr 					worldPtr;
		transport::NodePtr 					nodePtr;
		transport::PublisherPtr 			pubPtr;
		transport::SubscriberPtr 			subPtrEnv, subptrMet;
		ros::Timer 							timer;

      	// Message containing information
      	msgs::Satellites 					msg;

      	// Create the time
      	gpstk::CommonTime GetTime(gpstk::TimeSystem ts)
      	{
			// Get the current UTC time
            gpstk::CommonTime currentTime = startTime;

            // Add simulation
			currentTime.addSeconds(worldPtr->GetSimTime().Double());
            
			// Add correction
            currentTime.addSeconds(
            	gpstk::TimeSystem::Correction(			
              	  	gpstk::TimeSystem::UTC, ts,
               	 	civilTime.year, civilTime.month, civilTime.day
               	)
            );

            // Set the new time system
            currentTime.setTimeSystem(ts);

            // Return the time
            return currentTime;
      	}

	    //  Called to update the world information
	    void Update(const ros::TimerEvent& event)
	    {
			// Get the time in various systems
			gpstk::CommonTime currentTimeUTC = GetTime(gpstk::TimeSystem::UTC);
			gpstk::CommonTime currentTimeGPS = GetTime(gpstk::TimeSystem::GPS);
			gpstk::CommonTime currentTimeGLO = GetTime(gpstk::TimeSystem::GLO);
	
			// Obtain the TEC grid for the current time and position
			gpstk::Triple tec;
			try
			{
				tec = tecStore.getIonexValue(currentTimeGPS, originECEF, 1); 
			}
			catch (const std::exception& e)
			{
				ROS_DEBUG("Problem setting ionospheric data: %s", e.what());
			}

			// Get the epoch from the time subsystem
			double epoch = currentTimeUTC.getDays();

			// Set the epoch
			msg.set_epoch(epoch);

			// Clear all the GPS data
			msg.clear_gps();
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

			// Publish wind information to all subscribers
			pubPtr->Publish(msg);
	    }

		// This will be called whenever a new meteorlogical topic is posted
		void ReceiveMeteorological(MeteorologicalPtr& msg)
		{
			// Set the tropospheric model parameters
			tropModel.setWeather(
				msg->temperature() - 273.15, 
				msg->pressure(), 
				msg->humidity()
			);
		}

		// This will be called whenever a new environment topic is posted
		void ReceiveEnvironment(MEnvironmentPtr& msg)
		{
			// We are now ready to broadcast
			bootstrapped = true;

			// Try to see if we can grab some weather data from the RINEX file
			try
			{
				// Set the time and time system
				startTime.set(
					environment->utc(),
					(gpstk::TimeSystem) gpstk::TimeSystem::UTC
				);

				// Convert to a human readable time
				civilTime = startTime;
	        }
			catch(const std::exception& e)
			{
				ROS_ERROR("Problem converting time: %s", e.what());
			}
		}

	public:

		// Constructor
		Satellites() : rate(1.0), rcvTime(false), rcvMeterological(false)
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

			// Subscribe to meteorological updates (for )
			subPtrMet = nodePtr->Subscribe("~/meteorological", 
				&Satellites::ReceiveMeteorological, this);

			// Subscribe to environment updates (for time)
			subPtrEnv = nodePtr->Subscribe("~/environment", 
				&Satellites::ReceiveEnvironment, this);
        }

	};

}

*/
