// Simulator description format (SDF)
#include <sdf/sdf.hh>

// Boost includes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

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

// ROS includes
#include <ros/ros.h>

// Custom messages
#include "environment.pb.h"
#include "meteorological.pb.h"
#include "satellites.pb.h"

// Required for noise distributions
#include "noise/NoiseFactory.h"

// Approx frequencies used to claculate ionospheric perturbation
#define GPS_L1 			1575.42e6
#define GLO_L1 			1602.00e6
#define SPEED_OF_LIGHT 	299792458.0

namespace gazebo
{
	// Convenience declarations
	typedef const boost::shared_ptr<const msgs::Meteorological> 	MeteorologicalPtr;
	typedef const boost::shared_ptr<const msgs::Environment> 		EnvironmentPtr;

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
	    std::vector<Noise*> 				nGPSeph;
	    std::vector<Noise*> 				nGPSclk;
	    std::vector<Noise*> 				nGPStro;
	    std::vector<Noise*> 				nGPSion;
	    std::vector<Noise*> 				nGLOeph;
	    std::vector<Noise*> 				nGLOclk;
	    std::vector<Noise*> 				nGLOtro;
	    std::vector<Noise*> 				nGLOion;

	    // Required for messaging
	    physics::WorldPtr 					worldPtr;
		transport::NodePtr 					nodePtr;
		transport::PublisherPtr 			pubPtr;
		transport::SubscriberPtr 			subPtrMet, subPtrEnv;
		ros::Timer 							timer;

      	// Message containing information
      	msgs::Satellites 					msg;
      	msgs::Meteorological                met;

	    //  Called to update the world information
	    void Update(const ros::TimerEvent& event)
	    {
	    	// We can onyl proceed if we know the UTC time and current weather
	    	if (!rcvMet || !rcvEnv)
	    		return;

			// TIME CACHING /////////////////////////////////////////////////////////////////////
	
			gpstk::CommonTime currentTimeUTC = GetTime(gpstk::TimeSystem::UTC);
			gpstk::CommonTime currentTimeGPS = GetTime(gpstk::TimeSystem::GPS);
			gpstk::CommonTime currentTimeGLO = GetTime(gpstk::TimeSystem::GLO);

			// TROPOSHPERIC DELAYS //////////////////////////////////////////////////////////////

			tropModel.setWeather(met.temperature() - 273.15, met.pressure(), met.relhumidity());

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

			// Set the epoch and clea rany existing satellites
			msg.set_epoch(epoch);
			msg.clear_svs();

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
						msgs::Vehicle* gps = msg.add_svs();
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
								SPEED_OF_LIGHT * (ephb.getClockBias()-eph.getClockBias())
							);

							// Set the tropospheic delay, based on the simulation location
							gps->set_err_tro(
								SPEED_OF_LIGHT *
								tropModel.correction(
									originECEF,                      	 // Current receiver position
									satellitePos,                        // Current satellite position
									currentTimeGPS                       // Time of observation
								)
							);

							// GET L1 ionosphere delay
							gps->set_err_ion(
								SPEED_OF_LIGHT *
								tecStore.getIono(
									originECEF.elvAngle(satellitePos),    // Elevation of the satellite
									tec[0],                               // Total electron count
									GPS_L1,                               // L1 GPS frequency
									"NONE"                                // Mapping function
								) 
							);
						}
						catch (const std::exception& e)
						{ 
							// Draw a new ephemeride error
							math::vector err_pos = nGPSeph[prn]->DrawVector(t);
							double       err_clk = nGPSclk[prn]->DrawVector(t);
							double       err_ion = nGPSion[prn]->DrawVector(t);
							double       err_tro = nGPStro[prn]->DrawVector(t);

							// Set the ephemeris error
							gps->mutable_err_pos()->set_x(err_pos.x);
							gps->mutable_err_pos()->set_y(err_pos.y);
							gps->mutable_err_pos()->set_z(err_pos.z);
							gps->set_err_clk(err_clk);
							gps->set_err_tro(err_tro);
							gps->set_err_ion(err_iom);
						}
					}
				}
				catch (const std::exception& e)
				{ 
				  ROS_DEBUG("Problem with GPS satellite: %s", e.what());
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
					msgs::Vehicle* glo = msg.add_svs();
					glo->set_system(gpstk::SatID::systemGPS);
					glo->set_prn(prn);

					// Write the position
					glo->mutable_pos()->set_x(satellitePos[0]);
					glo->mutable_pos()->set_y(satellitePos[1]);
					glo->mutable_pos()->set_z(satellitePos[2]);

					// Try and get the broadcast ephemeride
					try
					{
						// Get the broadcast ephemeris
						gpstk::Xvt ephb = gpsEphemerides.getXvt(
							gpstk::SatID(prn,gpstk::SatID::systemGlonass),
							currentTimeGPS
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
							SPEED_OF_LIGHT *
							tropModel.correction(
								originECEF,                      	 // Current receiver position
								satellitePos,                        // Current satellite position
								currentTimeGPS                       // Time of observation
							)
						);

						// GET L1 ionosphere delay
						glo->set_err_ion(
							SPEED_OF_LIGHT *
							tecStore.getIono(
								originECEF.elvAngle(satellitePos),    // Elevation of the satellite
								tec[0],                               // Total electron count
								GLO_L1,                               // L1 GPS frequency
								"NONE"                                // Mapping function
							) 
						);
					}
					catch (const std::exception& e)
					{ 
						// Draw a new ephemeride error
						math::vector err_pos = nGLOeph[prn]->DrawVector(t);
						double       err_clk = nGLOclk[prn]->DrawVector(t);
						double       err_ion = nGLOion[prn]->DrawVector(t);
						double       err_tro = nGLOtro[prn]->DrawVector(t);

						// Set the ephemeris error
						glo->mutable_err_pos()->set_x(err_pos.x);
						glo->mutable_err_pos()->set_y(err_pos.y);
						glo->mutable_err_pos()->set_z(err_pos.z);
						glo->set_err_clk(err_clk);
						glo->set_err_tro(err_tro);
						glo->set_err_ion(err_iom);
					}			  
				}
				catch(const std::exception& e)
				{ 
				  ROS_DEBUG("Problem with GLO satellite: %s", e.what());
				}
			}

			// Publish wind information to all subscribers
			pubPtr->Publish(msg);
	    }

	    // CALLBACKS FOR MESSAGES //////////////////////////////////////////////////

		// This will be called whenever a new meteorlogical topic is posted
		void ReceiveEnvironment(EnvironmentPtr& inmsg)
		{
			// Use copy constructor to indicate
			env = *inmsg;

			// Received an environment update message
			rcvEnv = true;
		}

		// This will be called whenever a new meteorlogical topic is posted
		void ReceiveMeteorological(MeteorologicalPtr& inmsg)
		{
			// Use copy constructor to indicate
			met = *inmsg;

			// Received an environment update message
			rcvMet = true;
		}

	public:

		// Constructor
		Satellites() : rate(1.0), rcvMet(false), rcvEnv(false)
		{
			// Do nothing
		}

        // All sensors must be configured using the current world information
        void Load(physics::WorldPtr world, sdf::ElementPtr root)
        {
			// Save the world pointer
			worldPtr = world;

			// INITIALIZE THE NOISE DISTRIBUTION GENERATOR ////////////////////////

	    	// Initialize the random number generator
	    	NoiseFactory::Init();

			// PROCESS CONFIGURATION FILE /////////////////////////////////////////

			// Basic parameters
			root->GetElement("rate")->GetValue()->Get(rate);

		    // Create a noise distribution for each satellite
		    for (int prn = 1; prn <= gpstk::MAX_PRN; ++prn)
		    {
				nGPSeph[prn] = NoiseFactory::Create(linkPtr,root->GetElement("errors")->GetElement("gpsclk"));
				nGPSclk[prn] = NoiseFactory::Create(linkPtr,root->GetElement("errors")->GetElement("gpsclk"));
				nGPStro[prn] = NoiseFactory::Create(linkPtr,root->GetElement("errors")->GetElement("gpsclk"));
				nGPSion[prn] = NoiseFactory::Create(linkPtr,root->GetElement("errors")->GetElement("gpsclk"));
				nGLOeph[prn] = NoiseFactory::Create(linkPtr,root->GetElement("errors")->GetElement("gloclk"));
				nGLOclk[prn] = NoiseFactory::Create(linkPtr,root->GetElement("errors")->GetElement("gloclk"));
				nGLOtro[prn] = NoiseFactory::Create(linkPtr,root->GetElement("errors")->GetElement("gloclk"));
				nGLOion[prn] = NoiseFactory::Create(linkPtr,root->GetElement("errors")->GetElement("gloclk"));
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
				ROS_DEBUG("Could not obtain broadcast GPS ephemerides: %s",e.what());
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

}