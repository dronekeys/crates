// System includes
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

// Required to bind to non-static methods
#include <boost/bind.hpp>

// For access to messages
#include <ros/ros.h>

///////// GEOGRAPHIC INCLUDES //////////

#include <GeographicLib/MagneticModel.hpp>
#include <GeographicLib/GravityModel.hpp>

///////// GPSTK INCLUDES ///////////////

// A common time system for comparing erht with satellites
#include <gpstk/CommonTime.hpp>
#include <gpstk/UTCTime.hpp>

// The ellipsoind of the eatch used in GPS
#include <gpstk/WGS84Ellipsoid.hpp>

// Classes for handling RINEX satellite navigation parameters (ephemerides)
#include <gpstk/Rinex3NavHeader.hpp>
#include <gpstk/Rinex3NavData.hpp>
#include <gpstk/Rinex3NavStream.hpp>

// Classes for handling RINEX files with meteorological parameters
#include <gpstk/RinexMetData.hpp>
#include <gpstk/RinexMetHeader.hpp>
#include <gpstk/RinexMetStream.hpp>

// Classes for handling RINEX files with IONEX data
#include <gpstk/IonexData.hpp>
#include <gpstk/IonexHeader.hpp>
#include <gpstk/IonexStream.hpp>

// Class to store satellite precise navigation data
#include <gpstk/SP3Data.hpp>
#include <gpstk/SP3Header.hpp>
#include <gpstk/SP3Stream.hpp>

// Class for storing "broadcast-type" ephemerides
#include <gpstk/SP3EphemerisStore.hpp>
#include <gpstk/GPSEphemerisStore.hpp>
#include <gpstk/GloEphemerisStore.hpp>
#include <gpstk/IonexStore.hpp>

// Class for handling tropospheric and ionospheric delay models
#include <gpstk/TropModel.hpp>
#include <gpstk/IonoModel.hpp>

// For shifting between GLO <-> UTC <-> GPS time
#include <gpstk/TimeSystemCorr.hpp>

///////// GAZEBO INCLUDES ///////////////

// Gazebo libraries
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

// Simulator description format (SDF)
#include <sdf/sdf.hh>

// Messages
#include "environment.pb.h"

// Frequencies used for ION correction
#define GPS_L1               1575.42e6
#define GPS_L2               1227.60e6
#define GLONASS_L1OF_OFFSET  1602.0e6
#define GLONASS_L1OF_SCALE   0.5625e6
#define GLONASS_L2OF_OFFSET  1246.0e6
#define GLONASS_L2OF_SCALE   0.4375e6

#define DEBUG                false

using namespace std;
using namespace gpstk;

namespace uas_controller
{
  class Simulation : public gazebo::WorldPlugin
  {

  private:

    // Minimum elevation for visibility
    double                          minElevation;

    // Magnetic and gravitational field vectors
    gazebo::math::Vector3           vec_mag;
    gazebo::math::Vector3           vec_grav;

    // For storing GPS and Glonass ephemerides
    GPSEphemerisStore               gps_ephemerides;
    GloEphemerisStore               glo_ephemerides;
    SP3EphemerisStore               sp3_ephemerides;
    IonexStore                      tec_store;

    // For storing and iterating trhough tropo and ionosphere adata
    list<RinexMetData>              ml;
    list<RinexMetData>::iterator    mi;

    // Path delay models
    GGTropModel                     tropModel;  // Goad and Goodman (1974) model
    IonoModel                       ionoModel;  // ICD-GPS-200, section 20.3.3.5.2.5

    // Earth ellipsoid
    WGS84Ellipsoid                  wgs84;

    // Current time of simulation starting
    CivilTime                       startTime;

    // Current position
    Position                        originPosECEF;
    Position                        originPosGeocentric;
    Position                        originPosGeodetic;

    // GAZEBO PRIVATE VARIABLES

    // Pointer to the current model
    gazebo::physics::WorldPtr       worldPtr;

    // Pointer to the update event connection
    gazebo::event::ConnectionPtr    conPtr;

    // Presents this as a node in the simulator
    gazebo::transport::NodePtr      nodePtr;

    // Service that helps publish data
    gazebo::transport::PublisherPtr pubPtr;


  public:
    
    Simulation() : WorldPlugin() {}

    // Called on plugin loaded
    void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr root)
    {
      // Save the world pointer
      worldPtr = _world;

      // STORE POSITION AND REFERENCE TIME ///////////////////////////////

      // Will be useful for parsing the sdf
      sdf::ElementPtr el;

      // Get the time element and important properties
      root->GetElement("gnss")->GetElement("minelevation")->GetValue()->Get(minElevation);

      // Temporary storage
      int y,m,d,h,i; double s;
      root->GetElement("utc")->GetElement("year")->GetValue()->Get(y);
      root->GetElement("utc")->GetElement("month")->GetValue()->Get(m);
      root->GetElement("utc")->GetElement("day")->GetValue()->Get(d);
      root->GetElement("utc")->GetElement("hour")->GetValue()->Get(h);
      root->GetElement("utc")->GetElement("minute")->GetValue()->Get(i);
      root->GetElement("utc")->GetElement("second")->GetValue()->Get(s);

      // Create a common time variable from the UTC info in the SDF data
      startTime = CivilTime(y,m,d,h,i,s,TimeSystem::UTC);
      UTCTime utcTime(y,m,d,h,i,s);

      // Get the local origin position
      gazebo::math::Vector3 msPositionLocal(0.0,0.0,0.0);
      gazebo::math::Vector3 msPositionGlobal = worldPtr->GetSphericalCoordinates()->SphericalFromLocal(msPositionLocal);
      
      // Do all neccessary coordinate transforms
      originPosGeodetic    = Position(msPositionGlobal.x, msPositionGlobal.y, msPositionGlobal.z, Position::Geodetic, &wgs84);
      originPosGeocentric  = originPosGeodetic.transformTo(Position::Geocentric);
      originPosECEF        = originPosGeodetic.asECEF();
  
      // GET THE MAGNETIC AND GRAVITATIONAL VECTORS //////////////////////

      try
      {
        GeographicLib::MagneticModel mag("wmm2010");
        mag(y,msPositionGlobal.y,msPositionGlobal.x,msPositionGlobal.z,vec_mag.x,vec_mag.y,vec_mag.z);
      }
      catch (const exception& e)
      {
        ROS_WARN("Could not determine magnetic field strength: %s",e.what());
      }

      try
      {
        GeographicLib::GravityModel grav("egm96");
        grav.Gravity(msPositionGlobal.y,msPositionGlobal.x,msPositionGlobal.z, vec_grav.x, vec_grav.y, vec_grav.z);
      }
      catch (const exception& e)
      {
        ROS_WARN("Could not determine gravitational field strength: %s",e.what());
      }

      // Immediately, modify the gravity
      _world->GetPhysicsEngine()->SetGravity(vec_grav);

      // OPEN AND STORE FINAL GPS EPHEMERIDES ////////////////////////////

      el = root->GetElement("gnss")->GetElement("gps")->GetElement("final")->GetFirstElement();
      do
      {
        // Obtain the URI
        std::string uri;
        el->GetValue()->Get(uri);

        // Open and store final ephemerides (Note that all times are GPS)
        sp3_ephemerides.loadSP3File(gazebo::common::SystemPaths::Instance()->FindFileURI(uri).c_str());
      }
      while(el = el->GetNextElement());

      // OPEN AND STORE FINAL GLONASS EPHEMERIDES ////////////////////////////

      // So that we don't get nonsense
      sp3_ephemerides.rejectBadPositions(true);
      sp3_ephemerides.rejectBadClocks(true);

      el = root->GetElement("gnss")->GetElement("glonass")->GetElement("final")->GetFirstElement();
      do
      {
        // Obtain the URI
        std::string uri;
        el->GetValue()->Get(uri);

        // Open and store final ephemerides
        sp3_ephemerides.loadSP3File(gazebo::common::SystemPaths::Instance()->FindFileURI(uri).c_str());
      }
      while(el = el->GetNextElement());

      // OPEN AND STORE GPS EPHEMERIDES //////////////////////////////////

      // This is not how I would have chosen to setup the API...
      el = root->GetElement("gnss")->GetElement("gps")->GetElement("broadcast")->GetFirstElement();
      do
      {
        // Obtain the URI
        std::string uri;
        el->GetValue()->Get(uri);

        // Read nav file and store unique list of ephemerides
        Rinex3NavStream gps_rnffs(gazebo::common::SystemPaths::Instance()->FindFileURI(uri).c_str());
        Rinex3NavData   gps_rne;
        Rinex3NavHeader gps_hdr;

        // Let's read the header (may be skipped)
        gps_rnffs >> gps_hdr;
        if (DEBUG)
          gps_hdr.dump(cout);

        // Storing the ephemeris in "bcstore"
        while (gps_rnffs >> gps_rne) 
           gps_ephemerides.addEphemeris(gps_rne);

      }
      while(el = el->GetNextElement());

      // OPEN AND STORE GLONASS EPHEMERIDES ///////////////////////////////

      // This is not how I would have chosen to setup the API...
      el = root->GetElement("gnss")->GetElement("glonass")->GetElement("broadcast")->GetFirstElement();
      do
      {
        // Obtain the URI
        std::string uri;
        el->GetValue()->Get(uri);

        // Read nav file and store unique list of ephemerides
        Rinex3NavStream glo_rnffs(gazebo::common::SystemPaths::Instance()->FindFileURI(uri).c_str());
        Rinex3NavData   glo_rne;
        Rinex3NavHeader glo_hdr;

        // Let's read the header (may be skipped)
        glo_rnffs >> glo_hdr;
        if (DEBUG)
          glo_hdr.dump(cout);

        // Storing the ephemeris in "bcstore"
        while (glo_rnffs >> glo_rne) 
           glo_ephemerides.addEphemeris(glo_rne);

      }
      while(el = el->GetNextElement());

      // METEROLOGICAL DATA /////////////////////////////////////////////

      // This is not how I would have chosen to setup the API...
      el = root->GetElement("gnss")->GetElement("meteorological")->GetFirstElement();
      do
      {
        // Obtain the URI
        std::string uri;
        el->GetValue()->Get(uri);

        // Open meteorological data file
        RinexMetStream rms(gazebo::common::SystemPaths::Instance()->FindFileURI(uri).c_str()); 
        RinexMetHeader rmh;
        RinexMetData   rmd;

        // Let's read the header (may be skipped)
        rms >> rmh;
        if (DEBUG)
          rmh.dump(cout);

        // Read data into linked list
        while (rms >> rmd)
        {
          // We need to specify that this file is in UTC
          rmd.time.setTimeSystem(TimeSystem::GPS);
          ml.push_back(rmd);
        }

      }
      while(el = el->GetNextElement());

      // Set the iterator to the beginning of the linked list
      mi = ml.begin();

      // IONOSPHERIC DATA ///////////////////////////////////////////////

      // This is not how I would have chosen to setup the API...
      el = root->GetElement("gnss")->GetElement("ionosphere")->GetFirstElement();
      do
      {
        // Obtain the URI
        std::string uri;
        el->GetValue()->Get(uri);

        // Open meteorological data file
        IonexStream is(gazebo::common::SystemPaths::Instance()->FindFileURI(uri).c_str()); 
        IonexHeader ih;
        IonexData   id;

        // Let's read the header (may be skipped)
        is >> ih;
        if (DEBUG)
          ih.dump(cout);

        // Storing the ephemeris in "bcstore"
        while (is >> id) 
        {
          id.time.setTimeSystem(TimeSystem::GPS);
          tec_store.addMap(id);
        }

      }
      while(el = el->GetNextElement());

      // START PUBLISHING DATA //////////////////////////////////////////

      // Create a new transport node
      this->nodePtr = gazebo::transport::NodePtr(new gazebo::transport::Node());

      // Initialize the node with the world name
      this->nodePtr->Init(_world->GetName());
      
      // Create a publisher on the ~/wind topic
      this->pubPtr = this->nodePtr->Advertise<msgs::Environment>("~/environment");

      // Set up callback for updating the model
      this->conPtr = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&Simulation::Update, this, _1));

      ROS_INFO("Advertised to environment");

    }

    // When the simulation is reset
    void Reset() 
    {
      // Set the iterator to the beginning of the linked list
      mi = ml.begin();
    }

    // Broadcast the wind parameters
    void Update(const gazebo::common::UpdateInfo& _info)
    {
      // Message containing information
      msgs::Environment msg;

      // TIME AND POSITION MANAGEMENT /////////////////////////////////////////////////////////

      // Get the current time tick, which is the start of the experimetn plus simulated time
      CommonTime currentTimeUTC(startTime.convertToCommonTime());
      currentTimeUTC.addSeconds(_info.simTime.Double());
      currentTimeUTC.setTimeSystem(TimeSystem::UTC);

      // GPS time is slightly different to UTC, so account for this when searching ephemerides
      CommonTime currentTimeGPS(startTime.convertToCommonTime());
      currentTimeGPS.addSeconds(_info.simTime.Double() +
        TimeSystem::Correction(TimeSystem::UTC,TimeSystem::GPS,startTime.year,startTime.month,startTime.day)
      );
      currentTimeGPS.setTimeSystem(TimeSystem::GPS);

      // As far as I can tell, glonass time is the same as UTC
      CommonTime currentTimeGLO(startTime.convertToCommonTime());
      currentTimeGLO.addSeconds(_info.simTime.Double());
      currentTimeGLO.setTimeSystem(TimeSystem::GLO);

      // Extract two numbers that completely describe the UTC time
      long day; double sod;
      currentTimeUTC.get(day, sod);

      // Assemble the epoch message
      msg.mutable_epoch()->set_days(day);
      msg.mutable_epoch()->set_secondsofdays(sod);
      
      // Assemble the wind message
      msg.mutable_wind()->set_speed(0);
      msg.mutable_wind()->set_direction(90);

      // Assemble the gravity and magnetic message
      msg.mutable_gravity()->set_x(vec_grav.x);
      msg.mutable_gravity()->set_y(vec_grav.y);
      msg.mutable_gravity()->set_z(vec_grav.z);
      msg.mutable_magnetic()->set_x(vec_mag.x);
      msg.mutable_magnetic()->set_y(vec_mag.y);
      msg.mutable_magnetic()->set_z(vec_mag.z);

      // Debug
      if (DEBUG)
      {
        ROS_INFO("MSG TIME: %ld %f", day, sod);
        ROS_INFO("UTC TIME: %f %f", currentTimeUTC.getDays(), currentTimeUTC.getSecondOfDay());
        ROS_INFO("GPS TIME: %f %f", currentTimeGPS.getDays(), currentTimeGPS.getSecondOfDay());
        ROS_INFO("GEODETIC: %f %f %f", originPosGeodetic.X(), originPosGeodetic.Y(), originPosGeodetic.Z());
        ROS_INFO("GEOCENTRIC: %f %f %f", originPosGeocentric.X(), originPosGeocentric.Y(), originPosGeocentric.Z());
        ROS_INFO("POS: %f %f %f", originPosECEF.X(), originPosECEF.Y(), originPosECEF.Z());
      }

      // TROPOSHPERIC DELAYS //////////////////////////////////////////////////////////////

      try
      {
        // Seek for the data
        while ((!ml.empty())  && (mi!= ml.end() && (*mi).time < currentTimeGPS)) 
           mi++; 

        //
        double t_c = (*mi).data[RinexMetHeader::TD] - 273.15; // Celcius <- Kelvin
        double h_p = (*mi).data[RinexMetHeader::HR];          // Relative humidity
        double p_m = (*mi).data[RinexMetHeader::PR];          // hPa -> mbar
        tropModel.setWeather(t_c,p_m,h_p);

        // Package up the message
        msg.set_temperature((*mi).data[RinexMetHeader::TD]);
        msg.set_humidity((*mi).data[RinexMetHeader::HR]);
        msg.set_pressure((*mi).data[RinexMetHeader::PR]);
        
        // Meteorlogical
        if (DEBUG)
        {
          ROS_INFO("MET: %f %f %f", 
            (*mi).data[RinexMetHeader::TD], 
            (*mi).data[RinexMetHeader::HR], 
            (*mi).data[RinexMetHeader::PR]);
        }
      }
      catch(InvalidRequest& e)
      {
        ROS_WARN("Problem querying weather data: %s", e.what().c_str());
      }

      // IONOSPHERIC CONDITIONS //////////////////////////////////////////////////////////

      Triple tec;
      try
      {
        // Get the TEC count for the current receiver position and time
        tec = tec_store.getIonexValue(currentTimeGPS, originPosGeocentric, 1); 

        // Total electron count
        if (DEBUG)
        {
          ROS_INFO("TEC: %f %f %f", tec[0], tec[1], tec[2]);
        }
      }
      catch(InvalidRequest& e)
      {
        ROS_WARN("Problem querying ionosphere data: %s", e.what().c_str());
      }

      // GPS EPHEMERIDES /////////////////////////////////////////////////////////////////

      // Clear any existing glonass
      msg.clear_gnss();

      // Iterate over possible satellite identifiers
      for (int prn = 1; prn <= gpstk::MAX_PRN; ++prn)
      {
        try
        {
          // Don't look for invalid satellites
          if (sp3_ephemerides.isPresent(SatID(prn,SatID::systemGPS)))
          {
            // Get the ephemeris
            Xvt eph = sp3_ephemerides.getXvt(SatID(prn,SatID::systemGPS),currentTimeGPS);

            // Elevation of the current satellite with respect to HOME position
            Triple satellitePos = eph.getPos();

            // Get the elevation
            double elv = originPosECEF.elvAngle(satellitePos);

            // Only count satellites above a minnimum elevation
            if (elv > minElevation)
            {          
              // Save to the message
              msgs::Ephemeris* gps = msg.add_gnss();
              gps->set_system(SatID::systemGlonass);
              gps->set_prn(prn);
              gps->set_elevation(elv);

              // Write the clock bias and relativity correction
              gps->set_clkbias(eph.getClockBias());
              gps->set_clkdrift(eph.getClockDrift());
              gps->set_relcorr(eph.getRelativityCorr());

              // Write the position
              gps->mutable_pos()->set_x(satellitePos[0]);
              gps->mutable_pos()->set_y(satellitePos[1]);
              gps->mutable_pos()->set_z(satellitePos[2]);

              // Try and get the broadcast ephemeride
              try
              {
                // Work out the ephemeris error
                Triple err = gps_ephemerides.getXvt(SatID(prn,SatID::systemGPS),currentTimeGPS).getPos() 
                           - satellitePos;

                // Set the ephemeris error
                gps->mutable_err()->set_x(err[0]);
                gps->mutable_err()->set_y(err[1]);
                gps->mutable_err()->set_z(err[2]);

              }
              catch(InvalidRequest& e)
              { 
                // Set the ephemeris error (zero = problem)
                gps->mutable_err()->set_x(0);
                gps->mutable_err()->set_y(0);
                gps->mutable_err()->set_z(0);
              }

              // Set the tropospheic delay, based on the simulation location
              gps->set_delay_trop(
                tropModel.correction(
                  originPosECEF,                       // Current receiver position
                  satellitePos,                        // Current satellite position
                  currentTimeGPS                       // Time of observation
                )
              );

              // GET L1 ionosphere delay
              gps->set_delay_iono_f1(
                tec_store.getIono(
                  elv,                                  // Elevation of the satellite
                  tec[0],                               // Total electron count
                  GPS_L1,                               // L1 GPS frequency
                  "NONE"                                // Mapping function
                ) 
              );

              // GET L2 ionosphere delay
              gps->set_delay_iono_f2(
                tec_store.getIono(
                  elv,                                  // Elevation of the satellite
                  tec[0],                               // Total electron count
                  GPS_L2,                               // L1 GPS frequency
                  "NONE"                                // Mapping function
                ) 
              );

              // Print infor about satellite
              if (DEBUG)
              {
                ROS_INFO("GPS satellite %d with elevation %f considered in view", gps->prn(), gps->elevation());
                ROS_INFO(" -- POS TRU: %f %f %f",  gps->pos().x(), gps->pos().y(), gps->pos().z());
                ROS_INFO(" -- POS ERR: %f %f %f", gps->err().x(), gps->err().y(), gps->err().z());
                ROS_INFO(" -- ATM DEL: F1: %f | F2: %f | TROP: %f", gps->delay_iono_f1(), gps->delay_iono_f2(), gps->delay_trop());
                ROS_INFO(" -- TIM ADJ: BIAS: %f | DRIFT: %f | RELCORR: %f", gps->clkbias(), gps->clkdrift(), gps->relcorr());
              }
            }
          }
        }
        catch(InvalidRequest& e)
        { 
          ROS_WARN("Problem with satellite: %s", e.what().c_str());
        }
      }

      // GLONASS EPHEMERIDES ////////////////////////////////////////////////////////////////

      // Iterate over possible satellite identifiers
      for (int prn = 1; prn <= gpstk::MAX_PRN; ++prn)
      {
        try
        {
          // Don't look for invalid satellites
          if (sp3_ephemerides.isPresent(SatID(prn,SatID::systemGlonass)))
          {
            // Get the ephemeris (not that SP3 are in GPST)
            Xvt eph = sp3_ephemerides.getXvt(SatID(prn,SatID::systemGlonass),currentTimeGPS);

            // Elevation of the current satellite with respect to HOME position
            Triple satellitePos = eph.getPos();

            // Get the elevation
            double elv = originPosECEF.elvAngle(satellitePos);

            // Only count satellites above a minnimum elevation
            if (elv > minElevation)
            {          
              // Save to the message
              msgs::Ephemeris* glonass = msg.add_gnss();
              glonass->set_system(SatID::systemGlonass);
              glonass->set_prn(prn);
              glonass->set_elevation(elv);

              // Write the clock bias and relativity correction
              glonass->set_clkbias(eph.getClockBias());
              glonass->set_clkdrift(eph.getClockDrift());
              glonass->set_relcorr(eph.getRelativityCorr());

              // Write the position
              glonass->mutable_pos()->set_x(satellitePos[0]);
              glonass->mutable_pos()->set_y(satellitePos[1]);
              glonass->mutable_pos()->set_z(satellitePos[2]);

              // Default frequency approximation
              double f = (double) prn;

              // Try and get the broadcast ephemeride
              try
              {
                // Work out the ephemeris error (note that Rinex are in UTC)
                Triple err = glo_ephemerides.getXvt(SatID(prn,SatID::systemGlonass),currentTimeGLO).getPos()
                           - satellitePos;

                // Set the ephemeris error
                glonass->mutable_err()->set_x(err[0]);
                glonass->mutable_err()->set_y(err[1]);
                glonass->mutable_err()->set_z(err[2]);

                // Set the frequency
                f = (double) glo_ephemerides.findEphemeris(SatID(prn,SatID::systemGlonass),currentTimeGLO).getfreqNum();

              }
              catch(InvalidRequest& e)
              { 
                ROS_WARN("Problem with satellite: %s", e.what().c_str());

                // Set the ephemeris error (zero = problem)
                glonass->mutable_err()->set_x(0);
                glonass->mutable_err()->set_y(0);
                glonass->mutable_err()->set_z(0);
              }

              // GET L1 ionosphere delay bsed on PRN
              glonass->set_delay_iono_f1(
                tec_store.getIono(
                  elv,                                       // Elevation of the satellite
                  tec[0],                                    // Total electron count
                  GLONASS_L1OF_OFFSET+f*GLONASS_L1OF_SCALE,  // L1 GPS frequency
                  "NONE"                                     // Mapping function
                ) 
              );

              // GET L2 ionosphere delay bsed on PRN
              glonass->set_delay_iono_f2(
                tec_store.getIono(
                  elv,                                        // Elevation of the satellite
                  tec[0],                                     // Total electron count
                  GLONASS_L2OF_OFFSET+f*GLONASS_L2OF_SCALE, // L1 GPS frequency
                  "NONE"                                      // Mapping function
                ) 
              );

              // Set the tropospheic delay, based on the simulation location
              glonass->set_delay_trop(
                tropModel.correction(
                  originPosECEF,                       // Current receiver position
                  satellitePos,                        // Current satellite position
                  currentTimeGPS                       // Time of observation
                )
              );

              // Print infor about satellite
              if (DEBUG)
              {
                ROS_INFO("GLONASS satellite %d with elevation %f considered in view", glonass->prn(), glonass->elevation());
                ROS_INFO(" -- POS TRU: %f %f %f",  glonass->pos().x(), glonass->pos().y(), glonass->pos().z());
                ROS_INFO(" -- POS ERR: %f %f %f", glonass->err().x(), glonass->err().y(), glonass->err().z());
                ROS_INFO(" -- ATM DEL: F1: %f | F2: %f | TROP: %f", glonass->delay_iono_f1(), glonass->delay_iono_f2(), glonass->delay_trop());
                ROS_INFO(" -- TIM ADJ: BIAS: %f | DRIFT: %f | RELCORR: %f", glonass->clkbias(), glonass->clkdrift(), glonass->relcorr());
              }
            }
          }
        }
        catch(InvalidRequest& e)
        { 
          ROS_WARN("Problem with satellite: %s", e.what().c_str());
        }
      }

      // Publish wind information to all subscribers
      pubPtr->Publish(msg);
    }
  };

  GZ_REGISTER_WORLD_PLUGIN(Simulation)

} 