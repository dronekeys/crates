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

// Class for storing "broadcast-type" ephemerides
#include <gpstk/GPSEphemerisStore.hpp>
#include <gpstk/GloEphemerisStore.hpp>
#include <gpstk/IonexStore.hpp>

// Class for handling tropospheric and ionospheric delay models
#include <gpstk/TropModel.hpp>
#include <gpstk/IonoModel.hpp>

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

using namespace std;
using namespace gpstk;

namespace uas_controller
{
  class Simulation : public gazebo::WorldPlugin
  {

  private:

    // Magnetic and gravitational field vectors
    gazebo::math::Vector3           vec_mag;
    gazebo::math::Vector3           vec_grav;

    // For storing GPS and Glonass ephemerides
    GPSEphemerisStore               gps_ephemerides;
    GloEphemerisStore               glo_ephemerides;
    IonexStore                      tec_store;

    // For storing and iterating trhough tropo and ionosphere adata
    list<RinexMetData>              ml;
    list<RinexMetData>::iterator    mi;

    // Path delay models
    GGTropModel                     tropModel;  // Goad and Goodman (1974) model
    IonoModel                       ionoModel;  // ICD-GPS-200, section 20.3.3.5.2.5

    // Earth ellipsoid
    WGS84Ellipsoid                  wgs84;

    // Current time
    CommonTime                      startTime;
    CommonTime                      currentTime;

    // Current position
    Position                        currentPos;

    // GAZEBO PRIVATE VARIABLES

    // Pointer to the current model
    gazebo::physics::WorldPtr       worldPtr;

    // Pointer to the update event connection
    gazebo::event::ConnectionPtr    conPtr;

    // Presents this as a node in the simulator
    gazebo::transport::NodePtr      nodePtr;

    // Service that helps publish data
    gazebo::transport::PublisherPtr pubPtr;

    // Message contacining wind information
    msgs::Environment               msg;

  public:
    
    Simulation() : WorldPlugin() {}

    // Called on plugin loaded
    void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr root)
    {
      // Will be useful for parsing the sdf
      sdf::ElementPtr el;

      // STORE POSITION AND REFERENCE TIME ///////////////////////////////

      // Temporary storage
      int y,m,d,h,i; double s;

      // Get the time element and important properties
      root->GetElement("utc")->GetElement("year")->GetValue()->Get(y);
      root->GetElement("utc")->GetElement("month")->GetValue()->Get(m);
      root->GetElement("utc")->GetElement("day")->GetValue()->Get(d);
      root->GetElement("utc")->GetElement("hour")->GetValue()->Get(h);
      root->GetElement("utc")->GetElement("minute")->GetValue()->Get(i);
      root->GetElement("utc")->GetElement("second")->GetValue()->Get(s);

      // Create a common time variable from the UTC info in the SDF data
      CivilTime   civ(y,m,d,h,i,s,TimeSystem::UTC);
      CommonTime  startTime = civ.convertToCommonTime();
      
      // Get the origin position
      currentPos.setEllipsoidModel(&wgs84);
      currentPos.setGeocentric(-0.21052, 51.71190,  1);

      // GET THE MAGNETIC AND GRAVITATIONAL VECTORS //////////////////////

      try
      {
        GeographicLib::MagneticModel mag("wmm2010");
        double lat = 27.99, lon = 86.93, h = 8820, t = 2012; // Mt Everest
        double Bx, By, Bz;
        mag(t,lat,lon,h,Bx,By,Bz);
        vec_mag.Set(Bx, By, Bz);
      }
      catch (const exception& e)
      {
        ROS_WARN("Could not determine magnetic field strength: %s",e.what());
      }

      try
      {
        GeographicLib::GravityModel grav("egm96");
        double lat = 27.99, lon = 86.93, h = 8820, t = 2012; // Mt Everest
        double Gx, Gy, Gz;
        grav.Gravity(lat, lon, h, Gx, Gy, Gz);
        vec_grav.Set(Gx, Gy, Gz);
      }
      catch (const exception& e)
      {
        ROS_WARN("Could not determine gravitational field strength: %s",e.what());
      }

      // Immediately, modify the gravity
      _world->GetPhysicsEngine()->SetGravity(vec_grav);

      // OPEN AND STORE GPS EPHEMERIDES //////////////////////////////////

      // This is not how I would have chosen to setup the API...
      el = root->GetElement("gnss")->GetElement("gps")->GetFirstElement();
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

        // Storing the ephemeris in "bcstore"
        while (gps_rnffs >> gps_rne) 
           gps_ephemerides.addEphemeris(gps_rne);

      }
      while(el = el->GetNextElement());

      // OPEN AND STORE GLONASS EPHEMERIDES ///////////////////////////////

      // This is not how I would have chosen to setup the API...
      el = root->GetElement("gnss")->GetElement("glonass")->GetFirstElement();
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

        // Read data into linked list
        while (rms >> rmd) 
           ml.push_back(rmd);

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

        // Storing the ephemeris in "bcstore"
        while (is >> id) 
           tec_store.addMap(id);

      }
      while(el = el->GetNextElement());

      // START PUBLISHING DATA //////////////////////////////////////////

      // Save the world pointer
      this->worldPtr = _world;

      // Create a new transport node
      this->nodePtr = gazebo::transport::NodePtr(new gazebo::transport::Node());

      // Initialize the node with the world name
      this->nodePtr->Init(_world->GetName());

      // Create a publisher on the ~/wind topic
      this->pubPtr = this->nodePtr->Advertise<msgs::Environment>("~/environment");

      // Set up callback for updating the model
      this->conPtr = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&Simulation::Update, this, _1));

    }

    // When the simulation is reset
    void Reset() 
    {
      // Set the iterator to the beginning of the linked list
      mi = ml.begin();
    }

    // Broadcast the wind parameters
    void Update(const gazebo::common::UpdateInfo & _info)
    {
      // Work out the current time, based on the start time + simulated time
      currentTime = startTime + _info.simTime.Double();

      // Switch time model to UTC
      currentTime.setTimeSystem(TimeSystem::Any);
    
      // Assemble the epoch message
      msg.mutable_epoch()->set_days(currentTime.getDays());
      msg.mutable_epoch()->set_secondsofdays(currentTime.getSecondOfDay());
      
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

      // TROPOSHPERIC DELAYS ///////////////////////////////////////////////////////////////

      try
      {
        // Seek for the data
        while ((!ml.empty())  && (mi!= ml.end() && (*mi).time < currentTime)) 
           mi++; 

        // Set the weather for the troposphere model
        tropModel.setWeather(
          (*mi).data[RinexMetHeader::TD],
          (*mi).data[RinexMetHeader::PR],
          (*mi).data[RinexMetHeader::HR]);

        // Package up the message
        msg.set_temperature((*mi).data[RinexMetHeader::TD]);
        msg.set_humidity((*mi).data[RinexMetHeader::HR]);
        msg.set_pressure((*mi).data[RinexMetHeader::PR]);
      }
      catch(InvalidRequest& e)
      {
        ROS_WARN("Problem querying weather data");
      }

      // IONOSPHERIC CONDITIONS //////////////////////////////////////////////////////////

      Triple tec;
      try
      {
        // Get the TEC count for the current receiver position and time
        tec = tec_store.getIonexValue(currentTime, currentPos.asECEF()); 
      }
      catch(InvalidRequest& e)
      {
        ROS_WARN("Problem querying ionosphere data");
      }

      // GPS EPHEMERIDES /////////////////////////////////////////////////////////////////


      CivilTime   civ(2010,1,6,3,0,0,TimeSystem::UTC);
      currentTime = civ.convertToCommonTime();

      // Clear any existing glonass
      msg.clear_gps();

      // Switch time mode
      currentTime.setTimeSystem(TimeSystem::GPS);

      // Iterate over satellites
      for (int prn = 1; prn <= gpstk::MAX_PRN; ++prn)
      {
         try
         {
            // Don't look for invalid satellites
            if (!gps_ephemerides.isPresent(SatID(prn,SatID::systemGPS)))
              continue;

            // Get the ephemeris
            Triple satellitePos = gps_ephemerides.getXvt(SatID(prn,SatID::systemGPS),currentTime).getPos();
            
            // Get the health
            int health = gps_ephemerides.getSatHealth(SatID(prn,SatID::systemGPS),currentTime);
            
            // Save to the message
            msgs::Ephemeris* gps = msg.add_gps();
            gps->set_prn(prn);
            gps->mutable_pos()->set_x(satellitePos[0]);
            gps->mutable_pos()->set_y(satellitePos[1]);
            gps->mutable_pos()->set_z(satellitePos[2]);
            gps->set_health(health);

            /*
            // Set the tropospheic delay, based on the simulation location
            gps->set_delay_trop(
              tropModel.correction(
                currentPos,                               // Current receiver position
                satellitePos,                             // Current satellite position
                currentTime                               // Time of observation
              )
            );

            // Type of ionosphere mapping function (string) 
            // (0) NONE no mapping function is applied 
            // (1) SLM Single Layer Model (IGS) 
            // (2) MSLM Modified Single Layer Model (CODE) 
            // (3) ESM Extended Slab Model (JLP)
            gps->set_delay_iono_f1(
              tec_store.getIono(
                currentPos.elvAngle(satellitePos),  // Elevation of the satellite
                tec[0],                             // Total electron count
                GPS_L1,                             // L1 GPS frequency
                "MLSM"                              // Mapping function
              ) 
            );
            gps->set_delay_iono_f2(
              tec_store.getIono(
                currentPos.elvAngle(satellitePos),  // Elevation of the satellite
                tec[0],                             // Total electron count
                GPS_L2,                             // L1 GPS frequency
                "MLSM"                              // Mapping function
              ) 
            );
            */
         }
         catch(InvalidRequest& e)
         { 
            ROS_WARN("Problem with GPS satellite %d",prn);
            continue;
         }
      }
      
      // GLONASS EPHEMERIDES ///////////////////////////////////////////////////////////////

      // Clear any existing glonass
      msg.clear_glonass();

      // Switch time mode
      currentTime.setTimeSystem(TimeSystem::GLO);

      // Iterate over possible satellites
      for (int prn = 1; prn <= gpstk::MAX_PRN; ++prn)
      {
         try
         {
            // Don't look for invalid satellites
            if (!glo_ephemerides.isPresent(SatID(prn,SatID::systemGlonass)))
              continue;

            // Get the ephemeris
            Triple satellitePos = glo_ephemerides.getXvt(SatID(prn,SatID::systemGlonass),currentTime).getPos();
            
            // Save to the message
            msgs::Ephemeris* glonass = msg.add_glonass();
            glonass->set_prn(prn);
            glonass->mutable_pos()->set_x(satellitePos[0]);
            glonass->mutable_pos()->set_y(satellitePos[1]);
            glonass->mutable_pos()->set_z(satellitePos[2]);
            glonass->set_health(1);
           
            /*

            // Set the tropospheic delay, based on the simulation location
            glonass->set_delay_trop(
              tropModel.correction(
                currentPos,                               // Current receiver position
                satellitePos,                             // Current satellite position
                currentTime                               // Time of observation
              )
            );

            // Type of ionosphere mapping function (string) 
            // (0) NONE no mapping function is applied 
            // (1) SLM Single Layer Model (IGS) 
            // (2) MSLM Modified Single Layer Model (CODE) 
            // (3) ESM Extended Slab Model (JLP)
            glonass->set_delay_iono_f1(
              tec_store.getIono(
                currentPos.elvAngle(satellitePos),                        // Elevation of the satellite
                tec[0],                                                   // Total electron count of the area
                GLONASS_L1OF_OFFSET+eph.getfreqNum()*GLONASS_L1OF_SCALE,  // L1OF GPS frequency
                "MLSM"                                                    // Mapping function
              ) 
            );
            glonass->set_delay_iono_f2(
              tec_store.getIono(
                currentPos.elvAngle(satellitePos),                        // Elevation of the satellite
                tec[0],                                                   // Total electron count of the area
                GLONASS_L2OF_OFFSET+eph.getfreqNum()*GLONASS_L2OF_SCALE,  // L2OF GPS frequency
                "MLSM"                                                    // Mapping function
              ) 
            );
            */

         }
         catch(InvalidRequest& e)
         { 
            ROS_WARN("Problem with Glonass satellite %d",prn);
            continue;
         }
      }
    

      // Publish wind information to all subscribers
      pubPtr->Publish(msg);
    }

  };

  GZ_REGISTER_WORLD_PLUGIN(Simulation)

} 