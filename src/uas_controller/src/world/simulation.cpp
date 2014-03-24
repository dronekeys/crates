// System includes
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

// Required to bind to non-static methods
#include <boost/bind.hpp>

// For access to messages
#include <ros/ros.h>

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

// Class for handling tropospheric models
#include <gpstk/TropModel.hpp>

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

using namespace std;
using namespace gpstk;

namespace uas_controller
{
  class Simulation : public gazebo::WorldPlugin
  {

  private:

    // GPSTK PRIVATE VARIABLES

    // For storing GPS and Glonass ephemerides
    GPSEphemerisStore               gps_ephemerides;
    GloEphemerisStore               glo_ephemerides;

    // For storing and iterating trhough tropo and ionosphere adata
    list<RinexMetData>              ml;
    list<RinexMetData>::iterator    mi;
    list<IonexData>                 il;
    list<IonexData>::iterator       ii;

    // Object for Goad and Goodman (1974) model
    GGTropModel                     ggTropModel;

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
      currentPos.setGeocentric(-0.21052, 51.71190,  0);

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

        // Read data into linked list
        while (is >> id)
           il.push_back(id);

      }
      while(el = el->GetNextElement());

      // Set the iterator to the beginning of the linked list
      ii = il.begin();

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
      
      // Set the iterator to the beginning of the linked list
      ii = il.begin();

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

      // TROPOSHPERIC DELAYS ///////////////////////////////////////////////////////////////

      try
      {
        // Seek for the data
        while ((!ml.empty())  && (mi!= ml.end() && (*mi).time < currentTime)) 
           mi++; 

        // Set the weather for the troposphere model
        ggTropModel.setWeather(
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

      // IONOSPHERIC DELAYS ///////////////////////////////////////////////////////////////

      try
      {
        // Seek for the data
        while ((!il.empty())  && (ii!= il.end() && (*ii).time < currentTime)) 
           ii++;

        // Package up the message
        msg.set_tec((*ii).getValue(currentPos));
      }
      catch(InvalidRequest& e)
      {
        ROS_WARN("Problem querying ionospheric data");
      }

      // GPS EPHEMERIDES /////////////////////////////////////////////////////////////////

      // Clear any existing glonass
      msg.clear_gps();

      // Switch time mode
      currentTime.setTimeSystem(TimeSystem::GPS);

      // Iterate over satellites
      for (int prn = 1; prn <= gpstk::MAX_PRN; ++prn)
      {
         try
         {
            // Get the ephemeride information
            Xvt xvt = gps_ephemerides.getXvt(SatID(prn,SatID::systemGPS),currentTime);
            Vector<double> pos = xvt.getPos().toVector();

            // Save to the message
            msgs::Ephemeris* gps = msg.add_gps();
            gps->set_prn(prn);
            gps->set_x(pos[0]);
            gps->set_y(pos[1]);
            gps->set_z(pos[2]);
            gps->set_clkbias(xvt.getClockBias());
            gps->set_clkdrift(xvt.getClockDrift());
            gps->set_relcorr(xvt.getRelativityCorr());
            gps->set_health(true);
         }
         catch(InvalidRequest& e)
         { 
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
            // Get the ephemeris information
            Xvt xvt = glo_ephemerides.getXvt(SatID(prn,SatID::systemGlonass),currentTime);
            Vector<double> pos = xvt.getPos().toVector();
            
            // Save to the message
            msgs::Ephemeris* glonass = msg.add_glonass();
            glonass->set_prn(prn);
            glonass->set_x(pos[0]);
            glonass->set_y(pos[1]);
            glonass->set_z(pos[2]);
            glonass->set_clkbias(xvt.getClockBias());
            glonass->set_clkdrift(xvt.getClockDrift());
            glonass->set_relcorr(xvt.getRelativityCorr());
            glonass->set_health(true);
         }
         catch(InvalidRequest& e)
         { 
            continue;
         }
      }

      // Publish wind information to all subscribers
      pubPtr->Publish(msg);
    }

  };

  GZ_REGISTER_WORLD_PLUGIN(Simulation)

} 