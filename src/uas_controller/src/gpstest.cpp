// System includes
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

// Used throughout GPStk to represent time
#include <gpstk/IERSConventions.hpp>
#include <gpstk/CommonTime.hpp>
#include <gpstk/UTCTime.hpp>


#include <gpstk/GPSEllipsoid.hpp>

///////// ENVIRONMENT ///////////////

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

/////////// RECEIVER /////////////////

// Classes fro modelling ionosphereic delays
#include <gpstk/IonoModel.hpp>

// Class for handling tropospheric models
#include <gpstk/TropModel.hpp>

// Class for handling RAIM
#include <gpstk/PRSolution2.hpp>

   // Class defining GPS system constants
#include <gpstk/GNSSconstants.hpp>


using namespace std;
using namespace gpstk;

// 6th day of 2010 - GPS, GLONASS and meterological data (make sure there is no nonsense in the files!)
const char* eph_gps = "/home/asymingt/Dropbox/Documents/UCL/Research/Simulator/uas_framework/src/uas_controller/resources/gnss/brdc0060.10n";
const char* eph_glo = "/home/asymingt/Dropbox/Documents/UCL/Research/Simulator/uas_framework/src/uas_controller/resources/gnss/brdc0060.10g";
const char* met_tro = "/home/asymingt/Dropbox/Documents/UCL/Research/Simulator/uas_framework/src/uas_controller/resources/gnss/hers0060.10m";
const char* met_ion = "/home/asymingt/Dropbox/Documents/UCL/Research/Simulator/uas_framework/src/uas_controller/resources/gnss/igsg0060.10i";

int main(int argc, char *argv[])
{

   // For storing ephemerides
   GPSEphemerisStore gps_ephemerides;
   GloEphemerisStore glo_ephemerides;

   // Object for GG-type tropospheric model (Goad and Goodman, 1974)
   GGTropModel ggTropModel;

   // Pointer to one of the two available tropospheric models
   TropModel *tropModelPtr = &ggTropModel;

   try
   {
      // GET THE APPROXIMATE LOCATION OF TH RECEIVERS //////////////////

      // We really want to specify the time in UTC, but satellites all have their own
      // special timekeeping systems. So , we will need to accommodate this...
      //CommonTime  ct(TimeSystem::UTC);
      //ct.set(2455203, 8760, 0.0, TimeSystem::GPS);

      CivilTime   civ(2010,1,6,3,0,0,TimeSystem::UTC);
      CommonTime  ct = civ.convertToCommonTime();

      // Position
      WGS84Ellipsoid em;
      Position       pt(-0.21052, 51.71190,  0, Position::Geocentric, &em, ReferenceFrame::Unknown);
      
      // Object for GG-type tropospheric model (Goad and Goodman, 1974)
      // Default constructor => default values for model
      GGTropModel ggTropModel;

      //////////////////////////////////////////////////////////////////
      ///////// ALL OF THIS STUFF HAPPENS IN THE WORLD PLUGIN  /////////
      //////////////////////////////////////////////////////////////////

      // OPEN AND STORE GPS EPHEMERIDES ///////////////////////////////

      // Read nav file and store unique list of ephemerides
      Rinex3NavStream gps_rnffs(eph_gps);
      Rinex3NavData   gps_rne;
      Rinex3NavHeader gps_hdr;

      // Let's read the header (may be skipped)
      gps_rnffs >> gps_hdr;

      // Storing the ephemeris in "bcstore"
      while (gps_rnffs >> gps_rne) 
         gps_ephemerides.addEphemeris(gps_rne);

      // OPEN AND STORE GLONASS EPHEMERIDES ///////////////////////////////

      // Read nav file and store unique list of ephemerides
      Rinex3NavStream glo_rnffs(eph_glo);
      Rinex3NavData   glo_rne;
      Rinex3NavHeader glo_hdr;

      // Let's read the header (may be skipped)
      glo_rnffs >> glo_hdr;

      // Storing the ephemeris in "bcstore"
      while (glo_rnffs >> glo_rne) 
         glo_ephemerides.addEphemeris(glo_rne);
        
      // METEROLOGICAL DATA /////////////////////////////////////////////

      // Open meteorological data file
      RinexMetStream rms(met_tro); 
      RinexMetHeader rmh;
      RinexMetData   rmd;

      // Let's read the header (may be skipped)
      rms >> rmh;

      // Read data into linked list
      list<RinexMetData> rml;
      while (rms >> rmd) 
         rml.push_back(rmd);

      // IONOSPHERIC DATA ///////////////////////////////////////////////

      // Open meteorological data file
      IonexStream is(met_ion); 
      IonexHeader ih;
      IonexData   id;

      // Let's read the header (may be skipped)
      is >> ih;

      // Read data into linked list
      list<IonexData> il;
      while (is >> id)
         il.push_back(id);

      //////////////////////////////////////////////////////////////////
      // GOAL -> Produce the following messages                       //
      // - /scene/epoch/{day,secondsofday}                            //
      // - /scene/temperature                                         //
      // - /scene/humidity                                            //
      // - /scene/pressure                                            //
      // - /scene/wind/{speed,direction}                              //
      // - /scene/gnss/{gps,glonass}/{sat}/{errx,erry,errz,iondelay}  //
      //////////////////////////////////////////////////////////////////

      // GPS EPHERMERIDES //////////////////////////////////////////////

      ct.setTimeSystem(TimeSystem::GPS);
      for (int prn = 1; prn <= gpstk::MAX_PRN; ++prn)
      {
         try
         {
            Xvt xvt = gps_ephemerides.getXvt(SatID(prn,SatID::systemGPS),ct);
            Vector<double> pos = xvt.getPos().toVector();
            cout << "/scene/gnss/gps/" << prn << "/x\t\t\t" << pos[0] << endl;
            cout << "/scene/gnss/gps/" << prn << "/y\t\t\t" << pos[1] << endl;
            cout << "/scene/gnss/gps/" << prn << "/z\t\t\t" << pos[2] << endl;
            cout << "/scene/gnss/gps/" << prn << "/clkbias\t\t" << xvt.getClockBias() << endl;
            cout << "/scene/gnss/gps/" << prn << "/clkdrift\t\t" << xvt.getClockDrift() << endl;
            cout << "/scene/gnss/gps/" << prn << "/relcorr\t\t" << xvt.getRelativityCorr() << endl;
         }
         catch(InvalidRequest& e)
         { 
            continue;
         }
      }
         
      ct.setTimeSystem(TimeSystem::GLO);
      for (int prn = 1; prn <= gpstk::MAX_PRN; ++prn)
      {
         try
         {
            Xvt xvt = glo_ephemerides.getXvt(SatID(prn,SatID::systemGlonass),ct);
            Vector<double> pos = xvt.getPos().toVector();
            cout << "/scene/gnss/glonass/" << prn << "/x\t\t\t" << pos[0] << endl;
            cout << "/scene/gnss/glonass/" << prn << "/y\t\t\t" << pos[1] << endl;
            cout << "/scene/gnss/glonass/" << prn << "/z\t\t\t" << pos[2] << endl;
            cout << "/scene/gnss/glonass/" << prn << "/clkbias\t\t" << xvt.getClockBias() << endl;
            cout << "/scene/gnss/glonass/" << prn << "/clkdrift\t\t" << xvt.getClockDrift() << endl;
            cout << "/scene/gnss/glonass/" << prn << "/relcorr\t\t" << xvt.getRelativityCorr() << endl;
         }
         catch(InvalidRequest& e)
         { 
            continue;
         }
      }


      // METEOROLOGICAL DATA ///////////////////////////////////////////

      // Find a weather point. Only if a meteorological RINEX file
      // was provided, the meteorological data linked list "rml" is
      // neither empty or at its end, and the time of meteorological
      // records are below observation data epoch.
      list<RinexMetData>::iterator mi = rml.begin();
      while ((!rml.empty())  && (mi!= rml.end() && (*mi).time < ct)) 
         mi++; 

      cout << "/scene/epoch/day\t\t"         << (long) ct.getDays() << endl;
      cout << "/scene/epoch/secondofday\t"   << (float) ct.getSecondOfDay() << endl;
      cout << "/scene/temperature\t\t"       << (*mi).data[RinexMetHeader::TD] << endl;
      cout << "/scene/humidity\t\t\t"        << (*mi).data[RinexMetHeader::HR] << endl;
      cout << "/scene/pressure\t\t\t"        << (*mi).data[RinexMetHeader::PR] << endl;

      // Set the weather for the troposphere model
      ggTropModel.setWeather(
         (*mi).data[RinexMetHeader::TD],
         (*mi).data[RinexMetHeader::PR],
         (*mi).data[RinexMetHeader::HR]);

      cout << "/scene/troposphere/wetdelay\t\t\t"        << ggTropModel.wet_zenith_delay() << endl;
      cout << "/scene/troposphere/drydelay\t\t\t"        << ggTropModel.dry_zenith_delay() << endl;

      // IONOSPHERIC DATA //////////////////////////////////////////////


      ct.setTimeSystem(TimeSystem::UTC);
      list<IonexData>::iterator ii = il.begin();
      cout << (*ii).time.getTimeSystem().getTimeSystem() << endl;
      while ((!il.empty())  && (ii!= il.end() && (*ii).time < ct)) 
         ii++;

      cout << "/scene/ionosphere\t\t\t"        << (*ii).getValue(pt) << endl;

   }
   catch(Exception& e)
   {
      cerr << e << endl;
   }
   catch (...)
   {
      cerr << "Caught an unexpected exception." << endl;
   }

   exit(0);

}  // End of 'main()'


/*

// GNSS constants
#include <gpstk/GNSSconstants.hpp>

// Class to store satellite precise navigation data
#include <gpstk/SP3EphemerisStore.hpp>
#include <gpstk/SP3Header.hpp>
#include <gpstk/SP3Data.hpp>
#include <gpstk/SP3Stream.hpp>
#include <gpstk/SP3SatID.hpp>

// Class for handling tropospheric models
#include <gpstk/TropModel.hpp>

// Class providing RAIM solver
#include <gpstk/PRSolution2.hpp>

using namespace std;
using namespace gpstk;

int main(void)
{
   //////////////////////////
   // HAPPENS IN GPSORBITS //
   //////////////////////////

   // Basic tropospheric model
   ZeroTropModel noTropModel;
   TropModel*    tropModelPtr = &noTropModel;

   // Position solver
   //RAIMSolution raimSolver;

   // Declare a "SP3EphemerisStore" object to handle precise ephemeris
   SP3EphemerisStore SP3EphList;

   // Load all the SP3 ephemerides files
   SP3EphList.loadFile("/home/asymingt/Dropbox/Documents/UCL/Research/Simulator/uas_framework/src/uas_controller/resources/orbits/ngs15992_16to17.sp3");

   // Reject satellites with bad or absent positional values / clocks
   SP3EphList.rejectBadPositions(true);
   SP3EphList.rejectBadClocks(true);


   // Read nav file and store unique list of ephemerides
   SP3Stream         stream("/home/asymingt/Dropbox/Documents/UCL/Research/Simulator/uas_framework/src/uas_controller/resources/orbits/ngs15992_16to17.sp3");    // Open ephemerides data file
   Rinex3NavData     data;
   Rinex3NavHeader   header;

   // Let's read the header (may be skipped)
   stream >> hdr;

   // Let's process all lines of observation data, one by one
   while( roffs >> rod )
   {

      vector<SatID>  prnVec;     // Visibility 
      vector<double> rangeVec;   // Ranges

      // The default constructor for PRSolution2 objects (like
      // "raimSolver") is to set a RMSLimit of 6.5. We change that
      // here. With this value of 3e6 the solution will have a lot
      // more dispersion.
      raimSolver.RMSLimit = 3e6;

      // In order to compute positions we need the current time, the
      // vector of visible satellites, the vector of corresponding
      // ranges, the object containing satellite ephemerides, and a
      // pointer to the tropospheric model to be applied
      raimSolver.RAIMCompute( rod.time,         // Current receiver clock time
                              prnVec,           // Visible satellites
                              rangeVec,         // Pseudoranges
                              bcestore,         // Satellite ephemerides
                              tropModelPtr);    // Tropospheric model   

      if (raimSolver.isValid())
      {
         // Vector "Solution" holds the coordinates, expressed in
         // meters in an Earth Centered, Earth Fixed (ECEF) reference
         // frame. The order is x, y, z  (as all ECEF objects)
         cout << setprecision(12);
         cout << raimSolver.Solution[0] << " " ;
         cout << raimSolver.Solution[1] << " " ;
         cout << raimSolver.Solution[2];
         cout << endl ;

      } 

   }

   exit(0);       // End of program
}
*/