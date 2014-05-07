// STandar dincludes
#include <vector>

// GNSS basics
#include <gpstk/GNSSconstants.hpp>  // For the speed of light
#include <gpstk/WGS84Ellipsoid.hpp> // GPS ellipsoid
#include <gpstk/TropModel.hpp>      // Tropospheric model
#include <gpstk/GPSEllipsoid.hpp>   // Ellipsoid model
#include <gpstk/PRSolution.hpp>   	// RAIM solution

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// For logging
#include <ros/ros.h>

// Satellites message
#include "environment.pb.h"			// For access to UTC start time
#include "meteorological.pb.h"		// For troposhperic modelling
#include "satellites.pb.h"			// For ephemerides and ionospheric delay

#define DEBUG TRUE

namespace gazebo
{
    // Convenience declaraion
    typedef const boost::shared_ptr<const msgs::Satellites> 		SatellitesPtr;
    typedef const boost::shared_ptr<const msgs::Meteorological> 	MeteorologicalPtr;
    typedef const boost::shared_ptr<const msgs::Environment> 		EnvironmentPtr;

    // Provides GNSS receiver functionality
	class GNSS : public ModelPlugin
	{

  	private:

		// Parameters
		std::string gps_typ, glo_typ;
		double gps_sig, gps_clk, gps_rel, gps_rot, gps_eph, gps_tro, gps_ion;
		double glo_sig, glo_clk, glo_rel, glo_rot, glo_eph, glo_tro, glo_ion;

		// Pointer to the current model
		physics::ModelPtr       					modPtr;

        // For gazebo messaging
        transport::NodePtr 							nodePtr;
        transport::SubscriberPtr 					subPtrEnv, subPtrMet, subPtrSat;

		// Used to convert from gazebo local to spherical
		gpstk::WGS84Ellipsoid           			wgs84;

		// We will use this to model earth rotation
		gpstk::GPSEllipsoid             			ellip;

		// RAIM solver without rotation correction (NB)
		gpstk::PRSolution            				solver;

		// Tropospheric correction model
		gpstk::ZeroTropModel            			tropModelZero;
		gpstk::TropModel*               			tropModel;

		// Current spherical position in gazebo world
		math::Vector3           					msPositionGlobal;

		// Current spherical / ECEF position in gpstk world
		gpstk::Position                				msPosGeodetic;
		gpstk::Position                 			msPosGeocentric;
		gpstk::Position                 			msPosECEF;

		// SoSlution information
		std::vector<gpstk::SatID::SatelliteSystem>  systems;        // List of satellite systems
		std::vector<gpstk::SatID>                   satellites;     // List of satellite vehicles
		gpstk::Matrix<double>                  		covariance;     // For weighting variou solutions!
		gpstk::Matrix<double>                  		ephemerides;    // All ephemerides
		gpstk::Matrix<double>                  		SVD;            // Solution vector 
		gpstk::Vector<double>                  		resids;         // Residual errors 
		gpstk::Vector<double>                  		slopes;         // Residual slopes

		// Intermediary store
		gpstk::CommonTime              				currentTime;
		gpstk::CommonTime              				oldTime;
		gpstk::Position                 			oldSolution;
		bool                            			statusFix;
		bool                            			statusDOP;

		// Use the UTC time to update the internal representation of time
		void ReceiveEnvironment(EnvironmentPtr env)
		{
			
		}

		// Use the temp, pressure and humidity to calculate local tropospheric delays
		void ReceiveMeteorological(MeteorologicalPtr env)
		{
			
		}

		// Calculate a GNSS solution
		void ReceiveSatellites(SatellitesPtr env)
		{
			// BACKUP THE LAST POSITION AND TIME FOR VELOCITY CALCULATION /////////////////////
			oldTime = currentTime;
			oldSolution = gpstk::Position(
				solver.Solution[0],
				solver.Solution[1],
				solver.Solution[2]
			);

			// DETERMINE THE CURRENT TIME ////////////////////////////////////////////////////

			currentTime.set(env->epoch(),(gpstk::TimeSystem) gpstk::TimeSystem::UTC);

			// DETERMINE THE CURRENT POSITION ////////////////////////////////////////////////

		    msPositionGlobal = modPtr->GetWorld()->GetSphericalCoordinates()
		    	->SphericalFromLocal(modPtr->GetLink("body")->GetWorldPose().pos);
		      
		    // Convert the position from geotetic spherical to geocentric cartesian
		  	msPosGeodetic    = gpstk::Position(
		  		msPositionGlobal.x, 
		  		msPositionGlobal.y, 
		  		msPositionGlobal.z, 
		  		gpstk::Position::Geodetic, 
		  		&wgs84
			);
			msPosGeocentric  = msPosGeodetic.transformTo(gpstk::Position::Geocentric);
		    msPosECEF        = msPosGeocentric.asECEF();

		    // Clear the satellite list
			satellites.clear();
			
			// Make the ephemerides container big enough to store all SV information
			ephemerides.resize(env->gnss_size(),4,0.0);

		    // Count the useful satellites
		    int counter = 0;

		    // Create a store f the
			for (int i = 0; i < env->gnss_size(); i++)
			{
				// Work out the range
				try
				{
					// Satellite ID and system
					gpstk::SatID sid(env->gnss(i).prn(),(gpstk::SatID::SatelliteSystem)env->gnss(i).system());

					// Param mapping		
					bool rot, eph, tro, rel, ion, clk, sig;
					std::string F1, F2, typ;
					switch (env->gnss(i).system())
					{
					case ((int)gpstk::SatID::systemGPS) :
						rot = gps_rot; eph = gps_eph; tro = gps_tro;
						rel = gps_rel; ion = gps_ion; clk = gps_clk;
						sig = gps_sig; typ = gps_typ;
						F1  = "L1"; F2  = "L2";
						break;

					case ((int)gpstk::SatID::systemGlonass) :
						rot = glo_rot; eph = glo_eph; tro = glo_tro;
						rel = glo_rel; ion = glo_ion; clk = glo_clk;
						sig = glo_sig; typ = glo_typ;
						F1  = "L1O"; F2  = "L2O"; 
						break;
					}

					// DONT ADD A RECORD IF THE SYSTEM IS DISABLED ////////////////////

					if (typ.compare("DISABLED")==0)
						continue;

					// POSITION AND ERROR ESTIMATION /////////////////////////////////

					// Get the true position
					gpstk::Position svPosECEF = gpstk::Position(
						env->gnss(i).pos().x(),
						env->gnss(i).pos().y(),
						env->gnss(i).pos().z(), 
						gpstk::Position::Cartesian
					);

					// Get the true range
					double trurange = range(msPosECEF,svPosECEF);

					// Initialise the pseudorange
					double pseudorange = trurange;

					// DEAL WITH ROTATIONAL ERROR ////////////////////////////////////////

					if (rot < 0)
					{
		               // True time of flight
		               double rho = trurange / gpstk::C_MPS;

		               // cAngle change in radians
		               double wt = ellip.angVelocity()*rho;
		               
		               // Update the satellite position to reflect error
		               svPosECEF = gpstk::Position( 
		                	::cos(wt)*svPosECEF[0] + ::sin(wt)*svPosECEF[1],
		               	   -::sin(wt)*svPosECEF[0] + ::cos(wt)*svPosECEF[1],
		               		svPosECEF[2]);
					}
					else
						pseudorange += math::Rand::GetDblNormal(0,rot);

					// DEAL WITH EPHEMERIS ERROR /////////////////////////////////////////

					if (eph < 0)
					{
						// Get the broadcast position
						svPosECEF += gpstk::Position(
							env->gnss(i).err().x(),
							env->gnss(i).err().y(),
							env->gnss(i).err().z()
						);
					}
					else
						pseudorange += math::Rand::GetDblNormal(0,eph);


					// TROPOSPHERIC DELAY ///////////////////////////////////////////////

					if (tro < 0)
						pseudorange += env->gnss(i).delay_trop();
					else
						pseudorange += math::Rand::GetDblNormal(0,tro);

					// IONOSPHERIC DELAY ////////////////////////////////////////////////
								
					if (ion < 0)
					{
						if (typ.compare(F1)==0)
							pseudorange += env->gnss(i).delay_iono_f1();
						if (typ.compare(F2)==0)
							pseudorange += env->gnss(i).delay_iono_f2();
					}
					else
						pseudorange += math::Rand::GetDblNormal(0,ion);

					// RELATIVITY DELAY /////////////////////////////////////////////////

					if (rel < 0)
						pseudorange += env->gnss(i).relcorr() * gpstk::C_MPS;
					else
						pseudorange += math::Rand::GetDblNormal(0,rel);

					// CLOCK DELAY //////////////////////////////////////////////////////

					if (clk < 0)
						pseudorange += env->gnss(i).clkbias() * gpstk::C_MPS;
					else
						pseudorange += math::Rand::GetDblNormal(0,clk);

					// CLOCK DELAY //////////////////////////////////////////////////////

					if (sig > 0)
						pseudorange += math::Rand::GetDblNormal(0,sig);

					// SUMMARY //////////////////////////////////////////////////////////

					if (DEBUG)
					{
						ROS_INFO("-- Satellite %d : true range %f with pseudorange error %f",
							i, trurange, pseudorange - trurange);
					}

					// DATA PREPARATION //////////////////////////////////////////////////
		         	
					// Add satellite
					satellites.push_back(sid);

		         	// Add ephemeride
		         	ephemerides(counter,0) = svPosECEF[0];	// Erronous satellite X
		         	ephemerides(counter,1) = svPosECEF[1];
		         	ephemerides(counter,2) = svPosECEF[2];
		         	ephemerides(counter,3) = pseudorange;	// Erroneous pseudorange
		         	counter++;

				}
				catch (gpstk::Exception &e)
				{
					ROS_WARN("Processing exception : %s", e.what().c_str());
				}
			}


			if (DEBUG)
			{
				ROS_INFO("Number of satellites in view is %d ",counter);
			}

			// Resize the matrix
			SVD.resize(counter,4,0.0);
			for (int i = 0; i < counter; i++)
				for (int j = 0; j < 4; j++)
					SVD(i,j) = ephemerides(i,j);

			// Get a solution!
		  	try
		  	{

				// We want a once-off solution (segfaults if on)
				solver.hasMemory = false;
				solver.RMSLimit  = 3e6;

				// These wills tore the resifuals and slopes after convergence
			  	resids.resize(counter);
			  	slopes.resize(counter);
			 
			  	// Position solution
				statusFix =  solver.SimplePRSolution(
					currentTime,
					satellites,
					SVD,
					covariance,
					tropModel,
					6,
					3.e-7,
					systems,
					resids,
					slopes);
				
				// Dilution of precision
				statusDOP = solver.DOPCompute();


				// Finished!
				if (DEBUG)
				{

					// Get the position
					gpstk::Position solPosECEF = gpstk::Position(
						solver.Solution[0],
						solver.Solution[1],
						solver.Solution[2]
					);

					// Get the error
					gpstk::Position errPosECEF = solPosECEF - msPosECEF;

					// DEBUG!
					ROS_INFO("Solver finished in %d iterations with SOL status %d and DOP status %d", solver.NIterations, statusFix, statusDOP);
					ROS_INFO("-- POS: X:%f Y:%f Z:%f", solPosECEF.X(), solPosECEF.Y(), solPosECEF.Z());
					ROS_INFO("-- ERR: X:%f Y:%f Z:%f", errPosECEF.X(), errPosECEF.Y(), errPosECEF.Z());
					ROS_INFO("-- DOP: T:%f P:%f G:%f", solver.TDOP, solver.PDOP, solver.GDOP);
				}
			}
			catch (gpstk::Exception &e)
			{
				ROS_WARN("Solver exception : %s", e.what().c_str());
			}
		}

	public:

	    // Default constructor
	    GNSS() : 
	    	tropModel(&tropModelZero),
			gps_typ("L1"), 	 gps_sig(0.0), gps_clk(0.0), gps_rel(0.0), gps_rot(0.0), gps_eph(0.0), gps_tro(0.0), gps_ion(0.0),
			glo_typ("DUAL"), glo_sig(0.0), glo_clk(0.0), glo_rel(0.0), glo_rot(0.0), glo_eph(0.0), glo_tro(0.0), glo_ion(0.0)
	    {
	      // Do nothing
	    }

	    // REQUIRED METHODS

	    // All sensors must be configured using the current model information and the SDF
	    void Load(physics::ModelPtr model, sdf::ElementPtr root)
	    {
			// Save the model pointer
			modPtr = model;

			// GPS Parameters
			root->GetElement("gps")->GetElement("signal")->GetValue()->Get(gps_typ);
			root->GetElement("gps")->GetElement("corrections")->GetElement("sig")->GetValue()->Get(gps_sig);
			root->GetElement("gps")->GetElement("corrections")->GetElement("clk")->GetValue()->Get(gps_clk);
			root->GetElement("gps")->GetElement("corrections")->GetElement("rel")->GetValue()->Get(gps_rel);
			root->GetElement("gps")->GetElement("corrections")->GetElement("rot")->GetValue()->Get(gps_rot);
			root->GetElement("gps")->GetElement("corrections")->GetElement("eph")->GetValue()->Get(gps_eph);
			root->GetElement("gps")->GetElement("corrections")->GetElement("tro")->GetValue()->Get(gps_tro);
			root->GetElement("gps")->GetElement("corrections")->GetElement("ion")->GetValue()->Get(gps_ion);

			// GLONASS Parameters
			root->GetElement("glo")->GetElement("signal")->GetValue()->Get(glo_typ);
			root->GetElement("glo")->GetElement("corrections")->GetElement("sig")->GetValue()->Get(glo_sig);
			root->GetElement("glo")->GetElement("corrections")->GetElement("clk")->GetValue()->Get(glo_clk);
			root->GetElement("glo")->GetElement("corrections")->GetElement("rel")->GetValue()->Get(glo_rel);
			root->GetElement("glo")->GetElement("corrections")->GetElement("rot")->GetValue()->Get(glo_rot);
			root->GetElement("glo")->GetElement("corrections")->GetElement("eph")->GetValue()->Get(glo_eph);
			root->GetElement("glo")->GetElement("corrections")->GetElement("tro")->GetValue()->Get(glo_tro);
			root->GetElement("glo")->GetElement("corrections")->GetElement("ion")->GetValue()->Get(glo_ion);

			// We will be using two GNSS systems in total
			systems.push_back(gpstk::SatID::systemGPS);
			systems.push_back(gpstk::SatID::systemGlonass);

            // Initialise a node pointer
            nodePtr = transport::NodePtr(new transport::Node());
            nodePtr->Init(modPtr->GetWorld()->GetName());

            // Subscribe to messages about wind conditions
            subPtrEnv = nodePtr->Subscribe("~/environment",    &GNSS::ReceiveEnvironment,    this);
            subPtrMet = nodePtr->Subscribe("~/meteorological", &GNSS::ReceiveMeteorological, this);
            subPtrSat = nodePtr->Subscribe("~/satellites", 	   &GNSS::ReceiveSatellites,     this);

			// Issue a reset on load
			Reset();

	    }

	    // All sensors must be resettable
	    void Reset()
	    {
			// Reset the solver state
			solver = gpstk::PRSolution();
	    }


		bool GetStatusFix()
		{
			return statusFix;
		}

		bool GetStatusDOP()
		{
			return statusDOP;
		}

		// Number of satellites
		int GetNumSats()
		{
			return solver.Nsvs;
		}

		// Position
		math::Vector3 GetPosition()
		{
			return math::Vector3(
				solver.Solution[0],
				solver.Solution[1],
				solver.Solution[2]
			);
		}

		// Velocity
		math::Vector3 GetVelocity()
		{
			return math::Vector3(
				solver.Solution[0] - oldSolution[0],
				solver.Solution[1] - oldSolution[1],
				solver.Solution[2] - oldSolution[2]
			) / (currentTime - oldTime);
		}

		// Dilution of precision
		math::Vector3 GetDOP()
		{
			return math::Vector3(
				solver.TDOP,
				solver.PDOP,
				solver.GDOP
			);
		}
  };

  // Register the plugin
  GZ_REGISTER_MODEL_PLUGIN(GNSS);
}


/*
// Modification of PRSolution to drop the earth rotation
namespace gpstk
{
	class PRSolutionNoRotation : public PRSolution
	{
	public:

		SimplePRSolution(const CommonTime& T,
	                                    const vector<SatID>& Sats,
	                                    const Matrix<double>& SVP,
	                                    const Matrix<double>& invMC,
	                                    TropModel *pTropModel,
	                                    const int& niterLimit,
	                                    const double& convLimit,
	                                    const vector<SatID::SatelliteSystem>& Syss,
	                                    Vector<double>& Resids,
	                                    Vector<double>& Slopes)
	      throw(Exception)
	    {
	      if(!pTropModel) {
	         Exception e("Undefined tropospheric model");
	         GPSTK_THROW(e);
	      }
	      if(Sats.size() != SVP.rows() ||
	         (invMC.rows() > 0 && invMC.rows() != Sats.size())) {
	         LOG(ERROR) << "Sats has length " << Sats.size();
	         LOG(ERROR) << "SVP has dimension " << SVP.rows() << "x" << SVP.cols();
	         LOG(ERROR) << "invMC has dimension " << invMC.rows() << "x" << invMC.cols();
	         Exception e("Invalid dimensions");
	         GPSTK_THROW(e);
	      }
	      if(Syss.size() == 0) {
	         Exception e("Allowed satellite systems input (Syss) is empty!");
	         GPSTK_THROW(e);
	      }

	      int iret(0),k,n;
	      size_t i, j;
	      double rho,wt,svxyz[3];
	      GPSEllipsoid ellip;

	      Valid = Mixed = false;

	      try {
	         // -----------------------------------------------------------
	         // counts, systems and dimensions
	         vector<SatID::SatelliteSystem> mySyss;
	         {
	            // define the Syss (system IDs) vector, and count good satellites
	            vector<SatID::SatelliteSystem> tempSyss;
	            for(Nsvs=0,i=0; i<Sats.size(); i++) {
	               if(Sats[i].id <= 0)                          // reject marked sats
	                  continue;

	               SatID::SatelliteSystem sys(Sats[i].system);  // get this system
	               if(vectorindex(Syss, sys) == -1)             // reject disallowed sys
	                  continue;

	               Nsvs++;                                      // count it
	               if(vectorindex(tempSyss, sys) == -1)         // add unique system
	                  tempSyss.push_back(sys);
	            }

	            // must sort as in Syss
	            for(i=0; i<Syss.size(); i++)
	               if(vectorindex(tempSyss, Syss[i]) != -1)
	                  mySyss.push_back(Syss[i]);
	         }

	         // dimension of the solution vector (3 pos + 1 clk/sys)
	         const int dim(3 + mySyss.size());
	         if(dim > 4) Mixed = true;

	         // require number of good satellites to be >= number unknowns (no RAIM here)
	         if(Nsvs < dim) return -3;

	         // -----------------------------------------------------------
	         // build the measurement covariance matrix
	         Matrix<double> iMC;
	         if(invMC.rows() > 0) {
	            LOG(DEBUG) << "Build inverse MCov";
	            iMC = Matrix<double>(Nsvs,Nsvs,0.0);
	            for(n=0,i=0; i<Sats.size(); i++) {
	               if(Sats[i].id <= 0) continue;
	               for(k=0,j=0; j<Sats.size(); j++) {
	                  if(Sats[j].id <= 0) continue;
	                  iMC(n,k) = invMC(i,j);
	                  ++k;
	               }
	               ++n;
	            }
	            LOG(DEBUG) << "inv MCov matrix is\n" << fixed << setprecision(4) << iMC;
	         }

	         // -----------------------------------------------------------
	         // define for computation
	         Vector<double> CRange(Nsvs),dX(dim);
	         Matrix<double> P(Nsvs,dim,0.0),PT,G(dim,Nsvs),PG(Nsvs,Nsvs),Rotation;
	         Triple dirCos;
	         Xvt SV,RX;

	         Solution.resize(dim);
	         Covariance.resize(dim,dim);
	         Resids.resize(Nsvs);
	         Slopes.resize(Nsvs);
	         LOG(DEBUG) << " Solution dimension is " << dim << " and Nsvs is " << Nsvs;

	         // prepare for iteration loop
	         // iterate at least twice so that trop model gets evaluated
	         int n_iterate(0), niter_limit(niterLimit < 2 ? 2 : niterLimit);
	         double converge(0.0);

	         // start with solution = apriori
	         Vector<double> APSolution;
	         if(hasMemory) {
	            APSolution = Solution = memory.getAprioriSolution(mySyss);
	            LOG(DEBUG) << " apriori solution (" << Solution.size() << ") is [ "
	               << fixed << setprecision(3) << Solution << " ]";
	         }
	         else {
	            Solution = Vector<double>(dim,0.0);
	            LOG(DEBUG) << " no memory - no apriori solution";
	         }

	         // -----------------------------------------------------------
	         // iteration loop
	         do {
	            TropFlag = false;       // true means the trop corr was NOT applied

	            // current estimate of position solution
	            RX.x = Triple(Solution(0),Solution(1),Solution(2));

	            // loop over satellites, computing partials matrix
	            for(n=0,i=0; i<Sats.size(); i++) {
	               // ignore marked satellites
	               if(Sats[i].id <= 0) continue;

	               // ------------ ephemeris
	               // rho is time of flight (sec)
	               if(n_iterate == 0)
	                  rho = 0.070;             // initial guess: 70ms
	               else
	                  rho = RSS(SVP(i,0)-Solution(0),
	                           SVP(i,1)-Solution(1), SVP(i,2)-Solution(2))/ellip.c();

	               // correct for earth rotation
	               wt = ellip.angVelocity()*rho;             // radians
	               svxyz[0] =  ::cos(wt)*SVP(i,0) + ::sin(wt)*SVP(i,1);
	               svxyz[1] = -::sin(wt)*SVP(i,0) + ::cos(wt)*SVP(i,1);
	               svxyz[2] = SVP(i,2);

	               // rho is now geometric range
	               rho = RSS(svxyz[0]-Solution(0),
	                         svxyz[1]-Solution(1),
	                         svxyz[2]-Solution(2));

	               // direction cosines
	               dirCos[0] = (Solution(0)-svxyz[0])/rho;
	               dirCos[1] = (Solution(1)-svxyz[1])/rho;
	               dirCos[2] = (Solution(2)-svxyz[2])/rho;

	               // ------------ data
	               // corrected pseudorange (m) minus geometric range
	               CRange(n) = SVP(i,3) - rho;

	               // correct for troposphere and PCOs (but not on the first iteration)
	               if(n_iterate > 0) {
	                  SV.x = Triple(svxyz[0],svxyz[1],svxyz[2]);
	                  Position R,S;
	                  R.setECEF(RX.x[0],RX.x[1],RX.x[2]);
	                  S.setECEF(SV.x[0],SV.x[1],SV.x[2]);

	                  // trop
	                  double tc(R.getHeight());  // tc is a dummy here
	                  // must test R for reasonableness to avoid corrupting TropModel
	                  if(R.elevation(S) < 0.0 || tc > 100000.0 || tc < -1000.0) {
	                     tc = 0.0;
	                     TropFlag = true;        // true means failed to apply trop corr
	                  }
	                  else
	                     tc = pTropModel->correction(R,S,T);    // pTropModel not const

	                  CRange(n) -= tc;
	                  LOG(DEBUG) << "Trop " << i << " " << Sats[i] << " "
	                     << fixed << setprecision(3) << tc;

	               }  // end if n_iterate > 0

	               // get the index, for this clock, in the solution vector
	               j = 3 + vectorindex(mySyss, Sats[i].system); // Solution ~ X,Y,Z,clks

	               // find the clock for the sat's system
	               const double clk(Solution(j));
	               LOG(DEBUG) << "Clock is (" << j << ") " << clk;

	               // data vector: corrected range residual
	               Resids(n) = CRange(n) - clk;

	               // ------------ least squares
	               // partials matrix
	               P(n,0) = dirCos[0];           // x direction cosine
	               P(n,1) = dirCos[1];           // y direction cosine
	               P(n,2) = dirCos[2];           // z direction cosine
	               P(n,j) = 1.0;                 // clock

	               // ------------ increment index
	               // n is index and number of good satellites - also used for Slope
	               n++;

	            }  // end loop over satellites

	            if(n != Nsvs) {
	               Exception e("Counting error after satellite loop");
	               GPSTK_THROW(e);
	            }

	            LOG(DEBUG) << "Partials (" << P.rows() << "x" << P.cols() << ")\n"
	               << fixed << setprecision(4) << P;
	            LOG(DEBUG) << "Resids (" << Resids.size() << ") "
	               << fixed << setprecision(3) << Resids;

	            // ------------------------------------------------------
	            // compute information matrix (inverse covariance) and generalized inverse
	            PT = transpose(P);

	            // weight matrix = measurement covariance inverse
	            if(invMC.rows() > 0) Covariance = PT * iMC * P;
	            else                 Covariance = PT * P;

	            // invert using SVD
	            try {
	               Covariance = inverseSVD(Covariance);
	            }
	            catch(SingularMatrixException& sme) { return -2; }
	            LOG(DEBUG) << "InvCov (" << Covariance.rows() << "x" << Covariance.cols()
	               << ")\n" << fixed << setprecision(4) << Covariance;

	            // generalized inverse
	            if(invMC.rows() > 0) G = Covariance * PT * iMC;
	            else                 G = Covariance * PT;

	            // PG is used for Slope computation
	            PG = P * G;
	            LOG(DEBUG) << "PG (" << PG.rows() << "x" << PG.cols()
	               << ")\n" << fixed << setprecision(4) << PG;

	            n_iterate++;                        // increment number iterations

	            // ------------------------------------------------------
	            // compute solution
	            dX = G * Resids;
	            LOG(DEBUG) << "Computed dX(" << dX.size() << ")";
	            Solution += dX;

	            // ------------------------------------------------------
	            // test for convergence
	            converge = norm(dX);
	            if(n_iterate > 1 && converge < convLimit) {              // success: quit
	               iret = 0;
	               break;
	            }
	            if(n_iterate >= niter_limit || converge > 1.e10) {       // failure: quit
	               iret = -1;
	               break;
	            }

	         } while(1);    // end iteration loop
	         LOG(DEBUG) << "Out of iteration loop";

	         if(TropFlag) LOG(DEBUG) << "Trop correction not applied at time "
	                                 << printTime(T,timfmt);

	         // compute slopes and find max member
	         MaxSlope = 0.0;
	         Slopes = 0.0;
	         if(iret == 0) for(j=0,i=0; i<Sats.size(); i++) {
	            if(Sats[i].id <= 0) continue;

	            // NB when one (few) sats have their own clock, PG(j,j) = 1 (nearly 1)
	            // and slope is inf (large)
	            if(::fabs(1.0-PG(j,j)) < 1.e-8) continue;

	            for(int k=0; k<dim; k++) Slopes(j) += G(k,j)*G(k,j); // TD dim=4 here?
	            Slopes(j) = SQRT(Slopes(j)*double(n-dim)/(1.0-PG(j,j)));
	            if(Slopes(j) > MaxSlope) MaxSlope = Slopes(j);
	            j++;
	         }

	         // compute pre-fit residuals
	         if(hasMemory)
	            PreFitResidual = P*(Solution-APSolution) - Resids;

	         // Compute RMS residual (member)
	         RMSResidual = RMS(Resids);

	         // Find the maximum slope (member)
	         //MaxSlope = 0.0;
	         //for(i=0; i<Slopes.size(); i++)
	         //   if(Slopes(i) > MaxSlope)
	         //      MaxSlope = Slopes[i];

	         // save to member data
	         currTime = T;
	         SatelliteIDs = Sats;
	         SystemIDs = mySyss;
	         invMeasCov = iMC;
	         Partials = P;
	         NIterations = n_iterate;
	         Convergence = converge;
	         Valid = true;

	         return iret;
	      
	      } catch(Exception& e) { GPSTK_RETHROW(e); }

	   } // end PRSolution::SimplePRSolution

	};
}
*/
