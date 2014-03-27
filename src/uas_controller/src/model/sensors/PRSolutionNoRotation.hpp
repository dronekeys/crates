/// @file PRSolution.hpp
/// Pseudorange navigation solution, either a simple solution using all the
/// given data, or a solution including editing via a RAIM algorithm.
 
#ifndef PRS_POSITION_SOLUTION_NOROTATION_HPP
#define PRS_POSITION_SOLUTION_NOROTATION_HPP

//============================================================================
//
//  This file is part of GPSTk, the GPS Toolkit.
//
//  The GPSTk is free software; you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published
//  by the Free Software Foundation; either version 2.1 of the License, or
//  any later version.
//
//  The GPSTk is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with GPSTk; if not, write to the Free Software Foundation,
//  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
//  
//  Copyright 2004, The University of Texas at Austin
//
//============================================================================

#include <vector>
#include <ostream>
#include <gpstk/PRSolution.hpp>
#include <gpstk/GNSSconstants.hpp>
#include <gpstk/CommonTime.hpp>
#include <gpstk/RinexSatID.hpp>
#include <gpstk/Stats.hpp>
#include <gpstk/Matrix.hpp>
#include <gpstk/Namelist.hpp>
#include <gpstk/XvtStore.hpp>
#include <gpstk/TropModel.hpp>
#include <gpstk/TropModel.hpp>

namespace gpstk
{


   /// This class defines an interface to routines which compute a position
   /// and time solution from pseudorange data, with a data editing algorithm
   /// based on Receiver Autonomous Integrity Monitoring (RAIM) concepts.
   /// RAIM ref. "A Baseline GPS RAIM Scheme and a Note on the Equivalence of
   /// Three RAIM Methods," by R. Grover Brown, Journal of the Institute of
   /// Navigation, Vol. 39, No. 3, Fall 1992, pg 301.
   ///
   /// The main point of entry is RAIMCompute(); it will compute a solution given
   /// the pseudoranges from a number of satellites, using a RAIM-based algorithm
   /// to detect and exclude 'bad' data from the solution. Alternately, the user
   /// may compute a straightforward solution using all the input data, without
   /// the RAIM algorithm; this is done by first calling PreparePRSolution()
   /// and then SimplePRSolution().
   ///
   /// The class is able to use GPS and/or Glonass satellite data, and assumes that
   /// the Glonass ephemeris is in the GPS frame but the Glonass clocks are in
   /// the Glonass time frame. If the input data is mixed (both GPS and GLO) then
   /// the solution has a 5th element, namely the GPS-GLO time offset, as estimated
   /// by the algorithm. If the data is not mixed (either all-GPS or all-GLO),
   /// this element is zero and has no meaning. An all-GLO solution will have clock
   /// bias in Glonass time; otherwise it will be in GPS time.
   /// 
   /// Note that a problem can arise when processing a timeseries of mixed data by
   /// calling RAIMCompute at each epoch; it is possible that the RAIM algorithm
   /// may, at some epochs, happen to reject all the Glonass satellites or all the
   /// GPS satellites. In this case the GPS-GLO timeoffset is undefined (and returned
   /// as zero).

   class PRSolutionNoRotation
   {
   public:
         /// Constructor
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wreorder"
       PRSolutionNoRotation() throw() : RMSLimit(6.5),
                             SlopeLimit(1000.),
                             NSatsReject(-1),
                             MaxNIterations(10),
                             ConvergenceLimit(3.e-7),
                             Valid(false),
                             hasMemory(true)
         {};
#pragma clang diagnostic pop
      /// Return the status of solution
      bool isValid() const throw() { return Valid; }

      // input parameters: -------------------------------------------------

      /// RMS limit (m) on residual of fit
      double RMSLimit;

      /// Slope limit (dimensionless).
      double SlopeLimit;

      /// Maximum number of satellites that may be rejected in the RAIM algorithm;
      /// if this = -1, as many as possible will be rejected (RAIM requires at least 5
      /// satellites). A (single) non-RAIM solution can be obtained by setting this
      /// to 0 before calling RAIMCompute().
      int NSatsReject;

      /// Maximum number of iterations allowed in the linearized least squares
      /// algorithm.
      int MaxNIterations;

      /// Convergence limit (m): continue iteration loop while RSS change in
      /// solution exceeds this.
      double ConvergenceLimit;

      /// vector<SatID> containing the satellite systems included in the solution. 
      /// It should be defined before the first solution call; if it is empty at that
      /// time it will be determined by the input SatelliteIDs. It is used to
      /// determine which clock biases are included in the solution; ordering of this
      /// vector determines ordering of the clock states in the RAIM solution:
      /// Solution ~ X,Y,Z,(clks ~ SystemIDs). The SimplePRSolution is similarly
      /// defined by the vector which is returned by that routine.
      std::vector<SatID::SatelliteSystem> SystemIDs;

      /// This determines whether this object will maintain a "memory" of all the
      /// solutions it has computed. This is used for several things, including the
      /// computation of pre-fit residuals, and thus of the aposteriori variance of
      /// unit weight (APV), the number of data, solutions and degrees of freedom
      /// and a combined weighted average solution.
      bool hasMemory;

      // input and output: -------------------------------------------------

      /// vector<SatID> containing satellite IDs for all the satellites input, with
      /// bad (excluded) ones identified by (Satellite[.] < 0). This vector is saved
      /// after each call to the computation routines (SimplePRSolution and
      /// RAIMCompute) and used for printing.
      std::vector<SatID> SatelliteIDs;

      // output: -------------------------------------------------

      /// Vector<double> containing the computed position solution (3 components,
      /// ECEF in the frame of the ephemeris, meter), the receiver clock bias (m),
      /// and the GPS-GLO time offset (m). In the case of GPS-only or GLO-only
      /// data, the last element is zero and has no meaning. This vector is valid
      /// only when isValid() is true.
      /// If this vector is defined on input, it is used as an apriori position, both
      /// to initialized the iterative algorithm, and to compute position residuals
      /// after a good solution is found.
      Vector<double> Solution;

      /// Matrix<double> containing the computed solution covariance (meter^2);
      /// see doc. for Solution for the components; valid only when isValid() is true.
      Matrix<double> Covariance;

      /// Matrix<double> containing the inverse measurement covariance matrix (m^-2)
      /// that was used in computing the final solution.
      Matrix<double> invMeasCov;

      /// Matrix<double> containing the partials matrix used in the final solution.
      Matrix<double> Partials;

      /// The "memory" object, used only when hasMemory is true.
      PRSMemory memory;

      /// Prefit residuals; only valid if memory exists b/c it needs apriori solution.
      /// Vector<double> of 'pre-fit' residuals, computed by the solution routines,
      /// but only if APrioriSol is defined; equal to Partials*(Sol-APrioriSol)-Resid
      /// where Resid is the data residual vector on the first iteration.
      Vector<double> PreFitResidual;

      /// Root mean square residual of fit (except when RMSDistanceFlag is set,
      /// then RMS distance from apriori position); in meters.
      double RMSResidual;

      /// Slope computed in the RAIM algorithm (largest of all satellite values)
      /// for the returned solution, dimensionless.
      double MaxSlope;

      /// DOPs computed in a call to DOPCompute() or outputString()
      double TDOP,PDOP,GDOP;

      /// the actual number of iterations used
      int NIterations;

      /// the RSS change in solution at the end of iterations.
      double Convergence;

      /// the number of good satellites used in the final computation
      int Nsvs;

      /// if true, the solution was constructed from a mixed dataset, including
      /// both GPS and Glonass satellites. This means the Solution vector will have
      /// length 5, with the last element being the estimated GPS-GLO time offset.
      bool Mixed;

      /// if true, the returned solution may be degraded because the tropospheric
      /// correction was not applied to one or more satellites; applies after calls to
      /// both SimplePRSolution() and RAIMCompute().
      bool TropFlag;

      /// if true, the returned solution may be degraded because the RMS residual or
      /// the slope is large; applies only after calls to RAIMCompute().
      bool RMSFlag, SlopeFlag;

      // member functions -------------------------------------------

      /// Compute the satellite position / corrected range matrix (SVP) which is used
      /// by SimplePRSolution(). SVP is output, dimensioned (N,4) where N is the
      /// number of satellites and the length of both Satellite and Pseudorange.
      /// Data is ignored whenever Sats[i].id is < 0. NB caller should verify that the
      /// number of good entries (Satellite[.] > 0) is > 4 before proceeding.
      /// Even though this is a member function, it changes none of the member data.
      /// @param Tr          input Measured time of reception of the data.
      /// @param Sats        input std::vector<SatID> of satellites; satellites that
      ///                     are to be excluded by the algorithm are marked by a
      ///                     negative 'id' member; this call will mark satellites for
      ///                     which there is no ephemeris.
      /// @param Syss        std::vector<SatID::SatelliteSystem> of systems to be used
      ///                     in the computation; determines order of clocks in Sol.
      ///                     If empty, it will be filled with all systems in Sats.
      /// @param Pseudorange input std::vector<double> of raw pseudoranges (parallel
      ///                     to Sats), in meters
      /// @param pEph        input pointer to gpstk::XvtStore<SatID> to be used
      /// @param SVP         output gpstk::Matrix<double> of dimension (N,4), N is
      ///                     the number of satellites in Sats[] (marked or not),
      ///                     on output this contains the satellite positions at
      ///                     transmit time (cols 0-2), the corrected pseudorange (1).
      /// @return Return values:
      ///  >= 0 number of good satellites found
      /// -4    ephemeris not found for all the satellites
      int PreparePRSolution(const CommonTime& Tr,
                            std::vector<SatID>& Sats,
                            std::vector<SatID::SatelliteSystem>& Syss,
                            const std::vector<double>& Pseudorange,
                            const XvtStore<SatID> *pEph,
                            Matrix<double>& SVP) const throw();

      /// Compute a single autonomous pseudorange solution.
      /// On output, all the member data is filled with results.
      /// Input only (first 3 should be just as returned from PreparePRSolution()):
      /// @param Tr          const. Measured time of reception of the data.
      ///                     On output member currTime set to this.
      /// @param Sats        const std::vector<SatID> of satellites. Satellites
      ///                     that are to be excluded by the algorithm are marked by a
      ///                     negative 'id' member. Length N.
      ///                     On output member SatelliteIDs set to this.
      /// @param SVP         const Matrix<double> of dimension (N,5) contains sat.
      ///                     direction cosines and corrected pseudorange data.
      /// @param invMC       const gpstk::Matrix<double> NXN measurement covariance
      ///                     matrix inverse (meter^-2) of the pseudorange data (for N
      ///                     see Sats). If this matrix has dimension 0, no weighting
      ///                     of the data is done.
      /// @param pTropModel  pointer to a gpstk::TropModel for trop correction.
      /// @param niterLimit  integer limit on the number of iterations. On output,
      ///                     member NIterations = number of iterations actually used.
      /// @param convLimit   double convergence criterion, = RSS change in solution,
      ///                     in meters. On output, member Convergence = final value.
      /// @param Syss        std::vector<SatID::SatelliteSystem> of systems to be used
      ///                     in the computation. On output, systems actually used
      ///                     stored in member SystemIDs; this determines order of
      ///                     clock biases in Solution.
      /// Output:  (these will be resized within the function)
      /// @param Resids      gpstk::Vector<double> post-fit range residuals for each
      ///                     satellite (m), the length of this Vector is the number
      ///                     of satellites actually used (see Sats).
      /// @param Slopes      gpstk::Vector<double> slope value used in RAIM for each
      ///                     good satellite, length m.
      /// @return Return values:
      ///  0  ok      (but check TropFlag to see if trop. correction was not applied)
      /// -1  failed to converge
      /// -2  singular problem
      /// -3  not enough good data to form a solution (at least 4 satellites required)
      int SimplePRSolution(const CommonTime& Tr,
                           const std::vector<SatID>& Sats,
                           const Matrix<double>& SVP,
                           const Matrix<double>& invMC,
                           TropModel *pTropModel,
                           const int& niterLimit,
                           const double& convLimit,
                           const std::vector<SatID::SatelliteSystem>& Syss,
                           Vector<double>& Resids,
                           Vector<double>& Slopes) throw(Exception);

      /// Compute a position/time solution, given satellite PRNs and pseudoranges
      /// using a RAIM algorithm. This is the main computation done by this class.
      /// @param Tr          Measured time of reception of the data.
      /// @param Satellites  std::vector<SatID> of satellites; on successful
      ///                    return, satellites that were excluded by the algorithm
      ///                    are marked by a negative 'id' member.
      /// @param Systems     std::vector<SatID::SatelliteSystem> of systems to be
      ///                     allowed in the computation. On output, systems actually
      ///                     used are stored in member SystemIDs; this determines
      ///                     the order of clock biases in Solution and Covariance.
      /// @param Pseudorange std::vector<double> of raw pseudoranges (parallel to
      ///                    Satellite), in meters.
      /// @param invMC       gpstk::Matrix<double> NXN measurement covariance matrix
      ///                     inverse (meter^-2) of the pseudorange data (for N
      ///                     see Sats). If this matrix has dimension 0, no weighting
      ///                     of the data is done.
      /// @param pEph        pointer to gpstk::XvtStore to be used in the algorithm.
      /// @param pTropModel  pointer to gpstk::TropModel for trop correction.
      /// @return Return values:
      ///  1  solution is ok, but may be degraded; check TropFlag, RMSFlag, SlopeFlag
      ///  0  ok
      /// -1  algorithm failed to converge
      /// -2  singular problem, no solution is possible
      /// -3  not enough good data (> 4) to form a (RAIM) solution
      ///     (the 4 satellite solution might be ok)
      /// -4  ephemeris not found for all the satellites
      int RAIMCompute(const CommonTime& Tr,
                      std::vector<SatID>& Satellites,
                      std::vector<SatID::SatelliteSystem>& Systems,
                      const std::vector<double>& Pseudorange,
                      const Matrix<double>& invMC,
                      const XvtStore<SatID> *pEph,
                      TropModel *pTropModel)
         throw(Exception);

      /// Compute DOPs using the partials matrix from the last successful solution.
      /// RAIMCompute(), if successful, calls this before returning.
      /// Results stored in PRSolution::TDOP,PDOP,GDOP.
      int DOPCompute(void) throw(Exception);

      /// conveniences for printing the results of the pseudorange solution algorithm
      /// output position, error code and V/NV
      std::string outputPOSString(std::string tag, int iret=-99,
                                    const Vector<double>& Vec=PRSNullVector) throw();
      /// output {SYS clock} for all systems, error code and V/NV
      std::string outputCLKString(std::string tag, int iret=-99) throw();
      /// output info in POS and CLK
      std::string outputNAVString(std::string tag, int iret=-99,
                                    const Vector<double>& Vec=PRSNullVector) throw();
      /// output NSVdropped, Nsvs, RMS residual, TDOP, PDOP, GDOP, Slope, niter, conv,
      /// satellites, error code and V/NV
      std::string outputRMSString(std::string tag, int iret=-99) throw();
      std::string outputValidString(int iret=-99) throw();
      /// output POS, CLK and RMS strings
      std::string outputString(std::string tag, int iret=-99,
                               const Vector<double>& Vec=PRSNullVector) throw();

      /// A convenience for printing the error code (return value)
      std::string errorCodeString(int iret) throw();

      /// A convenience for printing the current configuarion
      std::string configString(std::string tag) throw();

   private:

      /// flag: output content is valid.
      bool Valid;

      /// time tag of the current solution
      CommonTime currTime;

      /// time formats used in prints
      static const std::string calfmt,gpsfmt,timfmt;

      /// empty vector used to detect default
      static const Vector<double> PRSNullVector;

   }; // end class PRSolution

   //@}

} // namespace gpstk

#endif

