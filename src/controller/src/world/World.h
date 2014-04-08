#ifndef UAS_CONTROLLER_MODULE_H
#define UAS_CONTROLLER_MODULE_H

// Boost includes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// GPSTK time management
#include <gpstk/CommonTime.hpp>
#include <gpstk/CivilTime.hpp>

// ROS communication subsystem (mainly for debugging)
#include <ros/ros.h>

// All the different gazebo message types
#include "meteorological.pb.h"
#include "environment.pb.h"
#include "satellites.pb.h"

namespace controller
{
    // The GNSS subsystem needs to know the meterological and world info for pseudoranges
    typedef const boost::shared_ptr<const msgs::Meteorological> MeteorologicalPtr;
    typedef const boost::shared_ptr<const msgs::Environment>    EnvironmentPtr;
    typedef const boost::shared_ptr<const msgs::Satellites>     SatellitesPtr;
    
    /*  All modules inherit from this base class. The base class provides reusable methods
        for parsing SDF files. It also provides a GetTime(-) method, which returns time  */ 
    class World
    {

    private:

        // The start time and current time, both in UTC
        static gpstk::CivilTime     startTime;
        static gpstk::CommonTime    currentTime;

    protected:

        ros::NodeHandle rosNode;

        // SDF PARSING METHODS ///////////////////////////////////////////////////////

    	// Recurse down the tree  looking for elements
    	static bool FindElement(sdf::ElementPtr &cur, std::string path)
    	{
    		// Stores the query path
    		std::vector<std::string> e;

    		// Aplit the query path string into nice tokens
    		boost::split(e, path, boost::is_any_of("."));

    		// Iterate over tokens, checking that each child exists
    		for (std::vector<std::string>::iterator i = e.begin(); i != e.end(); i++)
    		{
    			if (cur->HasElement(*i))
    				cur = cur->GetElement(*i);
    			else
    				return false;
    		}

    		// Element found
    		return true;
    	}

    	// Get a SDF double parameter
    	static double GetSDFDouble(sdf::ElementPtr cur, const char* path, double value)
    	{
			if (FindElement(cur, (std::string) path))
    			cur->GetValue()->Get(value);
    		return value;
    	}

    	// Get a SDF double parameter
    	static int GetSDFInteger(sdf::ElementPtr cur, const char* path, int value)
    	{
			if (FindElement(cur, (std::string) path))
    			cur->GetValue()->Get(value);
    		return value;
    	}

        // Get a SDF double parameter
        static std::string GetSDFString(sdf::ElementPtr cur, const char* path, std::string value)
        {
            if (FindElement(cur, (std::string) path))
                cur->GetValue()->Get(value);
            return value;
        }

        // Get a SDF double parameter
        static bool GetSDFBool(sdf::ElementPtr cur, const char* path, bool value)
        {
            if (FindElement(cur, (std::string) path))
                cur->GetValue()->Get(value);
            return value;
        }

        // METHOD FOR CHILDREN TO GET THE CURENT TIME ////////////////////////////////////   

        // Get the current time in a specific format
        static gpstk::CommonTime GetTime(const gpstk::TimeSystem &ts)
        {
            // Get the current time tick, which is the start of the experimetn plus simulated time
            gpstk::CommonTime ret = currentTime;

            // UTC -> GPS
            if (ts == gpstk::TimeSystem::GPS)
            {
                ret.addSeconds(
                    gpstk::TimeSystem::Correction(
                        gpstk::TimeSystem::UTC, gpstk::TimeSystem::GPS,        // Source & Dest
                        startTime.year, startTime.month, startTime.day         // Rough period
                    )
                );
            }

            // Set the time system
            ret.setTimeSystem(ts);

            // Return the time
            return ret;
        }


    public:

        // Constructor
        World(const char *name) : rosNode(ros::NodeHandle(name))
        {
            if (!ros::isInitialized())
                ROS_FATAL("A ROS node has not been initialized");
        }

        // ALL CHILD CLASSES HAVE ACCESS TO THE WORLD TIME //////////////////////////////

        // Initilialise  the
        static void Init(sdf::ElementPtr root)
        {
            //  Extract time from the sdf
            try
            {       
                int ye = GetSDFInteger(root,"year",2010);           // Year
                int mo = GetSDFInteger(root,"month",1);             // Month
                int da = GetSDFInteger(root,"day",6);               // Day
                int ho = GetSDFInteger(root,"hour",10);             // Hour
                int mi = GetSDFInteger(root,"minute",0);            // Minute
                double se = GetSDFDouble(root,"second",0.0);        // Second

                // Create a common time variable from the UTC info in the SDF data
                startTime = gpstk::CivilTime(ye,mo,da,ho,mi,se,gpstk::TimeSystem::UTC);
            }
            catch (const std::exception& e)
            {
                ROS_FATAL("Could not  extract time from world file: %s",e.what());
            }
        }

        // Update the current time with an offset in fractional seconds. Must be very efficient
        // because  it is calle don every tick of the simulation physics engine
        static void Update(const double &offset)
        {
            try
            {
                currentTime = startTime.convertToCommonTime();
                currentTime.addSeconds(offset);
                currentTime.setTimeSystem(gpstk::TimeSystem::UTC);
            }
            catch (const std::exception& e)
            {
                ROS_FATAL("Could not update the clock: %s",e.what());
            }
        }

        // ALL CHILDREN MUST IMPLEMENT THE FOLLOWING METHODS ////////////////////////////

        // All sensors must be configured using the current model information and the SDF
        virtual void Configure(sdf::ElementPtr root, gazebo::physics::WorldPtr world) = 0;

        // All sensors must be resettable
        virtual void Reset() = 0;

    };
}

#endif