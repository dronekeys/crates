#ifndef CONTROLLER_MODEL_H
#define CONTROLLER_MODEL_H

// System includes
#include <string>
#include <vector>

// Required for the maths functions
#include <gazebo/gazebo.hh>

// Mainly for debugging
#include <ros/ros.h>

// HAL includes
#include "environment.pb.h"
#include "meteorological.pb.h"
#include "satellites.pb.h"

// Used in a couple of places
#define MATH_PI 3.14159265359

namespace controller
{
    // Save some pain
    typedef const boost::shared_ptr<const msgs::Environment> EnvironmentPtr;

    class Model
    {
    protected:

        // Clamp a value to exist between two intervals
        static double limit(double val, double minval, double maxval)
        {
            if (val < minval) return minval;
            if (val > maxval) return maxval;
            return val;
        }
        
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

    public:

        // All sensors must be configured using the current model information and the SDF
        virtual void Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr model) = 0;

        // All sensors must be resettable
        virtual void Reset() = 0;

    };
}

#endif