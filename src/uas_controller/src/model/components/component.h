#ifndef UAS_CONTROLLER_COMPONENT_H
#define UAS_CONTROLLER_COMPONENT_H

// System includes
#include <string>
#include <vector>

// Required for the maths functions
#include <gazebo/gazebo.hh>

// Core functionality
#include "component.h"

namespace uas_controller
{
    class Component
    {
    protected:

    	// Recurse down the tree  looking for elements
    	bool FindElement(sdf::ElementPtr &cur, std::string path)
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
    	double GetSDFDouble(sdf::ElementPtr cur, const char* path, double value)
    	{
			if (FindElement(cur, (std::string) path))
    			return cur->GetValue()->Get(value);
    		return value;
    	}

    	// Get a SDF double parameter
    	int GetSDFInteger(sdf::ElementPtr cur, const char* path, int value)
    	{
			if (FindElement(cur, (std::string) path))
    			return cur->GetValue()->Get(value);
    		return value;
    	}
    };
}

#endif