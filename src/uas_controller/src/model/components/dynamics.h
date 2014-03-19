#ifndef UAS_CONTROLLER_DYNAMICS_H
#define UAS_CONTROLLER_DYNAMICS_H

// Required for the maths functions
#include <gazebo/gazebo.hh>

namespace uas_controller
{
    class Dynamics : public Component
    {

    private:
    
	    // Constant model parameters
	    double _LOW_THROTT, _MAX_ANGVEL, _G,  _MASS_KG;         // Platform config
	    double _pq0, _pq1, _pq2, _r0, _r1;                      // Rotational params
	    double _Cth0, _Cth1, _Cth2, _Cvb0, _Cvb1, _tau0, _tau1; // Thrust params
	    double _kuv, _kw;                                       // Drag parameters	    

    public:

    	// Default constructor
    	Dynamics();

    };
}

#endif