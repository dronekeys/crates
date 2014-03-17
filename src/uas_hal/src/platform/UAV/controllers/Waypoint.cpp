// Local library incldues
#include "Waypoint.h"

using namespace uas_hal::controller;

// Constructor
Waypoint::Waypoint(double dt) : Controller(dt) {}

// Obtain the control
uas_hal::Control Waypoint::control(
	uas_hal::State& state, 					// The current platform state
	uas_hal::WaypointActionGoal& goal)		// The goal to reach
{
	// Check to see if the waypoint has changed (note that this ignore heading)
	bool wpChange = !((goal.x == wp.x) && (goal.y == wp.y) && (goal.z == wp.z));
    if (wpChange)
    {
    	wp.x = goal.x;
    	wp.y = goal.y;
    	wp.z = goal.z;
    }

    // Get the distance and angle to the destination
    double d = sqrt(((wp.x-state.x)*(wp.x-state.x)+(wp.y-state.y)*(wp.y-state.y)));
    double a = atan2((wp.y-state.y),(wp.x-state.x)) - state.psi;
    
    // Convert from polar to cartesian coordinates
    double bx = d * cos(a);
    double by = d * sin(a);
    
    // X and Y are separate P controllers with limits
    double desTheta = limit(-_Kv*(limit(_Kxy*bx,-_maxvel,_maxvel) - state.u),-_maxtilt,_maxtilt);
    double desPhi   = limit(-_Kv*(limit(_Kxy*by,-_maxvel,_maxvel) - state.v),-_maxtilt,_maxtilt);
    double desPsi   = limit(_Kya*(goal.yaw - state.psi),-_maxyawrate,_maxyawrate);
    
	// Z is a full PID controller...

    // Get the current altitude error
    double ez_ = -(wp.z - state.z);
    
    // Preserve the integral
    iz = iz + ez_ * DT;

    // Derivative is only valid if goal has not changed
    double de_ = (wpChange ? 0 : (ez_ - ez) / DT);
    
    // Save the altitude error
    ez = ez_;
    
    // PID control
    double desThrust = _thoffset + _Kpz*ez_ + _Kiz*iz + _Kdz*de
    double th = limit(th,0.0,1.0);

 	// Avoid controller operating in a nonlinear region where increasing the control signal
 	// has no effect on the system output, a condition known as 'winding up'
    iz = iz - (desThrust - th) * 2;

	/*    
    uas_hal::Control ctl;
    ctl.theta 	= desTheta;
    ctl.phi 	= desPhi;
    ctl.thrust 	= desThrust;
    ctl.psi 	= desPsi;
    ctl.voltage	= 12.0;
    */
}

// Reset the goal
void Waypoint::reset()
{
    // Reset attitude
    iz 	 = 0;
    ez 	 = 0;

    // Set the destination to zero
	wp.x = 0;
	wp.y = 0;
	wp.z = 0;
}
