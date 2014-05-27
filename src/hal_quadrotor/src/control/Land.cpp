#include <hal_quadrotor/control/Land.h>

// COntroller constants
#define _X          0
#define _Y          1
#define _Z          2
#define _YAW        3
#define _Kxy        0.9         /* position proportional constant */
#define _Kv         0.09        /* velocity proportional constant */
#define _Kiz        0.0008      /* altitude integrative constant  */
#define _Kpz        0.03        /* altitude proportional constant */ 
#define _Kdz        0.04        /* altitude derivative constant   */      
#define _th_hover   0.59        /* throttle hover offset          */
#define _maxtilt    0.34        /* max pitch/roll angle           */
#define _Kya        6.0         /* yaw proportional constant      */
#define _maxyawrate 4.4         /* max allowed yaw rate           */
#define _maxv       5.0         /* max allowed xy velocity        */

using namespace hal::quadrotor;

bool Land::SetGoal(
    hal_quadrotor::Land::Request  &req, 
    hal_quadrotor::Land::Response &res
) {
    // Set the new goal
    sp[_Z] = 0.0;

    // Carries for altitude PID
    iz = 0.0;
    ez = 0.0;

    // Mark as first iteration
    first = true;

    // Have we reached our desination
    reach = false;

    // Try and switch control
    res.success = true;
    res.status  = "Successfully switched to Land controller";
    return true;
}

bool Land::Update(const hal_quadrotor::State &state, 
    double dt, hal_quadrotor::Control &control)
{
    /******************************************************************
    %  Computes the quadtotor control signals given the current state 
    %  and a desired Land. The desired attitude is enforced by a
    %  P controller that tries to achieve a linear velocity proportional
    %  to the  distance from the target. Limits are in place to not 
    %  reach dangerous velocities.
    %
    ******************************************************************/
    
    ////////////////////// STICK TO THE POSITION /////////////////// 

	if (first)
	{
	    sp[_X]   = state.x;
	    sp[_Y]   = state.y;
	    sp[_YAW] = state.yaw;
	}
    
    ////////////////////// CALCULATE VELOCITIES ////////////////////

    double d = sqrt((sp[_X]-state.x)*(sp[_X]-state.x)
                   +(sp[_Y]-state.y)*(sp[_Y]-state.y));
    double a = atan2(sp[_Y]-state.y , sp[_X]-state.x) - state.yaw;
    double bx = d * cos(a);
    double by = d * sin(a); 
    
    ////////////////////// P ROLL CONTROLLER ////////////////////////

    double desu = limit(_Kxy*bx,-_maxv,_maxv);
    double desP = limit( _Kv*(desu - state.u), -_maxtilt, _maxtilt);
    
    ////////////////////// P PITCH CONTROLLER ////////////////////////

    double desv = limit(_Kxy*by,-_maxv,_maxv);
    double desR = limit(-_Kv*(desv - state.v), -_maxtilt, _maxtilt);
    
    //////////////////////// P YAW CONTROLLER ////////////////////////

    double desY = limit(_Kya * (sp[_YAW] - state.yaw), -_maxyawrate, _maxyawrate);
    
    /////////////////// PID THROTTLE CONTROLLER //////////////////////

    // Get the (P)roportional component (Changed by Andrew)
    double ez_ = sp[_Z] - state.z;

    // Get the (I)ntegral component
    iz = iz + ez_ * dt;
    
    // Get the (D)erivative component
    double de_ = (first ? 0 : (ez_ - ez) / dt);
    double desth = _th_hover + _Kpz * ez_ + _Kiz * iz + de_ * _Kdz;
    double th = limit(desth,0.0,1.0);
    
    // Save (P) contribution for next derivative calculation
    ez = ez_;

    // Save (I) contribution for next derivative calculation
    iz = iz - (desth - th) * 2.0;

    //////////////////////// CONTROL PACKAGING /////////////////////////

    // This is no longer the first iteration
    first = false;

    // This will be returned
    control.roll     = desR;
    control.pitch    = desP;
    control.yaw      = desY;
    control.throttle = th;

    //////////////////////// CHECK IF GOAL REACHED //////////////////////

    double dist = sqrt(
    		(sp[_X]-state.x)*(sp[_X]-state.x) 
    	+	(sp[_Y]-state.y)*(sp[_Y]-state.y)
    	+	(sp[_Z]-state.z)*(sp[_Z]-state.z)
    );
    if (!reach && dist < 0.1)
    	reach = true;

    /////////////////////////////////////////////////////////////////////

    // Return success
    return true;
}

// Goal reach implementations
bool Land::HasGoalBeenReached()
{
    return reach;
}