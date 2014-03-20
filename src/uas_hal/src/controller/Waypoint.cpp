// Local library incldues
#include <uas_hal/controller/Waypoint.h>

// Convenience declarations
#define _X      0
#define _Y      1
#define _Z      2
#define _YAW    3

using namespace uas_hal;

// Default construtor resets the controller
Waypoint::Waypoint() 
{
    Reset();
}

// Set the goal of the controller
void Waypoint::SetGoal(double x, double y, double z, double yaw)
{
    // Rest the controller
    Reset();

    // Set the new goal
    sp[_X]   = x;
    sp[_Y]   = y;
    sp[_Z]   = z;
    sp[_YAW] = yaw;
}

// Pass the state and discrete time step and receive a control back
void Waypoint::Update(State *state, double dt, Control *ctl)
{
    /******************************************************************
    %  Computes the quadtotor control signals given the current state 
    %  and a desired waypoint. The desired attitude is enforced by a
    %  P controller that tries to achieve a linear velocity proportional
    %  to the  distance from the target. Limits are in place to not 
    %  reach dangerous velocities.
    %
    ******************************************************************/
    
    // Obtain a b-frame (u,v) velocities
    double d = sqrt((sp[_X]-state->x)*(sp[_X]-state->x)
                   +(sp[_Y]-state->y)*(sp[_Y]-state->y));
    double a = atan2(sp[_Y]-state->y,sp[_X]-state->x) - state->yaw;
    double bx = d * cos(a);
    double by = d * sin(a); 
    
    ////////////////////// P PITCH CONTROLLER ////////////////////////

    double desu = limit(_Kxy*bx,-_maxv,_maxv);
    
    double desP = limit(_Kv*(-(desu - state->u)),-_maxtilt,_maxtilt);
    
    ////////////////////// P ROLL CONTROLLER ////////////////////////

    double desv = limit(_Kxy*by,-_maxv,_maxv);
    
    double desR = limit(_Kv*(desv - state->v),-_maxtilt,_maxtilt);
    
    //////////////////////// P YAW CONTROLLER ////////////////////////

    double desY = limit(_Kya * (sp[_YAW] - state->yaw), -_maxyawrate, _maxyawrate);
    
    /////////////////// PID THROTTLE CONTROLLER //////////////////////

    // Get the (P)roportional component
    double ez_ = -(sp[_Z] - state->z);

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

    // This will be returned
    ctl->pitch    = desP;
    ctl->roll     = desR;
    ctl->yaw      = desY;
    ctl->throttle = th;

    // This is no longer the first iteration
    first = false;
}

// Reset the controller
void Waypoint::Reset()
{

    // Set the new goal
    sp[_X]   = 0.0;
    sp[_Y]   = 0.0;
    sp[_Z]   = 0.0;
    sp[_YAW] = 0.0;

    // Carries for altitude PID
    iz = 0.0;
    ez = 0.0;

    // Reset
    first = true;
}
