#include <hal/model/controller/Velocity.h>

// Controller constants
#define _X          0
#define _Y          1
#define _Z          2
#define _YAW        3
#define _Kvp        0.25     /* xy velocity proportional constant */
#define _Kvi        0.003    /* xy velocity integrative constant  */
#define _Kvd        0.05     /* xy velocity derivative constant   */
#define _Kwp       -0.2      /* z velocity proportional constant  */
#define _Kwi       -0.002    /* z velocity integrative constant   */
#define _Kwd       -0.0      /* z velocity derivative constant    */
#define _th_hover   0.59     /* throttle hover offset             */
#define _maxtilt    0.34     /* max pitch/roll angle              */
#define _Kya        6.0      /* yaw proportional constant         */
#define _maxyawrate 4.4      /* max allowed yaw rate              */
#define _maxv       3.0      /* max allowed xy velocity           */

using namespace hal::model;

bool Velocity::SetGoal(
    hal_model_quadrotor::Velocity::Request  &req, 
    hal_model_quadrotor::Velocity::Response &res
) {

    // Rest the controller
    Reset();

    // Set the velocity
    sp[_X]   = req.u;
    sp[_Y]   = req.v;
    sp[_Z]   = req.w;
    sp[_YAW] = req.yaw;

    // Try and switch control
    return true;
}

Velocity::Velocity() : Controller()
{
    Reset();
}

bool Velocity::Update(const hal_model_quadrotor::State &state, 
    double dt, hal_model_quadrotor::Control &control)
{
    /******************************************************************
    %  Computes the quadtotor control signals given the current state 
    %  and a target navigation frame velocity and heading
    %
    %  The desired attitude is enforced by a P controller that tries to 
    %  achieve the required linear velocity. Limits are in place to not 
    %  reach dangerous velocities. 
    %
    ******************************************************************/

    double vt[3], r[3], e, de;

    ///////////////// ROTATE VELOCITY INTO B-FRAME /////////////////////

    // Get the Euler orientation
    r[_X] = state.roll;
    r[_Y] = state.pitch;
    r[_Z] = state.yaw;;

    // Make a copy of the n-frame velocity
    for (int i =0; i < 3; i++)
        vt[i] = sp[i];

    // Get the b-frame velocity
    n2b(r,vt);

    //////////////////// PID CONTROLLER FOR ROLL //////////////////////

    // b-frame X
    e = -(limit(vt[_X],-_maxv,_maxv) - state.u);  
    de = (first ? 0 : (e - ep[_X]) / dt);
    ei[_X] += e; 
    ep[_X]  = e;

    double desP = limit(_Kvp * e + _Kvi * ei[_X] + _Kvd * de, -_maxtilt, _maxtilt);

    //////////////////// PID CONTROLLER FOR PITCH //////////////////////

    // b-frame Y
    e = -(limit(vt[_Y],-_maxv,_maxv) - state.v);     
    de = (first ? 0 : (e - ep[_Y]) / dt);
    ei[_Y] += e; 
    ep[_Y]  = e;

    double desR = limit(_Kvp * e + _Kvi * ei[_Y] + _Kvd * de, -_maxtilt, _maxtilt);

    //////////////////// PID CONTROLLER FOR THRUST //////////////////////

    // b-frame w
    e = (limit(vt[_Z],-_maxv,_maxv) - state.w);
    de = (first ? 0 : (e - ep[_Z]) / dt);
    ei[_Z] += e;
    ep[_Z] =  e;

    double desT = limit(_Kwp * e + _Kwi * ei[_Z] + _Kwd * de + _th_hover, 0.0, 1.0);

    //////////////////// P CONTROLLER FOR YAW //////////////////////

    double desY = limit(_Kya * (sp[_YAW] - state.yaw), -_maxyawrate, _maxyawrate);

    //////////////////////// CONTROL PACKAGING /////////////////////////

    // This is no longer the first iteration
    first = false;

    // This will be returned
    control.roll     = desR;
    control.pitch    = desP;
    control.yaw      = desY;
    control.throttle = desT;
    return true;
}

// Goal reach implementations
bool Velocity::HasGoalBeenReached()
{
    return reach;
}

// Reset the current state
void Velocity::Reset()
{
    // Set the velocity
    sp[_X]   = 0.0;
    sp[_Y]   = 0.0;
    sp[_Z]   = 0.0;
    sp[_YAW] = 0.0;

    // For 
    for (int i = 0; i < 3; i++)
    {
        ei[i] = 0.0;
        ep[i] = 0.0;
    }

    // Reset
    first = true;
    reach = false;
}