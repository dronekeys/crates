#include <hal/model/controller/VelocityHeight.h>

// Controller constants
#define _U          0       
#define _V          1
#define _YAW        2
#define _HEIGHT     3
#define _Kvp        0.25     /* xy velocity proportional constant          */
#define _Kvi        0.003    /* xy velocity integrative constant           */
#define _Kvd        0.05     /* xy velocity derivative constant            */
#define _Kiz        0.0008   /* altitude integrative constant              */
#define _Kpz        0.03     /* altitude proportional constant             */
#define _Kdz        0.04     /* altitude derivative constant               */
#define _th_hover   0.59     /* throttle hover offset                      */
#define _maxtilt    0.34     /* max pitch/roll angle                       */
#define _Kya        6.0      /* yaw proportional constant                  */
#define _maxyawrate 4.4      /* max allowed yaw rate                       */
#define _maxv       3.0      /* max allowed xy velocity                    */

using namespace hal::model;

bool VelocityHeight::SetGoal(
    hal_model_quadrotor::VelocityHeight::Request  &req, 
    hal_model_quadrotor::VelocityHeight::Response &res
) {

    // Rest the controller
    Reset();

    // Horizontal velocity 
    sp[_U]      = req.u;
    sp[_V]      = req.v;
    sp[_YAW]    = req.yaw;
    sp[_HEIGHT] = req.z;

    // Try and switch control
    return true;
}

VelocityHeight::VelocityHeight() : Controller()
{
    Reset();
}

bool VelocityHeight::Update(const hal_model_quadrotor::State &state, 
    double dt, hal_model_quadrotor::Control &control)
{
    /******************************************************************
    %  Computes the quadrotor control signals given the current state,
    %  desired altitude, heading and velocity in global frame (ENU coords)
    %
    %  The desidred 2D velocity is enforced by a P controller that controls
    %  the pitch and roll angles of the platform as necessary. Altitude 
    %  and heading are controlled independently. Limits are in place to 
    %  not reach dangerous velocities. 
    %
    ******************************************************************/

    // Body-frame velocity (v) and b < n frame rotation (r)
    double vt[3], r[3];

    ///////////////// ROTATE VELOCITY INTO B-FRAME /////////////////////

    // Get the Euler orientation
    r[0] = state.roll;
    r[1] = state.pitch;
    r[2] = state.yaw;

    // Make a copy of the n-frame velocity
    vt[0] = sp[_U];
    vt[1] = sp[_V];
    vt[2] = 0.0;

    // Get the n-frame (x,y) velocity in the b-frame
    n2b(r,vt);

    // Used for calculations below
    double e, de;

    //////////////////////// PID CONTROLLER FOR PITCH ///////////////////////////

    // b-frame X controller is a full PID
    e =  (limit(vt[0],-_maxv,_maxv) - state.u);  
    de = (first ? 0 : (e - ep[_U]) / dt);
    ei[_U] += e; 
    ep[_U]  = e;

    double desP = limit(_Kvp * e + _Kvi * ei[_U] + _Kvd * de,-_maxtilt,_maxtilt);

    ///////////////////////// PID CONTROLLER FOR ROLL ///////////////////////////

    // b-frame Y controller is a full PID
    e = -(limit(vt[1],-_maxv,_maxv) - state.v);     
    de = (first ? 0 : (e - ep[_V]) / dt);
    ei[_V] += e; 
    ep[_V]  = e;

    double desR = limit(_Kvp * e + _Kvi * ei[_V] + _Kvd * de,-_maxtilt,_maxtilt);

    ////////////////////////// PID THROTTLE CONTROLLER //////////////////////////

    // Get the (P)roportional component
    double ez_ = sp[_HEIGHT] - state.z;

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
    
    //////////////////// P CONTROLLER FOR YAW //////////////////////

    double desY = limit(_Kya * (sp[_YAW] - state.yaw), -_maxyawrate, _maxyawrate);

    //////////////////////// CONTROL PACKAGING /////////////////////////
    
    // This is no longer the first iteration
    first = false;

    // This will be returned
    control.roll     = desR;
    control.pitch    = desP;
    control.yaw      = desY;
    control.throttle = th;
    return true;
}

// Goal reach implementations
bool VelocityHeight::HasGoalBeenReached()
{
    return reach;
}

// Reset the current state
void VelocityHeight::Reset()
{
    // Horizontal velocity 
    sp[_U]      = 0.0;
    sp[_V]      = 0.0;
    sp[_YAW]    = 0.0;
    sp[_HEIGHT] = 0.0;

    // PID carries
    for (int i = 0; i < 2; i++)
    {
        ei[i] = 0.0;
        ep[i] = 0.0;
    }
    ez = 0.0;
    iz = 0.0;

    // Reset
    first = true;
    reach = false;
}