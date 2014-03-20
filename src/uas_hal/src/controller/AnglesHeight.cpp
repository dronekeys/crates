// Local library incldues
#include <uas_hal/controller/AnglesHeight.h>

// Convenience declarations
#define _PITCH  0
#define _ROLL   1 
#define _YAW    2
#define _HEIGHT 3

using namespace uas_hal;

// Default construtor sets PID goal
AnglesHeight::AnglesHeight()
{
    Reset();
}

// Constructor ta
void AnglesHeight::SetGoal(double pitch, double roll, double yaw, double height)
{
    // Rest the controller
    Reset();

    // Set the new goal
    sp[_PITCH]  = pitch;
    sp[_ROLL]   = roll;
    sp[_YAW]    = yaw;
    sp[_HEIGHT] = height;
}

// Pass the state and discrete time step and receive a control back
void AnglesHeight::Update(State *state, double dt, Control *ctl)
{
    /******************************************************************
    %  Computes the quadrotor control signals given the current state, 
    %  desired angles, heading and altitude
    %
    %  The desidred pitch and roll angles are directly passed as input
    %  to the platform, altitude and heading are controlled using 2 
    %  independent PIDs.  Limits are in place to not reach dangerous 
    %
    ******************************************************************/
    // _PITCH _ROLL : direct control
    // _YAW         : uses a P-controller
    // _HEIGHT      : uses a PID-controller

    ////////////////////////// YAW CONTROLLER /////////////////////////

    double ya = limit(_Kya*(sp[_YAW] - state->yaw),-_maxyawrate,_maxyawrate);

    //////////////////////// THROTTLE CONTROLLER ////////////////////////

    // Get the (P)roportional component
    double ez_ = -(sp[_HEIGHT] - state->z);

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
    ctl->pitch    = sp[_PITCH];
    ctl->roll     = sp[_ROLL];
    ctl->yaw      = ya;
    ctl->throttle = th;

    // This is no longer the first iteration
    first = false;
}

// Reset the controller
void AnglesHeight::Reset()
{
    // Reset goal
    sp[_PITCH]  = 0.0;
    sp[_ROLL]   = 0.0;
    sp[_YAW]    = 0.0;
    sp[_HEIGHT] = 0.0;

    // Carry for altitude PID
    iz = 0.0;
    ez = 0.0;

    // Reset
    first = true;
}
