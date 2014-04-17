#include <hal_platform_quadrotor/controller/AnglesHeight.h>

using namespace hal::controller;

bool AnglesHeight::Receive(
    hal_platform_quadrotor::AnglesHeight::Request  &req, 
    hal_platform_quadrotor::AnglesHeight::Response &res
) {
    // Rest the controller
    Reset();

    // Set the new goal
    sp[_ROLL]   = req.roll;
    sp[_PITCH]  = req.pitch;
    sp[_YAW]    = req.yaw;
    sp[_HEIGHT] = req.z;

    // Eveything OK
    return true;
}

AnglesHeight::AnglesHeight() : Controller<hal_platform_quadrotor::State, hal_platform_quadrotor::Control,
    hal_platform_quadrotor::AnglesHeight::Request, hal_platform_quadrotor::AnglesHeight::Response>("AnglesHeight")
{}

hal_platform_quadrotor::Control AnglesHeight::Update(
    const hal_platform_quadrotor::State &state, 
    const double &dt
) {
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

    double ya = limit(_Kya*(sp[_YAW] - state.yaw),-_maxyawrate,_maxyawrate);

    //////////////////////// THROTTLE CONTROLLER ////////////////////////

    // Get the (P)roportional component (sign change by Andrew)
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
    
    //////////////////////// CONTROL PACKAGING /////////////////////////

    // This is no longer the first iteration
    first = false;
    
    // This will be returned
    hal_platform_quadrotor::Control control;
    control.roll     = sp[_ROLL];
    control.pitch    = sp[_PITCH];
    control.yaw      = ya;
    control.throttle = th;
    return control;
}

// Reset the current state
void AnglesHeight::Reset()
{
    // Reset goal
    sp[_ROLL]   = 0.0;
    sp[_PITCH]  = 0.0;
    sp[_YAW]    = 0.0;
    sp[_HEIGHT] = 0.0;

    // Carry for altitude PID
    iz = 0.0;
    ez = 0.0;

    // Reset
    first = true;
}