// Standard libraries
#include <hal_quadrotor/controller/VelocityHeight.h>

// Convenience declarations
#define _U      0
#define _V      1
#define _YAW    2
#define _HEIGHT 3

using namespace hal_quadrotor;

// Configure data broadcast at a given rate (<= 0.0 means disable)
bool VelocityHeight::Receive(ControlVelocityHeight::Request &req, ControlVelocityHeight::Response &res)
{
    // Rest the controller
    Reset();

    // Horizontal velocity 
    sp[_U]      = req.u;
    sp[_V]      = req.v;
    sp[_YAW]    = req.yaw;
    sp[_HEIGHT] = req.z;

    // Eveything OK
    return true;
}


// Constructor
VelocityHeight::VelocityHeight(ros::NodeHandle &node, std::string name) 
    : Controller(ActionType)
{
    service = node.advertiseService(name.c_str(), &VelocityHeight::Receive, this);
}

// Get new control from current state and time step
Control VelocityHeight::Update(const State &state, const double &dt)
{
    /******************************************************************
    %  Computes the quadrotor control signals given the current state,
    %  desired altitude, heading and velocity in global frame (NE  coords)
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
    //n2b(r,vt);

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
    
    // This will be returned
    control.roll     = desR;
    control.pitch    = desP;
    control.yaw      = desY;
    control.throttle = th;

    // This is no longer the first iteration
    first = false;

    // Return the control
    return control;
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
}