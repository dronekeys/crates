// Standard libraries
#include <hal/model/Quadrotor.h>

using namespace hal::model;

// Rate at which control is updated internally
#define DEFAULT_UPDATE_RATE  50.0
#define DEFAULT_CONTROL_RATE  1.0
#define DEFAULT_STATE_RATE    1.0

// This is called on every platform-level controller clock tick
void Quadrotor::Update(const ros::TimerEvent& event)
{
    // Determine the required control to satisfy action, given current state
    /*
    control = hal::controller::ControllerBase<hal_model_quadrotor::State,hal_model_quadrotor::Control>::GetControl(
        state, event.current_real.toSec() - tick
    );
    */
    
    // Update the flight control system
    Control(control);

    // Save the current time tick
    tick = event.current_real.toSec();
}

void Quadrotor::BroadcastControl(const ros::TimerEvent& event)
{                  
    pubControl.publish(control);
}

void Quadrotor::BroadcastState(const ros::TimerEvent& event)
{                  
    pubState.publish(state);
}

Quadrotor::Quadrotor() : hal::HAL()
{
    // Do nothing
}

bool Quadrotor::RcvAnglesHeight(
    hal_model_quadrotor::AnglesHeight::Request  &req, 
    hal_model_quadrotor::AnglesHeight::Response &res)
{
    return true;
}

bool Quadrotor::RcvEmergency(
    hal_model_quadrotor::Emergency::Request  &req, 
    hal_model_quadrotor::Emergency::Response &res)
{
    return true;
}

bool Quadrotor::RcvHover(
    hal_model_quadrotor::Hover::Request  &req, 
    hal_model_quadrotor::Hover::Response &res)
{
    return true;
}

bool Quadrotor::RcvIdle(
    hal_model_quadrotor::Idle::Request  &req, 
    hal_model_quadrotor::Idle::Response &res)
{
    return true;
}

bool Quadrotor::RcvLand(
    hal_model_quadrotor::Land::Request  &req, 
    hal_model_quadrotor::Land::Response &res)
{
    return true;
}

bool Quadrotor::RcvTakeoff(
    hal_model_quadrotor::Takeoff::Request  &req, 
    hal_model_quadrotor::Takeoff::Response &res)
{
    return true;
}

bool Quadrotor::RcvVelocity(
    hal_model_quadrotor::Velocity::Request  &req, 
    hal_model_quadrotor::Velocity::Response &res)
{
    return true;
}

bool Quadrotor::RcvVelocityHeight(
    hal_model_quadrotor::VelocityHeight::Request  &req, 
    hal_model_quadrotor::VelocityHeight::Response &res)
{
    return true;
}

bool Quadrotor::RcvWaypoint(
    hal_model_quadrotor::Waypoint::Request  &req, 
    hal_model_quadrotor::Waypoint::Response &res)
{
    return true;
}

void Quadrotor::OnInit()
{
    // Advertise the various controllers on the ROS backbone
    srvAnglesHeight     = GetRosNodePtr()->advertiseService("controller/AnglesHeight", &Quadrotor::RcvAnglesHeight, this);
    srvEmergency        = GetRosNodePtr()->advertiseService("controller/Emergency", &Quadrotor::RcvEmergency, this);
    srvHover            = GetRosNodePtr()->advertiseService("controller/Hover", &Quadrotor::RcvHover, this);
    srvIdle             = GetRosNodePtr()->advertiseService("controller/Idle", &Quadrotor::RcvIdle, this);
    srvLand             = GetRosNodePtr()->advertiseService("controller/Land", &Quadrotor::RcvLand, this);
    srvTakeoff          = GetRosNodePtr()->advertiseService("controller/Takeoff", &Quadrotor::RcvTakeoff, this);
    srvVelocity         = GetRosNodePtr()->advertiseService("controller/Velocity", &Quadrotor::RcvVelocity, this);
    srvVelocityHeight   = GetRosNodePtr()->advertiseService("controller/VelocityHeight", &Quadrotor::RcvVelocityHeight, this);
    srvWaypoint         = GetRosNodePtr()->advertiseService("controller/Waypoint", &Quadrotor::RcvWaypoint, this);

/*
    // 1 argument: any node can transition to these controllers immediately
    hal::controller::ControllerBase<hal_model_quadrotor::State,hal_model_quadrotor::Control>::PermitInstant("Emergency");
    hal::controller::ControllerBase<hal_model_quadrotor::State,hal_model_quadrotor::Control>::PermitInstant("Land");
    hal::controller::ControllerBase<hal_model_quadrotor::State,hal_model_quadrotor::Control>::PermitInstant("Idle","Takeoff");
    hal::controller::ControllerBase<hal_model_quadrotor::State,hal_model_quadrotor::Control>::PermitQueued("Land","Takeoff");
    hal::controller::ControllerBase<hal_model_quadrotor::State,hal_model_quadrotor::Control>::PermitQueued("Land","Idle");

    // All non-critical actions can be switched between automatically
    std::vector<const char*> actions;
    actions.push_back("Hover");
    actions.push_back("AnglesHeight");
    actions.push_back("VelocityHeight");
    actions.push_back("Velocity");
    actions.push_back("Waypoint");
    for (std::vector<const char*>::iterator i = actions.begin(); i != actions.end(); ++i)
    {
        hal::controller::ControllerBase<hal_model_quadrotor::State,hal_model_quadrotor::Control>::PermitQueued("Takeoff",*i);
        for (std::vector<const char*>::iterator j = actions.begin(); j != actions.end(); ++j)
            hal::controller::ControllerBase<hal_model_quadrotor::State,hal_model_quadrotor::Control>::PermitInstant(*i,*j);
    }

    // The UAV is always started on the ground
    hal::controller::ControllerBase<hal_model_quadrotor::State,hal_model_quadrotor::Control>::SetController("Idle");
*/
    // CREATE THE DATA BROADCAST TIMERS ///////////////////////////////////////////////////////////////

    // Advertise this message on the ROS backbone
    pubState   = hal::model::Quadrotor::GetRosNodePtr()->template advertise<hal_model_quadrotor::State>("State", DEFAULT_QUEUE_LENGTH);
    pubControl = hal::model::Quadrotor::GetRosNodePtr()->template advertise<hal_model_quadrotor::Control>("Control", DEFAULT_QUEUE_LENGTH);

    // Immediately start control loop
    timerUpdate = GetRosNodePtr()->createTimer(
        ros::Duration(1.0/DEFAULT_UPDATE_RATE), 
        &Quadrotor::Update, 
        this
    );

    // Immediately start control loop
    timerState = GetRosNodePtr()->createTimer(
        ros::Duration(1.0/DEFAULT_STATE_RATE), 
        &Quadrotor::BroadcastState, 
        this
    );

    // Immediately start control loop
    timerControl = GetRosNodePtr()->createTimer(
        ros::Duration(1.0/DEFAULT_CONTROL_RATE), 
        &Quadrotor::BroadcastControl, 
        this
    );
    
}

bool Quadrotor::SetState(const hal_model_quadrotor::State &sta)
{
    state = sta;
}