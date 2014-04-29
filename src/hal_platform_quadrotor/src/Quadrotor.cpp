// Standard libraries
#include <hal/platform/quadrotor/Quadrotor.h>

using namespace hal::platform;

// Rate at which control is updated internally
#define DEFAULT_UPDATE_RATE  50.0
#define DEFAULT_CONTROL_RATE  1.0
#define DEFAULT_STATE_RATE    1.0

// This is called on every platform-level controller clock tick
void Quadrotor::Update(const ros::TimerEvent& event)
{
    // Determine the required control to satisfy action, given current state
    control = hal::controller::ControllerBase<hal_platform_quadrotor::State,hal_platform_quadrotor::Control>::GetControl(
        state, event.current_real.toSec() - tick
    );
    
    // Update the flight control system
    ReceiveControl(control);

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

Quadrotor::Quadrotor(ros::NodeHandle& node) : 
    hal::platform::Platform(node, "quadrotor"),                  /* Platform basics      */
    hal::controller::Emergency(node,"Emergency"),                /* Motors off           */
    hal::controller::Hover(node,"Hover"),                        /* Hold position        */
    hal::controller::Land(node,"Land"),                          /* Land in place        */
    hal::controller::Idle(node,"Idle"),                          /* Ground, motors off   */
    hal::controller::Takeoff(node,"Takeoff"),                    /* Takeoff to altitude  */
    hal::controller::Velocity(node,"Velocity"),
    hal::controller::AnglesHeight(node,"AnglesHeight"),
    hal::controller::VelocityHeight(node,"VelocityHeight"),
    hal::controller::Waypoint(node,"Waypoint")
{
    // DEFINE A VALID SET OF TRANSITIONS /////////////////////////////////////////////////////////////

    // 1 argument: any node can transition to these controllers immediately
    hal::controller::ControllerBase<hal_platform_quadrotor::State,hal_platform_quadrotor::Control>::PermitInstant("Emergency");
    hal::controller::ControllerBase<hal_platform_quadrotor::State,hal_platform_quadrotor::Control>::PermitInstant("Land");
    hal::controller::ControllerBase<hal_platform_quadrotor::State,hal_platform_quadrotor::Control>::PermitInstant("Idle","Takeoff");
    hal::controller::ControllerBase<hal_platform_quadrotor::State,hal_platform_quadrotor::Control>::PermitQueued("Land","Takeoff");
    hal::controller::ControllerBase<hal_platform_quadrotor::State,hal_platform_quadrotor::Control>::PermitQueued("Land","Idle");

    // All non-critical actions can be switched between automatically
    std::vector<const char*> actions;
    actions.push_back("Hover");
    actions.push_back("AnglesHeight");
    actions.push_back("VelocityHeight");
    actions.push_back("Velocity");
    actions.push_back("Waypoint");
    for (std::vector<const char*>::iterator i = actions.begin(); i != actions.end(); ++i)
    {
        hal::controller::ControllerBase<hal_platform_quadrotor::State,hal_platform_quadrotor::Control>::PermitQueued("Takeoff",*i);
        for (std::vector<const char*>::iterator j = actions.begin(); j != actions.end(); ++j)
            hal::controller::ControllerBase<hal_platform_quadrotor::State,hal_platform_quadrotor::Control>::PermitInstant(*i,*j);
    }

    // The UAV is always started on the ground
    hal::controller::ControllerBase<hal_platform_quadrotor::State,hal_platform_quadrotor::Control>::SetController("Idle");

    // CREATE THE DATA BROADCAST TIMERS ///////////////////////////////////////////////////////////////

    // Advertise this message on the ROS backbone
    pubState   = hal::platform::Platform::rosNode.advertise<hal_platform_quadrotor::State>("State", DEFAULT_QUEUE_LENGTH);
    pubControl = hal::platform::Platform::rosNode.advertise<hal_platform_quadrotor::Control>("Control", DEFAULT_QUEUE_LENGTH);

    // Immediately start control loop
    timerUpdate = hal::platform::Platform::rosNode.createTimer(
        ros::Duration(1.0/DEFAULT_UPDATE_RATE), 
        &Quadrotor::Update, 
        this
    );

    // Immediately start control loop
    timerState = hal::platform::Platform::rosNode.createTimer(
        ros::Duration(1.0/DEFAULT_STATE_RATE), 
        &Quadrotor::BroadcastState, 
        this
    );

    // Immediately start control loop
    timerControl = hal::platform::Platform::rosNode.createTimer(
        ros::Duration(1.0/DEFAULT_CONTROL_RATE), 
        &Quadrotor::BroadcastControl, 
        this
    );
}

bool Quadrotor::SetState(const hal_platform_quadrotor::State &sta)
{
    state = sta;
}