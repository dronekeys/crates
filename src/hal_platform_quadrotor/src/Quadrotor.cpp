// Standard libraries
#include <hal_platform_quadrotor/Quadrotor.h>

using namespace hal::platform;

// Rate at which control is updated internally
#define DEFAULT_UPDATE_RATE  50.0
#define DEFAULT_CONTROL_RATE  1.0
#define DEFAULT_STATE_RATE    1.0

// This is called on every platform-level controller clock tick
void Quadrotor::Update(const ros::TimerEvent& event)
{
    // Determine the required control to satisfy action, given current state
    control = controllers[current]->Update(state, event.current_real.toSec() - tick);
    
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

bool Quadrotor::SwitchController(const std::string& request)
{
    // Only accept valid controllers
    if (controllers.find(request) == controllers.end())
        return false;

    // Determine the controller type
    FlightLogicType next = controllers[request]->GetType();
    FlightLogicType curr = controllers[current]->GetType();

    // Permissable
    bool allowed = false;

    // Always permit a transition to emergency
    if (next == EmergencyType)
        allowed = true;
    else
    {
        // Flight logic
        switch (curr)
        {
        case EmergencyType: allowed = false;
            break;
        case IdleType:      allowed = (next == TakeoffType);
            break;
        case TakeoffType:   allowed = false;   
            break;
        case HoverType:     allowed = (next == LandType || next == ActionType);
            break;
        case LandType:      allowed = false;
            break;
        case ActionType:    allowed = (next == LandType || next == HoverType || next == ActionType);
            break;
        }
    }

    // If this transition is allowed, and the lookup key exists
    if (allowed)
        current = request;
    
    // Default not permit
    return allowed;
} 

Quadrotor::Quadrotor(const char *name) : hal::platform::Platform(name),
    hal::controller::Emergency("Emergency"),
    hal::controller::Hover("Hover"),
    hal::controller::Land("Land"),
    hal::controller::Takeoff("Takeoff"),
    hal::controller::Velocity("Velocity"),
    hal::controller::AnglesHeight("AnglesHeight"),
    hal::controller::VelocityHeight("VelocityHeight"),
    hal::controller::Waypoint("Waypoint")
{
    // DEFINE A VALID SET OF TRANSITIONS /////////////////////////////////////////////////////////////

    // 1 argument: any node can transition to these controllers immediately
    hal::controller::Controller::AddImmediate("Emergency");
    hal::controller::Controller::AddImmediate("Land");
    
    // 2 arguments: directional transition
    hal::controller::Controller::AddWait("Land", "Takeoff");
    hal::controller::Controller::AddWait("Takeoff", "Hover");

    // 2+ arguments: fully connected transitions
    hal::controller::Controller::AddImmediate("Hover","AnglesHeight","VelocityHeight","Velocity","Waypoint");

    // Assume the UAV is started on the ground
    hal::controller::Controller::SetStartController("Idle");

    // CREATE THE DATA BROADCAST TIMERS ///////////////////////////////////////////////////////////////

    // Advertise this message on the ROS backbone
    pubState   = rosNode.advertise<hal_platform_quadrotor::State>("State", DEFAULT_QUEUE_LENGTH);
    pubControl = rosNode.advertise<hal_platform_quadrotor::Control>("Control", DEFAULT_QUEUE_LENGTH);

    // Immediately start control loop
    timerUpdate = rosNode.createTimer(
        ros::Duration(1.0/DEFAULT_UPDATE_RATE), 
        &Quadrotor::Update, 
        this
    );

    // Immediately start control loop
    timerState = rosNode.createTimer(
        ros::Duration(1.0/DEFAULT_STATE_RATE), 
        &Quadrotor::BroadcastState, 
        this
    );

    // Immediately start control loop
    timerControl = rosNode.createTimer(
        ros::Duration(1.0/DEFAULT_CONTROL_RATE), 
        &Quadrotor::BroadcastControl, 
        this
    );
}

bool Quadrotor::SetState(const hal_platform_quadrotor::State &sta)
{
    state = sta;
}