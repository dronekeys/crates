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
    //control = Controller::GetControl(state, event.current_real.toSec() - tick);
    
    // Update the flight control system
    //OnControl(control);

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

Quadrotor::Quadrotor(const char *name) : 
    hal::platform::Platform(name),
    hal::controller::Emergency(boost::)
{

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