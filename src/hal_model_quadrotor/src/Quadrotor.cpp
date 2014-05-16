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
    // Update the state
    GetState(state);

    // Update control, given a state and discrete time step
    flightLogic.Update(state, event.current_real.toSec() - tick, control);

    // Update the flight control system
    SetControl(control);

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

bool Quadrotor::RcvState(
    hal_model_quadrotor::SetState::Request  &req, 
    hal_model_quadrotor::SetState::Response &res)
{
    // Pass the control down to the HAL
    SetState(req.state);

    // Notify the user
    res.success = true;
    res.status  = "State received. Updating simulated UAV.";
    return true;
}

bool Quadrotor::RcvControl(
    hal_model_quadrotor::SetControl::Request  &req, 
    hal_model_quadrotor::SetControl::Response &res)
{
    // Save the control
    control = req.control;

    // Disable the active controller
    flightLogic.Switch(DISABLED);

    // Pass the control down to the HAL
    SetControl(control);  

    // Notify the user
    res.success = true;
    res.status  = "Control received. Disabled current position controller.";
    return true;
}

void Quadrotor::OnInit()
{
    // If we are using simulation time, then we are in simulation mode
    bool isSimulated = false;
    if (GetRosNodePtr()->getParam("/use_sim_time",isSimulated) && isSimulated)
    {
        // Additional ROS services to manually set the control and state
        srvSetState   = GetRosNodePtr()->advertiseService("SetState",   &Quadrotor::RcvState, this);
        srvSetControl = GetRosNodePtr()->advertiseService("SetControl", &Quadrotor::RcvControl, this);
    }

    // Advertise this message on the ROS backbone
    pubState   = hal::model::Quadrotor::GetRosNodePtr()->template 
        advertise<hal_model_quadrotor::State>("State", DEFAULT_QUEUE_LENGTH);
    pubControl = hal::model::Quadrotor::GetRosNodePtr()->template 
        advertise<hal_model_quadrotor::Control>("Control", DEFAULT_QUEUE_LENGTH);

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

    // Create all controllers and set the default one to IDLE (on the ground, motors off)
    flightLogic.Init(GetRosNodePtr(), IDLE);
}