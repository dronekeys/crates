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
    // Get the truthful state
    GetTruth(truth);

    // TODO: introduce fusion within the HAL
    GetEstimate(estimate);

    // Update control, given a state and discrete time step
    flightLogic.Update(estimate, event.current_real.toSec() - tick, control);

    // Pass the control to the HAL
    SetControl(control);

    // Save the current time tick
    tick = event.current_real.toSec();
}

void Quadrotor::BroadcastControl(const ros::TimerEvent& event)
{                  
    pubControl.publish(control);
}

void Quadrotor::BroadcastEstimate(const ros::TimerEvent& event)
{                  
    pubEstimate.publish(estimate);
}

void Quadrotor::BroadcastTruth(const ros::TimerEvent& event)
{                  
    pubTruth.publish(truth);
}

Quadrotor::Quadrotor() : hal::HAL()
{
    // Do nothing
}


bool Quadrotor::RcvGetTruth(
    hal_model_quadrotor::GetTruth::Request  &req, 
    hal_model_quadrotor::GetTruth::Response &res)
{
    // Obtain the truthful state
    res.state = truth;
    return true;
}

bool Quadrotor::RcvGetEstimate(
    hal_model_quadrotor::GetEstimate::Request  &req, 
    hal_model_quadrotor::GetEstimate::Response &res)
{
    res.state = estimate;
    return true;
}

bool Quadrotor::RcvGetControl(
    hal_model_quadrotor::GetControl::Request  &req, 
    hal_model_quadrotor::GetControl::Response &res)
{
    res.control = control;
    return true;
}

bool Quadrotor::RcvSetTruth(
    hal_model_quadrotor::SetTruth::Request  &req, 
    hal_model_quadrotor::SetTruth::Response &res)
{
    // Pass the control down to the HAL
    SetTruth(req.state);

    // Save the estimate locally
    truth = req.state;

    // Notify the user
    res.success = true;
    res.status  = "Truthful state set";
    return true;
}


bool Quadrotor::RcvSetEstimate(
    hal_model_quadrotor::SetEstimate::Request  &req, 
    hal_model_quadrotor::SetEstimate::Response &res)
{
    // Save the estimate locally
    estimate = req.state;

    // Notify the user
    res.success = true;
    res.status  = "State received. Updating simulated UAV.";
    return true;
}

bool Quadrotor::RcvSetControl(
    hal_model_quadrotor::SetControl::Request  &req, 
    hal_model_quadrotor::SetControl::Response &res)
{
    // Pass the control down to the HAL
    SetControl(control);  

    // Save the control locally
    control = req.control;

    // Notify the user
    res.success = true;
    res.status  = "Control received. Disabled current position controller.";
    return true;
}

void Quadrotor::OnInit()
{
    // In both experiments and simulation the state and LL control can be queried
    srvGetEstimate = GetRosNodePtr()->advertiseService("GetEstimate", &Quadrotor::RcvGetEstimate, this);
    srvGetControl  = GetRosNodePtr()->advertiseService("GetControl", &Quadrotor::RcvGetControl, this);

    // If we are in a simulation, additional services are avilable
    bool isSimulated = false;
    if (GetRosNodePtr()->getParam("/use_sim_time",isSimulated) && isSimulated)
    {
        // Allow the true state to be mutable (hidden state in experiments)
        srvGetTruth = GetRosNodePtr()->advertiseService("GetTruth", &Quadrotor::RcvGetTruth, this);
        srvSetTruth = GetRosNodePtr()->advertiseService("SetTruth", &Quadrotor::RcvSetTruth, this);
        
        // Allow the low-level control to be set manually (too dangerous in experiments)
        srvSetControl = GetRosNodePtr()->advertiseService("SetControl", &Quadrotor::RcvSetControl, this);
        
        // Allow the internal state exstimate to be changed (too dangerous in experiments)
        srvSetEstimate = GetRosNodePtr()->advertiseService("SetEstimate", &Quadrotor::RcvSetEstimate, this);
    }

    // Publish the estimated state. For noiseless simulations, this is equal to the Truth.
    pubTruth = hal::model::Quadrotor::GetRosNodePtr()->template 
        advertise<hal_model_quadrotor::State>("Truth", DEFAULT_QUEUE_LENGTH);

    // Publish the estimated state. For noiseless simulations, this is equal to the Truth.
    pubEstimate = hal::model::Quadrotor::GetRosNodePtr()->template 
        advertise<hal_model_quadrotor::State>("Estimate", DEFAULT_QUEUE_LENGTH);

    // Publish the control (roll, pitch, yaw, throttle) from the low-level position controller.
    pubControl = hal::model::Quadrotor::GetRosNodePtr()->template 
        advertise<hal_model_quadrotor::Control>("Control", DEFAULT_QUEUE_LENGTH);

    // Immediately start control loop
    timerUpdate = GetRosNodePtr()->createTimer(
        ros::Duration(1.0/DEFAULT_UPDATE_RATE), 
        &Quadrotor::Update, 
        this
    );

    // Immediately start control loop
    timerTruth = GetRosNodePtr()->createTimer(
        ros::Duration(1.0/DEFAULT_STATE_RATE), 
        &Quadrotor::BroadcastTruth, 
        this
    );

    // Immediately start control loop
    timerEstimate = GetRosNodePtr()->createTimer(
        ros::Duration(1.0/DEFAULT_STATE_RATE), 
        &Quadrotor::BroadcastEstimate, 
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