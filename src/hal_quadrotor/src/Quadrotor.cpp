// Standard libraries
#include <hal_quadrotor/Quadrotor.h>

using namespace hal::quadrotor;

// Rate at which control is updated internally
#define DEFAULT_QUEUE_LENGTH   10
#define DEFAULT_UPDATE_RATE  50.0
#define DEFAULT_CONTROL_RATE  1.0
#define DEFAULT_TRUTH_RATE    1.0
#define DEFAULT_ESTIMATE_RATE 1.0

// Called on initialization
void Quadrotor::OnInit()
{
    // Initialise navigation
    actuation.Init(GetRosNodePtr(), CONTROLLER_IDLE);

    // In both experiments and simulation the state and LL control can be queried
    srvGetEstimate = GetRosNodePtr().advertiseService("GetEstimate", &Quadrotor::RcvGetEstimate, this);
    srvGetControl  = GetRosNodePtr().advertiseService("GetControl", &Quadrotor::RcvGetControl, this);

    // If we are in a simulation, additional services are avilable
    bool isSimulated = false;
    if (GetRosNodePtr().getParam("/use_sim_time",isSimulated) && isSimulated)
    {
        // Allow the true state to be mutable (hidden state in experiments)
        srvGetTruth = GetRosNodePtr().advertiseService("GetTruth", &Quadrotor::RcvGetTruth, this);
        srvSetTruth = GetRosNodePtr().advertiseService("SetTruth", &Quadrotor::RcvSetTruth, this);
        
        // Allow the low-level control to be set manually (too dangerous in experiments)
        srvSetControl = GetRosNodePtr().advertiseService("SetControl", &Quadrotor::RcvSetControl, this);
        
        // Allow the internal state exstimate to be changed (too dangerous in experiments)
        srvSetEstimate = GetRosNodePtr().advertiseService("SetEstimate", &Quadrotor::RcvSetEstimate, this);
    }

    // Publish the estimated state. For noiseless simulations, this is equal to the Truth.
    pubTruth = hal::quadrotor::Quadrotor::GetRosNodePtr().template 
        advertise<hal_quadrotor::State>("Truth", DEFAULT_QUEUE_LENGTH);

    // Publish the estimated state. For noiseless simulations, this is equal to the Truth.
    pubEstimate = hal::quadrotor::Quadrotor::GetRosNodePtr().template 
        advertise<hal_quadrotor::State>("Estimate", DEFAULT_QUEUE_LENGTH);

    // Publish the control (roll, pitch, yaw, throttle) from the low-level position controller.
    pubControl = hal::quadrotor::Quadrotor::GetRosNodePtr().template 
        advertise<hal_quadrotor::Control>("Control", DEFAULT_QUEUE_LENGTH);

    // Immediately start control loop
    timerUpdate = GetRosNodePtr().createTimer(
        ros::Duration(1.0/DEFAULT_UPDATE_RATE), 
        &Quadrotor::Update, 
        this
    );

    // Immediately start control loop
    timerTruth = GetRosNodePtr().createTimer(
        ros::Duration(1.0/DEFAULT_TRUTH_RATE), 
        &Quadrotor::BroadcastTruth, 
        this
    );

    // Immediately start control loop
    timerEstimate = GetRosNodePtr().createTimer(
        ros::Duration(1.0/DEFAULT_ESTIMATE_RATE), 
        &Quadrotor::BroadcastEstimate, 
        this
    );

    // Immediately start control loop
    timerControl = GetRosNodePtr().createTimer(
        ros::Duration(1.0/DEFAULT_CONTROL_RATE), 
        &Quadrotor::BroadcastControl, 
        this
    );
}

// This is called at a fixed rate of 50Hz
void Quadrotor::Update(const ros::TimerEvent& event)
{
    // Get the truthful state from the FCS
    GetTruth(truth);

    // Get the estimated state from the navigation engine
    navigation.GetState(estimate);

    // Find the control and whether motor arming is required
    bool arm = actuation.GetControl(
        estimate,                                  // current state
        event.current_real.toSec() - tick,         // time tick
        control                                    // resultant control
    );

    // If an arming change is needed
    if (armed != arm)
    {
        // Issue the arm change
        ArmMotors(arm);
        
        // Switch internal state
        armed = arm;
    }

    // Pass the control to the FCS and set the time it was applied
    control.t = SetControl(control);

    // Save the current time tick for the next iteration
    tick = event.current_real.toSec();
}

// Called to broadcast a control topic
void Quadrotor::BroadcastControl(const ros::TimerEvent& event)
{                  
    pubControl.publish(control);
}

// Called to broadcast a state estimate
void Quadrotor::BroadcastEstimate(const ros::TimerEvent& event)
{                  
    pubEstimate.publish(estimate);
}

// Called to broadcast a truth estimate
void Quadrotor::BroadcastTruth(const ros::TimerEvent& event)
{                  
    pubTruth.publish(truth);
}

// Constructor
Quadrotor::Quadrotor() : hal::HAL(), armed(false)
{
    // Do nothing
}

// Called when the user wants the most recent truthful state  (simulation only)
bool Quadrotor::RcvGetTruth(
    hal_quadrotor::GetTruth::Request  &req, 
    hal_quadrotor::GetTruth::Response &res)
{
    // Get the current truthful state
    res.state = truth;

    // Success!
    return true;
}

// Called when the user wants the most recent estimated state
bool Quadrotor::RcvGetEstimate(
    hal_quadrotor::GetEstimate::Request  &req, 
    hal_quadrotor::GetEstimate::Response &res)
{
    // Get the current estimated state
    res.state = estimate;

    // Success!
    return true;
}

// Called when the user wants the most recent control
bool Quadrotor::RcvGetControl(
    hal_quadrotor::GetControl::Request  &req, 
    hal_quadrotor::GetControl::Response &res)
{
    // Find the control, given the current state
    res.control = control;

    // Success!
    return true;
}

// Called when the user sets the truthful state (simulation only)
bool Quadrotor::RcvSetTruth(
    hal_quadrotor::SetTruth::Request  &req, 
    hal_quadrotor::SetTruth::Response &res)
{
    // Call the FCS to set the state
    SetTruth(req.state);

    // Notify the user
    res.success = true;
    res.status  = "Truthful state set";
    return true;
}

// Called when the user sets the state estimate (simulation only)
bool Quadrotor::RcvSetEstimate(
    hal_quadrotor::SetEstimate::Request  &req, 
    hal_quadrotor::SetEstimate::Response &res)
{
    // Get the current estimated state
    navigation.SetState(req.state);

    // Notify the user
    res.success = true;
    res.status  = "State received. Updating simulated UAV.";
    return true;
}

// Called when the user sets the control manually (simulation only)
bool Quadrotor::RcvSetControl(
    hal_quadrotor::SetControl::Request  &req, 
    hal_quadrotor::SetControl::Response &res)
{
    // Disable the current controller, as it will override manual control,
    // until a new controll goal is involed from service controllers/xyz
    actuation.Switch(CONTROLLER_DISABLED);

    // Save the control
    control = req.control;

    // Notify the user
    res.success = true;
    res.status  = "Control received. Disabled current position controller.";
    return true;
}

// Called when new altimeter data arrives
Navigation* Quadrotor::GetNavPtr()
{
    return &navigation;
}