// Standard libraries
#include <hal_quadrotor/Quadrotor.h>

using namespace hal_quadrotor;

// Rate at which control is updated internally
#define CONTROL_UPDATE_RATE 50.0

// This is called on every platform-level controller clock tick
void Quadrotor::Update(const ros::TimerEvent& event)
{
    // Get the current state 
    Topic<State>::Set(
        nav.GetState()
    );

    // Get the current orientation
    Topic<Orientation>::Set(
        nav.GetOrientation()
    );

    // Determine the required control to satisfy action, given current state
    Topic<Control>::Set(
        Controller::Get(Topic<State>::Get(), event.current_real.toSec() - tick)
    );

    // Update the flight control system
    Control(Topic<Control>Get());

    // Save the current time tick
    tick = event.current_real.toSec();
}

// Pass altiude measurement to navigation subsystem
void Quadrotor::Receive(const Altitude &msg)
{
    nav.Measurement(msg);
}

// Pass inertial measurement to navigation subsystem
void Quadrotor::Receive(const Inertial &msg)
{
    nav.Measurement(msg);
}

// Pass position measurement to navigation subsystem
void Quadrotor::Receive(const Position &msg)
{
    nav.Measurement(msg);
}

// Pass magnetic measurement to navigation subsystem
void Quadrotor::Receive(const Magnetic &msg)
{
    nav.Measurement(msg);
}

// Pass orientation measurement to navigation subsystem
void Quadrotor::Receive(const Orientation &msg)
{
    nav.Measurement(msg);
}

// Pass state measurement to navigation subsystem
void Quadrotor::Receive(const State &msg)
{
    nav.Measurement(msg);
}

// Pass the conf
void Quadrotor::ConfigRate(const std::string &name, const double* rate)
{
    Topic::Config(name, rate);
}

// CONSTRUCTOR ////////////////////////////////////////////////////////////////////

Quadrotor::Quadrotor(ros::NodeHandle& node, const std::string &name) : 

    // Default time is zero
    tick(0.0),

    // Buffered topics
    Topic<Information>          (node, std::string("information")),
    Topic<State>                (node, std::string("state")),
    Topic<Control>              (node, std::string("control")),
    Topic<Altitude>             (node, std::string("altitude")),
    Topic<Inertial>             (node, std::string("inertial")),
    Topic<Magnetic>             (node, std::string("magnetic")),
    Topic<Position>             (node, std::string("position")),
    Topic<Orientation>          (node, std::string("orientation")),

    // Services (controllers)
    Emergency                   (node, std::string("emergency")),
    Idle                        (node, std::string("idle")),
    Takeoff                     (node, std::string("takeoff")),
    Hover                       (node, std::string("hover")),
    Land                        (node, std::string("land")),
    AnglesHeight                (node, std::string("anglesheight")),
    Velocity                    (node, std::string("velocity")),
    VelocityHeight              (node, std::string("velocityheight")),
    Waypoint                    (node, std::string("waypoint")),
    
    // Services (configurators)
    Rate                        (node, std::string("config"))

{
    // Maske sure that ROS actually started, or there will be some issues...
    if (!ros::isInitialized())
        ROS_FATAL("A ROS node has not been initialized");

    // Immediately start control loop
    timer = node.createTimer(ros::Duration(1.0/CONTROL_UPDATE_RATE), &Quadrotor::Update, this);
}
