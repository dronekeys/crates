// Standard libraries
#include <hal_quadrotor/HAL.h>

using namespace hal_quadrotor;

// Rate at which control is updated internally
#define CONTROL_UPDATE_RATE 50.0

// This is called on every platform-level controller clock tick
void HAL::Update(const ros::TimerEvent& event)
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
        Controller::Get(Topic<State>::Get(), event.current_real.toSec()-tick)
    );

    // Update the flight control system
    Receive(Topic<Control>::Get());

    // Save the current time tick
    tick = event.current_real.toSec();
}

// Pass altiude measurement to navigation subsystem
void HAL::Receive(const Altitude &msg)
{
    nav.Measurement(msg);
}

// Pass inertial measurement to navigation subsystem
void HAL::Receive(const Inertial &msg)
{
    nav.Measurement(msg);
}

// Pass position measurement to navigation subsystem
void HAL::Receive(const Position &msg)
{
    nav.Measurement(msg);
}

// Pass magnetic measurement to navigation subsystem
void HAL::Receive(const Magnetic &msg)
{
    nav.Measurement(msg);
}

// Pass magnetic measurement to navigation subsystem
void HAL::Receive(const Energy &msg)
{
    nav.Measurement(msg);
}

// Pass orientation measurement to navigation subsystem
void HAL::Receive(const Orientation &msg)
{
    nav.Measurement(msg);
}

// Pass state measurement to navigation subsystem
void HAL::Receive(const State &msg)
{
    nav.Measurement(msg);
}

// Receive a rate configuration updates
bool HAL::ConfigRate(const std::string& name, const double& rate)
{
    return TopicBase::Config(name, rate);
}

// CONSTRUCTOR ////////////////////////////////////////////////////////////////////

HAL::HAL(const char *name) :    tick(0.0), rosNode(ros::NodeHandle(name)),
    Topic<Information>          (rosNode, std::string("information")),
    Topic<State>                (rosNode, std::string("state")),
    Topic<Control>              (rosNode, std::string("control")),
    Topic<Altitude>             (rosNode, std::string("altitude")),
    Topic<Inertial>             (rosNode, std::string("inertial")),
    Topic<Magnetic>             (rosNode, std::string("magnetic")),
    Topic<Position>             (rosNode, std::string("position")),
    Topic<Energy>               (rosNode, std::string("energy")),
    Topic<Orientation>          (rosNode, std::string("orientation")),
    Emergency                   (rosNode, std::string("emergency")),
    Idle                        (rosNode, std::string("idle")),
    Takeoff                     (rosNode, std::string("takeoff")),
    Hover                       (rosNode, std::string("hover")),
    Land                        (rosNode, std::string("land")),
    AnglesHeight                (rosNode, std::string("anglesheight")),
    Velocity                    (rosNode, std::string("velocity")),
    VelocityHeight              (rosNode, std::string("velocityheight")),
    Waypoint                    (rosNode, std::string("waypoint")),
    Rate                        (rosNode, std::string("config"))

{
    // Maske sure that ROS actually started, or there will be some issues...
    if (!ros::isInitialized())
        ROS_FATAL("A ROS node has not been initialized");

    // Immediately start control loop
    timer = rosNode.createTimer(ros::Duration(1.0/CONTROL_UPDATE_RATE), &HAL::Update, this);
}
