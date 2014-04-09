// Standard libraries
#include <hal_quadrotor/controller/Takeoff.h>

using namespace hal_quadrotor;

// Configure data broadcast at a given rate (<= 0.0 means disable)
bool Takeoff::Receive(ControlTakeoff::Request &req, ControlTakeoff::Response &res)
{
    // Eveything OK
    return true;
}

// Constructor
Takeoff::Takeoff(ros::NodeHandle &node, std::string name) 
    : Controller(TakeoffType)
{
    service = node.advertiseService(name.c_str(), &Takeoff::Receive, this);
}

// Get new control from current state and time step
Control Takeoff::Update(const State &state, const double &dt)
{
    return control;
}

// Reset the current state
void Takeoff::Reset()
{

}