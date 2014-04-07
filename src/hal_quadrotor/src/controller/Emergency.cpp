// Standard libraries
#include <hal_quadrotor/controller/Emergency.h>

using namespace hal_quadrotor;

// Configure data broadcast at a given rate (<= 0.0 means disable)
bool Emergency::Receive(ControlEmergency::Request &req, ControlEmergency::Response &res)
{

}

// Constructor
Emergency::Emergency(ros::NodeHandle &node, std::string name) 
    : Controller(EmergencyType)
{
    service = node.advertiseService(name.c_str(), &Emergency::Receive, this);
}

// Get new control from current state and time step
Control Emergency::Update(const State &state, const double &dt)
{
    return control;
}

// Reset the current state
void Emergency::Reset()
{

}