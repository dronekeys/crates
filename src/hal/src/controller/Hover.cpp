// Standard libraries
#include <hal_quadrotor/controller/Hover.h>

using namespace hal_quadrotor;

// Configure data broadcast at a given rate (<= 0.0 means disable)
bool Hover::Receive(ControlHover::Request &req, ControlHover::Response &res)
{
    // Eveything OK
    return true;
}

// Constructor
Hover::Hover(ros::NodeHandle &node, std::string name) 
    : Controller(HoverType)
{
    service = node.advertiseService(name.c_str(), &Hover::Receive, this);
}

// Get new control from current state and time step
Control Hover::Update(const State &state, const double &dt)
{
    return control;
}

// Reset the current state
void Hover::Reset()
{

}