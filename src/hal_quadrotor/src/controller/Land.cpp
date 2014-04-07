// Standard libraries
#include <hal_quadrotor/controller/Land.h>

using namespace hal_quadrotor;

// Configure data broadcast at a given rate (<= 0.0 means disable)
bool Land::Receive(ControlLand::Request &req, ControlLand::Response &res)
{

}

// Constructor
Land::Land(ros::NodeHandle &node, std::string name) 
    : Controller(LandType)
{
    service = node.advertiseService(name.c_str(), &Land::Receive, this);
}

// Get new control from current state and time step
Control Land::Update(const State &state, const double &dt)
{
    return control;
}

// Reset the current state
void Land::Reset()
{

}