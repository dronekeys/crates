// Standard libraries
#include <hal_quadrotor/controller/Idle.h>

using namespace hal_quadrotor;

// Configure data broadcast at a given rate (<= 0.0 means disable)
bool Idle::Receive(ControlIdle::Request &req, ControlIdle::Response &res)
{

}

// Constructor
Idle::Idle(ros::NodeHandle &node, std::string name) 
    : Controller(IdleType)
{
    service = node.advertiseService(name.c_str(), &Idle::Receive, this);
}

// Get new control from current state and time step
Control Idle::Update(const State &state, const double &dt)
{
    return control;
}

// Reset the current state
void Idle::Reset()
{

}