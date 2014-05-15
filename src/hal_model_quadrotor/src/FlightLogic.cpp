#include <hal/model/FlightLogic.h>

using namespace hal::model;

FlightLogic::FlightLogic()
{
	// Do nothing
}


FlightLogic::~FlightLogic()
{
	// Do nothing
}

void FlightLogic::Init(ros::NodeHandle* nh, ControllerType controller)

{
	// Create services
	services[ANGLES_HEIGHT] = nh->advertiseService(
		"controller/AnglesHeight", &FlightLogic::RcvAnglesHeight, this);
    services[EMERGENCY] = nh->advertiseService(
    	"controller/Emergency", &FlightLogic::RcvEmergency, this);
    services[HOVER] = nh->advertiseService(
    	"controller/Hover", &FlightLogic::RcvHover, this);
   	services[IDLE] = nh->advertiseService(
   		"controller/Idle", &FlightLogic::RcvIdle, this);                
    services[LAND] = nh->advertiseService(
    	"controller/Land", &FlightLogic::RcvLand, this);                 
    services[TAKEOFF] = nh->advertiseService(
    	"controller/Takeoff", &FlightLogic::RcvTakeoff, this);
    services[VELOCITY] = nh->advertiseService(
    	"controller/Velocity", &FlightLogic::RcvVelocity, this);
    services[VELOCITY_HEIGHT] = nh->advertiseService(
    	"controller/VelocityHeight", &FlightLogic::RcvVelocityHeight, this);
    services[WAYPOINT] = nh->advertiseService(
    	"controller/Waypoint", &FlightLogic::RcvWaypoint, this);

    // Set the starting controller
    Switch(controller);
}

void FlightLogic::Switch(ControllerType controller)
{
	current = controller;
}

bool FlightLogic::Update(const hal_model_quadrotor::State &state, 
	double dt, hal_model_quadrotor::Control &control)
{
	Controller* ptr;
	switch (current)
	{
		case ANGLES_HEIGHT: 	ptr = (Controller*) &cAnglesHeight;		break;
		case EMERGENCY: 		ptr = (Controller*) &cEmergency;		break;
		case HOVER: 			ptr = (Controller*) &cHover;			break;
		case IDLE: 				ptr = (Controller*) &cIdle;				break;
		case LAND: 				ptr = (Controller*) &cLand;				break;
		case TAKEOFF: 			ptr = (Controller*) &cTakeoff;			break;
		case VELOCITY_HEIGHT: 	ptr = (Controller*) &cVelocityHeight;	break;
		case VELOCITY: 			ptr = (Controller*) &cVelocity;			break;
		case WAYPOINT: 			ptr = (Controller*) &cWaypoint;			break;
		default: 			
			return false;
	}

	// Check to see if the controller has reached its goal
	if (ptr->HasGoalBeenReached())
	{
		// At the end of takeoff / motion control, always switch to hover
		switch (current)
		{
		case TAKEOFF:
		case ANGLES_HEIGHT: 
		case VELOCITY_HEIGHT: 
		case VELOCITY: 
		case WAYPOINT:
			current = HOVER;
			ptr = (Controller*) &cHover;
			break;

		// At the end of landing, switch to idle
		case LAND:
			current = IDLE;
			ptr = (Controller*) &cIdle;
			break;
		}
	}

	// Get some control from the controller
	return ptr->Update(state,dt,control);
}

bool FlightLogic::RcvAnglesHeight(
    hal_model_quadrotor::AnglesHeight::Request  &req, 
    hal_model_quadrotor::AnglesHeight::Response &res)
{
	switch (current)
	{
	case HOVER:
	case ANGLES_HEIGHT: 
	case VELOCITY_HEIGHT: 
	case VELOCITY: 
	case WAYPOINT:
		cAnglesHeight.SetGoal(req, res);
		if (res.success)
			Switch(ANGLES_HEIGHT);
	}
	return true;
}

bool FlightLogic::RcvEmergency(
    hal_model_quadrotor::Emergency::Request  &req, 
    hal_model_quadrotor::Emergency::Response &res)
{
	cEmergency.SetGoal(req, res);
	if (res.success)
		Switch(EMERGENCY);
	return true;
}

bool FlightLogic::RcvHover(
    hal_model_quadrotor::Hover::Request  &req, 
    hal_model_quadrotor::Hover::Response &res)
{
	switch (current)
	{
	case HOVER:
	case ANGLES_HEIGHT: 
	case VELOCITY_HEIGHT: 
	case VELOCITY: 
	case WAYPOINT:
		cHover.SetGoal(req, res);
		if (res.success)
			Switch(HOVER);
	}
	return true;
}

bool FlightLogic::RcvLand(
    hal_model_quadrotor::Land::Request  &req, 
    hal_model_quadrotor::Land::Response &res)
{
	switch (current)
	{
	case HOVER:
	case ANGLES_HEIGHT: 
	case VELOCITY_HEIGHT: 
	case VELOCITY: 
	case WAYPOINT:
		cLand.SetGoal(req, res);
		if (res.success)
			Switch(LAND);
	}
	return true;
}

bool FlightLogic::RcvTakeoff(
    hal_model_quadrotor::Takeoff::Request  &req, 
    hal_model_quadrotor::Takeoff::Response &res)
{
	switch (current)
	{
	case IDLE:
		cTakeoff.SetGoal(req, res);
		if (res.success)
			Switch(TAKEOFF);
	}
	return true;
}

bool FlightLogic::RcvVelocity(
    hal_model_quadrotor::Velocity::Request  &req, 
    hal_model_quadrotor::Velocity::Response &res)
{
	switch (current)
	{
	case HOVER:
	case ANGLES_HEIGHT: 
	case VELOCITY_HEIGHT: 
	case VELOCITY: 
	case WAYPOINT:
		cVelocity.SetGoal(req, res);
		if (res.success)
			Switch(VELOCITY);
	}
	return true;
}

bool FlightLogic::RcvVelocityHeight(
    hal_model_quadrotor::VelocityHeight::Request  &req, 
    hal_model_quadrotor::VelocityHeight::Response &res)
{
	switch (current)
	{
	case HOVER:
	case ANGLES_HEIGHT: 
	case VELOCITY_HEIGHT: 
	case VELOCITY: 
	case WAYPOINT:
		cVelocityHeight.SetGoal(req, res);
		if (res.success)
			Switch(VELOCITY_HEIGHT);
	}
	return true;
}

bool FlightLogic::RcvWaypoint(
    hal_model_quadrotor::Waypoint::Request  &req, 
    hal_model_quadrotor::Waypoint::Response &res)
{
	switch (current)
	{
	case HOVER:
	case ANGLES_HEIGHT: 
	case VELOCITY_HEIGHT: 
	case VELOCITY: 
	case WAYPOINT:
		cWaypoint.SetGoal(req, res);
		if (res.success)
			Switch(WAYPOINT);
	}
	return true;
}